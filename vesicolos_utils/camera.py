import picamera2
#from libcamera import controls

#cdn_off = controls.draft.NoiseReductionModeEnum.Off
cdn_off = 0
camcontrols = {
    "NoiseReductionMode": cdn_off,
    #"AeEnable": False,
    #"FrameDurationLimits": (50000,50000),
    #"ExposureTime": (48000),
    "AnalogueGain": 48.0,
    "Contrast": 1.5,
}

class CameraController ():
    def __init__ (self, filename, pts=None, keys={}, log=None):
        self.picam = None
        self.stop_ = False
        self.log = log
        self.imgpath = filename
        self.imgpath_keys = keys
        self.ptsfile = pts
        self.picam = picamera2.Picamera2()
        self.config = self.picam.create_video_configuration(
                main={'size':(1920,1080)},
                #main={'size':(3840,2160)},
                controls=camcontrols)
        self.picam.configure(self.config)
        self.encoder = picamera2.encoders.H264Encoder(10000000)
    def __del__ (self):
        self.stop()
    def record (self):
        if not self.picam:
            return
        frame = 0 # would only be needed if we write single frames ourselves
        imgfile = self.imgpath.format(**{'frame':frame,**self.imgpath_keys})
        pts = self.ptsfile.format(**self.imgpath_keys)
        self.picam.start_recording(self.encoder, imgfile, pts=pts)
        if self.log is not None:
            self.log.info("START cam recording "+imgfile)
        # should have some sort of interruptible loop here if we write
        # single frames ourselves... (because our stop() method should
        # be callable)
    def stop (self):
        self.stop_ = True
        if self.picam:
            self.picam.stop_recording()
            if self.log is not None:
                self.log.info("STOP cam recording")
            self.picam.close()
            self.picam = None

class CameraStream ():
    def __init__ (self, target_ip="0.0.0.0", udp_port=3333, log=None):
        self.log = log
        self.picam = picamera2.Picamera2()
        video_config = self.picam.create_video_configuration(
                main={'size':(640,480)},
                controls=camcontrols)
        self.picam.configure(video_config)
        self.encoder = picamera2.encoders.H264Encoder(repeat=True,iperiod=15)
        self.output = picamera2.outputs.FfmpegOutput(f"-f h264 udp://{target_ip}:{udp_port}", audio=False)
        self.encoder.output = [self.output]
    def __del__ (self):
        self.stop()
    def start (self):
        self.picam.start_recording(self.encoder, self.output)
        if self.log is not None:
            self.log.info("START videostream")
    def stop (self):
        if self.picam is not None:
            self.picam.stop_recording()
            if self.log is not None:
                self.log.info("START videostream")
            self.picam.close()
            self.picam = None
