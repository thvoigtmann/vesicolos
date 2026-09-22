import picamera2
from libcamera import controls

class CameraController ():
    def __init__ (self, filename, pts=None, keys={}):
        self.picam = None
        self.stop_ = False
        self.imgpath = filename
        self.imgpath_keys = keys
        self.ptsfile = pts
        self.picam = picamera2.Picamera2()
        self.config = self.picam.create_video_configuration()
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
        print("START cam recording",imgfile)
        # should have some sort of interruptible loop here if we write
        # single frames ourselves... (because our stop() method should
        # be callable)
    def stop (self):
        self.stop_ = True
        if self.picam:
            self.picam.stop_recording()
            print("STOP cam recording")
            self.picam.close()

class CameraStream ():
    def __init__ (self, target_ip="0.0.0.0", udp_port=3333):
        self.picam = picamera2.Picamera2()
        cdn_off = controls.draft.NoiseReductionModeEnum.Off
        video_config = self.picam.create_video_configuration(
                main={'size':(640,480)},
                controls={"FrameDurationLimits": (33333,33333),
                          "NoiseReductionMode": cdn_off})
        self.picam.configure(video_config)
        self.encoder = picamera2.encoders.H264Encoder(repeat=True,iperiod=15)
        self.output = picamera2.outputs.FfmpegOutput(f"-f h264 udp://{target_ip}:{udp_port}", audio=False)
        self.encoder.output = [output]
    def start (self):
        self.picam.start_recording(self.encoder, self.output)
    def stop (self):
        self.picam.stop_recording()
