# camera controller, used later
class CameraController ():
    def __init__ (self, filename, pts=None, keys={}):
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
        self.picam.stop_recording()
        print("STOP cam recording")
        self.picam.close()



