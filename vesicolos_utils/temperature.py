class TemperatureController:
    # TODO: make this carry its own logfile handle, init with fname
    # this is not really a logger, it is just an open file
    # maybe pass just the fh here so that we can embed in with ... as fh
    # or with TemperatureController(logfile) as Temp:
    # TODO FIXME: if no 'default' in TEMPERATURES: turn off temp control
    def __init__ (self, heater, sensor, temperature_profiles, tfunc, tfunc_signature, log):
        self.profiles = temperature_profiles
        self.set_profile ('default')
        self.heater = heater
        self.sensor = sensor
        self.Tfunc = tfunc
        self.signature = tfunc_signature
        self.log = log
        self.stop = False
    def __enter__ (self):
        return self
    def __exit__ (self, exc_type, exc_value, traceback):
        self.stop = True
        if self.heater is not None:
            self.heater.off()
    def set_profile (self, profilename):
        self.params = []
        self.t0 = time.time()
        pname = profilename
        if not pname in self.profiles:
            pname = 'default'
        if pname in self.profiles:
            profile = self.profiles[pname]
            for param in self.signature:
                self.params.append(profile.get(param,0))
            self.log.info(f"temperature profile {profilename} set")
            self.log.info(', '.join(["{} = {}".format(key,val) \
                    for key,val in zip(self.signature,self.params)]))
        else:
            self.log.error(f"no temperature profile {profilename}")
    def start (self, prog_end):
        if not self.heater or not self.sensor:
            self.log.error("no heater/sensor -> no temperature control")
            return
        while not self.stop and not prog_end.is_set():
            if self.params:
                t = time.time()
                Ttarget = self.Tfunc(t, *self.params)
                Tcurrent = self.sensor.temperature
                if Tcurrent < Ttarget:
                    self.heater.on()
                elif Tcurrent >= Ttarget:
                    # we cannot currently cool
                    # we could use PWM to heat less once we get near target
                    # but this is not implemented yet
                    self.heater.off()
                self.log.info(\
                    "t0 = {}, t = {}, T = {}, Ttarget = {}, heat {}" \
                    .format(self.t0,t,Tcurrent,Ttarget,self.heater.is_active))
            time.sleep(1)
