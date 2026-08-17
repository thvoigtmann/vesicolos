import time

def temperature_ramp (t, Tmin=0., Tmax=0., dt=0., tstart=0.):
    """Linear temperature ramp, defined like
       T(t) = Tmin                                  for t < tstart
       T(t) = Tmin + (Tmax - Tmin)*(t-tstart)/dt    for tstart < t < tstart+dt
       T(t) = Tmax                                  for tstart+dt < t
    """
    tau = t - tstart
    if tau <= 0:
        return Tmin
    if tau >= dt:
        return Tmax
    return Tmin + (Tmax - Tmin) * tau/dt

Tcontrols = {
    'ramp': temperature_ramp,
    'default': temperature_ramp
}

class TemperatureController:
    # TODO: make this carry its own logfile handle, init with fname
    # this is not really a logger, it is just an open file
    # maybe pass just the fh here so that we can embed in with ... as fh
    # or with TemperatureController(logfile) as Temp:
    # TODO FIXME: if no 'default' in TEMPERATURES: turn off temp control
    #def __init__ (self, heater, sensor, temperature_profiles, tfunc, tfunc_signature, log):
    def __init__ (self, heater, sensor, temperature_profiles, log):
        self.profiles = temperature_profiles
        self.heater = heater
        self.sensor = sensor
        self.log = log
        self.stop = False
        self.params = None
        self.set_profile ('default')
    def __enter__ (self):
        return self
    def __exit__ (self, exc_type, exc_value, traceback):
        self.stop = True
        if self.heater is not None:
            self.heater.off()
    def set_profile (self, profilename):
        """A profile should be a dict containing a 'type' keyword
        to pick the type of temperature control function, and
        a list of parameters to that function.
        If `profilename` is not in the profiles dictionary, the
        `default` profile will be chosen."""
        self.t0 = time.time()
        if not profilename in self.profiles:
            profilename = 'default'
        if profilename in self.profiles:
            profile = self.profiles[profilename]
            profiletype = profile.get('type', 'default')
            self.Tfunc = Tcontrols.get(profiletype, temperature_ramp)
            signature = inspect.signature(self.Tfunc).parameters
            # user-supplied parameters: all those in the profile
            # that are also valid function arguments
            params = { k,v for k,v in profile.items() if k in signature.keys() }
            # default parameters
            defaults = { k,v.default for k,v in signature.items() if v.default is not inspect.Parameter.empty }
            # merge, making sure user-defined overwrite default
            self.params = { **defaults, **params }
            self.log.info(f"temperature profile {profilename} set at t0 = {self.t0}")
            self.log.info(', '.join(["{} = {}".format(key,val) \
                    for key,val in self.params.items()]))
        else:
            self.log.error(f"no temperature profile {profilename}")
            self.params = None
    def start (self, prog_end):
        if not self.heater or not self.sensor:
            self.log.error("no heater/sensor -> no temperature control")
            return
        while not self.stop and not prog_end.is_set():
            if self.params:
                t = time.time() - self.t0
                Ttarget = self.Tfunc(t, **self.params)
                Tcurrent = self.sensor.temperature
                if Tcurrent < Ttarget:
                    self.heater.on()
                elif Tcurrent >= Ttarget:
                    # we cannot currently cool
                    # we could use PWM to heat less once we get near target
                    # but this is not implemented yet
                    self.heater.off()
                self.log.info(\
                    "t - t0 = {}, T = {}, Ttarget = {}, heat {}" \
                    .format(t,Tcurrent,Ttarget,self.heater.is_active))
            time.sleep(1)
