import logging
from vesicolos_utils import getkey, Keys

# TODO: CLI stores the positions, should invoke a thread
# to continuously save them, and then provide a way to return them
# in principle, a GUI could derive from this,
class CLI:
    def __init__ (self, motor_controller=None, monitor=None, led=None, heater=None, camera=None, keymap={}, movement_map={}):
        self.motor_controller = motor_controller
        if motor_controller is not None:
            self.motors = motor_controller._servos
        else:
            self.motors = None
        self.monitor = monitor
        self.led = led
        self.heater = heater
        self.camera = camera
        self.keymap = keymap
        #self.motor_moving = False # FIXME
        if movement_map:
            for k,m in movement_map.items():
                self.keymap[k] = (CLI.movement, m['axis'], m['dir'])
        self.log = logging.getLogger("VESICOLOS UI")
        # TODO FIXME
        # provide factory methods to make loggers with common formatting
        # sanity check
        for feature, featname, dependency in [
            (self.motors,"motors",[CLI.movement,CLI.stop_all,CLI.store_position,CLI.recall_position]),
            (self.led,"LED",CLI.toggle_led),
            (self.heater,"heater",CLI.toggle_heater)
            ]:
            if feature is None:
                self.log.warn(f"feature {featname} not configured")
                for k,m in self.keymap:
                    if m[0] in dependency:
                        self.keymap[k] = (CLI.notimpl,)
    def __enter__ (self):
        return self
    def __exit__ (self, exc_type, exc_value, traceback):
        pass
    def start (self, stop_event):
        """Start the UI loop, processing key strokes and performing the
        relevant actions. Interrupted if `stop_event.is_set()` from the
        main thread."""
        self.stop_event = stop_event
        while not stop_event.is_set():
            ch = getkey()
            if ch is None:
                # TODO FIXME implement motor-moving timeout here?
                #if motor_moving and motor_timeout > 0:
                #    motor_timeout -= 1
                #    if motor_timeout <= 0:
                #        if self.motors and self.motor_controller.stop_all():
                #            motor_timeout = MOTOR_TIMEOUT
                continue
            if ch == Keys.ESC:
                self.stop = True
                print("GOOD-BYE")
                self.log.info("user exit (esc)")
                stop_event.set()
                continue
            if ch in self.keymap:
                func, *args = (*self.keymap[ch],)
                func(*args)
            else:
                self.user_help()
    def notimpl(self):
        print ("NOT IMPLEMENTED / CONFIGURED")
    def user_help(self):
        """help"""
        for ch in self.keymap:
            if ch<32 or ch>255:
                chmap = next(k.name for k in reversed(Keys) if k==ch)
            else:
                chmap = chr(ch)
            print("{k:8.8s} - {doc}".format(k=chmap,doc=self.keymap[ch].__doc__))
        print("___")
        for i in range(1,5):
            poskey = 'savepos'+str(i)
            if poskey in POSITIONS:
                print (' ',poskey,POSITIONS[poskey])

    def movement (self, ax, direction):
        """move x / y / z-position"""
        vel = self.motor_controller.current_set_speed.get(ax,0) \
            + direction * self.motor_controller.motorconf.get(ax,{}).get('SPEED_INC',0)
        res = self.motors[ax].sram.write_running_speed(vel)
        if not (vel==0):
            # if we tried to set vel=0 but failed, this will keep motor_moving=True
            self.motor_moving = True
            rvel = self.motors[ax].sram.read_current_speed()
            self.log.debug(f"{ax} set_vel = {vel}, read_vel = {rvel}")
    def stop_all (self):
        res = self.motor_controller.stop_all()
        if res:
            log.debug("motors stop")
    def store_position (self, num):
        """store position"""
        self.motor_controller.stop_all()
        if num == 0:
            poskey = 'homepos'
            reset_wrap = True
        else:
            poskey = 'savepos'+str(num)
            reset_wrap = False
        success = self.monitor.update_pos()
        if success:
            # TODO FIXME global variables
            POSITIONS[poskey] = {}
            for ax in motors.axes:
                if reset_wrap:
                    monitor.wrap[ax] = 0
                POSITIONS[poskey][ax] = (monitor.pos[ax],monitor.wrap[ax])
            print ('saved',poskey,monitor.pos,monitor.wrap)
            last_savepos = poskey
    # TODO the following are FIXME
    def recall_position(self, num):
        """recall stored position"""
        if num == 0:
            poskey = 'homepos'
        else:
            poskey = 'savepos'+str(num)
        if not (poskey in POSITIONS):
            print ('no position',poskey,'saved')
        else:
            self.motors.stop_all()
            time.sleep(0.2)
            if self.monitor: self.monitor.stop()
            #TODO FIXME
            #motors.move_to_stored_position(poskey)
            if self.monitor: self.monitor.start()
            last_savepos = poskey
    def goto_position(self, key):
        """goto a specific position (servo mode)"""
        self.motors.stop_all()
        ax = input('axis? ').upper()
        posstr = input('position (servo mode)? ')
        if not ax in axes:
            print ("axis not found")
            return
        try:
            pos = int(posstr)
        except ValueError:
            print ("illegal position")
            return
        motorDriver.WheelMode(SERVOS[ax]['ID'], False)
        time.sleep(0.2)
        motorDriver.GotoPos(SERVOS[ax]['ID'], pos)
        time.sleep(0.2)
        motorDriver.WheelMode(SERVOS[ax]['ID'], True)
    def query_position(self, key):
        """query motor positions"""
        self.motors.stop_all()
        axes = motors.axes
        wheelpos = { _: 0 for _ in axes }
        servopos = { _: 0 for _ in axes }
        for ax in axes:
            servo = motors._servos[ax]
            wheelpos[ax] = servo.sram.read_current_location()
            time.sleep(0.2)
            servo.eeprom.write_operating_mode(0)
            time.sleep(0.2)
            servopos[ax] = servo.sram.read_current_location()
            time.sleep(0.2)
            servo.eeprom.write_operating_mode(1)
        print ("current position (wheel)",wheelpos)
        print ("current position (servo)",servopos)
    def toggle_led(self):
        """toggle LED"""
        self.led.toggle()
        log.info('LED {}'.format(['OFF','ON'][self.led.is_active]))
    def toggle_heater(self):
        """toggle heater"""
        self.heater.toggle()
        log.info(f"HEATER PWM active {self.heater.is_active} value {self.heater.value}")
    def enter_temperature_ramp(key):
        """enter temperature parameters"""
        tkey = last_savepos or 'default'
        print ("enter temperature ramp parameters for",tkey,\
               "(empty for default)")
        if tkey in TEMPERATURES:
            tkey_defaults = tkey
        else:
            tkey_defaults = 'default'
        Tmin, Tmax, dt, ts, tmax = (TEMPERATURES[tkey_defaults][_] \
            for _ in ['Tmin','Tmax','dt','ts','tmax'])
        Tminstr = input("Tmin = [{}]".format(Tmin))
        Tmaxstr = input("Tmax = [{}]".format(Tmax))
        dtstr = input("dt [{}] = ".format(dt))
        tsstr = input("ts [{}] = ".format(ts))
        tmaxstr = input("tmax [{}] = ".format(tmax))
        try:
            Tmin, Tmax, dt, ts, tmax = float(Tminstr), float(Tmaxstr), \
                float(dtstr), float(tsstr), float(tmaxstr)
            TEMPERATURES[tkey] = { 'Tmin': Tmin, 'Tmax': Tmax, \
                                   'dt': dt, 'ts': ts, 'tmax': tmax }
        except ValueError:
            print("illegal input, ignoring")
    def liftoff(self):
        """manual lift off"""
        self.log.info("MANUAL LIFT OFF")
        self.stop_event.set()
    def camera(self):
        """user camera recording"""
        if self.camera is not None:
            camera.stop()
            camera = None
        else:
            ckey = last_savepos or 'launch'
            ckey += '{i:02d}'
            i = 1
            while ckey.format(i=i) in recordings:
                i += 1
            ckey = ckey.format(i=i)
            recordings.append(ckey)
            try:
                camera = CameraController(camfile,pts=ptsfile,keys={'pos':ckey})
                threading.Thread(target=camera.record).start()
            except RuntimeError as e:
                camera = None
                print("ERROR starting camera",str(e))
