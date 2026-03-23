import logging, time
from vesicolos_utils import getkey, Keys, make_camera_key



# in principle, a GUI could derive from this class
class CLI:
    def __init__ (self, motor_controller=None, monitor=None, led=None, heater=None, keymap={}, movement_map={}, state={}):
        self.motor_controller = motor_controller
        if motor_controller is not None:
            self.motors = motor_controller._servos
        else:
            self.motors = None
        self.monitor = monitor
        self.led = led
        self.heater = heater
        self.camera = None
        self.recordings = []
        self.keymap = keymap
        if movement_map:
            for k,m in movement_map.items():
                self.keymap[k] = (CLI.movement, m['axis'], m['dir'])
        self.log = logging.getLogger("VESICOLOS UI")
        # TODO FIXME
        # provide factory methods to make loggers with common formatting
        # sanity check
        for feature, featname, dependency in [
            (self.motors,"motors",[CLI.movement,CLI.stop_all,CLI.store_position,CLI.recall_position]),
            (self.led,"LED",[CLI.toggle_led]),
            (self.heater,"heater",[CLI.toggle_heater])
            ]:
            if feature is None or not feature:
                self.log.warn(f"feature {featname} not configured")
                for k,m in self.keymap.items():
                    if m[0] in dependency:
                        self.keymap[k] = (CLI.notimpl,f"({featname} missing)")
        #self.motor_moving = False # FIXME
        self.stop = False
        self.last_savepos = None
        # if the caller included positions and temperatures as state variables
        # the following hopefully points us to those global arrays(!?)
        # then the automatic periodic state-saving will include these
        self.stored_positions = state.get('user.positions', {})
        self.stored_temperatures = state.get('user.temperatures', {})
    def __enter__ (self):
        return self
    def __exit__ (self, exc_type, exc_value, traceback):
        if self.stop and self.camera:
            self.camera.stop()
            self.camera = None
        pass
    def start (self, stop_event):
        """Start the UI loop, processing key strokes and performing the
        relevant actions. Interrupted if `stop_event.is_set()` from the
        main thread."""
        self.stop = False
        self.stop_event = stop_event
        while not stop_event.is_set():
            ch = getkey()
            if ch is None:
                # TODO FIXME implement motor-moving timeout here?
                # this could also be the job of the monitor
                # (who would need to have this set off after LO or not?)
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
                func(self,*args)
            else:
                self.user_help()
    def notimpl(self, errmsg):
        """not implemented"""
        print ("NOT IMPLEMENTED / CONFIGURED:",errmsg)
    def user_help(self):
        """help"""
        for ch in self.keymap:
            if ch<32 or ch>255:
                chmap = next(k.name for k in reversed(Keys) if k==ch)
            else:
                chmap = chr(ch)
            func, *args = (*self.keymap[ch],)
            if args:
                args = ' '+' '.join([str(_) for _ in args])
            else:
                args = ''
            print("{k:8.8s} - {doc}{args}".format(k=chmap,doc=self.keymap[ch][0].__doc__,args=args))
        print("___")
        for i in range(1,5):
            poskey = 'savepos'+str(i)
            if poskey in self.stored_positions:
                print (' ',poskey,self.stored_positions[poskey])
        print("___")
        if not self.camera is None:
            print ('  camera is recording ('+self.recordings[-1]+')')
    def movement (self, ax, direction):
        """move x / y / z-position"""
        if not ax in self.motor_controller.axes:
            return
        vel = self.motor_controller.current_set_speed[ax] \
            + direction * self.motor_controller.motorconf[ax]['SPEED_INC']
        res, rvel = self.motor_controller.set_speed(ax, vel, return_read=True)
        # TODO FIXME do we need motor_moving? should set here if needed
        self.log.debug(f"{ax} set_vel = {vel}, read_vel = {rvel}")
    def stop_all (self):
        """stop all motors"""
        res = self.motor_controller.stop_all()
        if res:
            self.log.debug("motors stop")
    def store_position (self, num):
        """store position"""
        self.motor_controller.stop_all()
        time.sleep(0.2)
        if num == 0:
            poskey = 'homepos'
            reset_wrap = True
        else:
            poskey = 'savepos'+str(num)
            reset_wrap = False
        success = self.monitor.update_pos()
        if success:
            self.stored_positions[poskey] = {}
            for ax in self.motor_controller.axes:
                if reset_wrap:
                    self.monitor.wrap[ax] = 0
                self.stored_positions[poskey][ax] = (self.monitor.pos[ax],self.monitor.wrap[ax])
            print ('saved',poskey,self.monitor.pos,self.monitor.wrap)
            self.last_savepos = poskey
    def recall_position(self, num):
        """recall stored position"""
        if num == 0:
            poskey = 'homepos'
        else:
            poskey = 'savepos'+str(num)
        if not (poskey in POSITIONS):
            print ('no position',poskey,'saved')
        else:
            self.motor_controller.stop_all()
            time.sleep(0.2)
            self.monitor.stop()
            #TODO FIXME
            self.motor_controller.move_to_stored_position(self.stored_positions[poskey])
            self.monitor.start()
            self.last_savepos = poskey
    def goto_position(self):
        """goto a specific position (servo mode)"""
        self.motor_controller.stop_all()
        self.monitor.stop()
        ax = input('axis? ').upper()
        if not ax in self.motor_controller.axes:
            print ("axis not found")
            return
        posstr = input('position (servo mode)? ')
        try:
            pos = int(posstr)
        except ValueError:
            print ("illegal position")
            return
        self.motor_controller.wheel_mode(ax,False)
        time.sleep(0.2)
        self.motor_controller.goto_position(ax,pos,wait_moving=False)
        time.sleep(0.2)
        self.motor_controller.wheel_mode(ax,True)
        self.monitor.start()
    def set_velocity (self):
        """set the velocity of a motor by hand"""
        self.motor_controller.stop_all()
        self.monitor.stop()
        ax = input('axis ?').upper()
        if not ax in self.motor_controller.axes:
            print ("axis not found")
            return
        velstr = input('velocity? ')
        try:
            vel = int(velstr)
        except ValueError:
            print ("illegal input")
            return
        res, rvel = self.motor_controller.set_speed(ax, vel, return_read=True)
        print("rvel",rvel)
        print("res",res)
        self.monitor.start()
    def query_position(self):
        """query motor positions"""
        self.motor_controller.stop_all()
        self.monitor.stop()
        wheelpos = self.motor_controller.read_position()
        time.sleep(0.2)
        self.motor_controller.wheel_mode(axis='',wheel=False)
        time.sleep(0.2)
        servopos = self.motor_controller.read_position()
        time.sleep(0.2)
        self.motor_controller.wheel_mode(axis='',wheel=True)
        print ("current position (wheel)",wheelpos)
        print ("current position (servo)",servopos)
        self.monitor.start()
    def toggle_led(self):
        """toggle LED"""
        self.led.toggle()
        self.log.info('LED {}'.format(['OFF','ON'][self.led.is_active]))
    def toggle_heater(self):
        """toggle heater"""
        self.heater.toggle()
        self.log.info(f"HEATER PWM active {self.heater.is_active} value {self.heater.value}")
    def enter_temperature_ramp(self):
        """enter temperature parameters"""
        # TODO FIXME does this work!?
        global T_SIGNATURE
        if self.motors:
            self.motor_controller.stop_all()
        self.monitor.stop()
        tkey = self.last_savepos or 'default'
        print ("enter temperature ramp parameters for",tkey,\
               "(empty for default)")
        if tkey in self.stored_temperatures:
            tkey_defaults = tkey
        else:
            tkey_defaults = 'default'
        Tparam = T_SIGNATURE
        defaults, inputs = {}, {}
        fail = False
        for t in Tparam:
            defaults[t] = self.stored_temperatures.get(tkey_defaults,{}).get(t,0)
            instr = input("{t} = [{d}]".format(t=t,d=defaults[t]))
            if not instr:
                inputs[t] = defaults[t]
            else:
                try:
                    inputs[t] = float(instr)
                except ValueError:
                    print("illegal input, ignoring")
                    fail = True
                    break
        if not fail:
            self.stored_temperatures[tkey] = inputs
            print("temperature ramp",tkey,self.stored_temperatures[tkey])
        self.monitor.start()
    def liftoff(self):
        """manual lift off"""
        self.log.info("MANUAL LIFT OFF")
        self.stop_event.set()
    def toggle_camera(self):
        """toggle user camera recording"""
        if self.camera is not None:
            self.camera.stop()
            self.camera = None
        else:
            ckey = make_camera_key (self.recordings, self.last_savepos or 'launch', pre='user_')
            self.recordings.append(ckey)
            try:
                self.camera = CameraController(camfile,pts=ptsfile,keys={'pos':ckey})
                threading.Thread(target=self.camera.record).start()
            except RuntimeError as e:
                self.camera = None
                self.log.error("could not start camera: "+str(e))
    def read_user_settings (self):
        return self.stored_positions, self.stored_temperatures



# USER-INTERFACE KEYBOARD MAPPING
# the movement keys will be automatically mapped, they are not defined here
keymap = {
    ord('0'): (CLI.stop_all,),
    Keys.F1: (CLI.store_position,1),
    Keys.F2: (CLI.store_position,2),
    Keys.F3: (CLI.store_position,3),
    Keys.F4: (CLI.store_position,4),
    Keys.F5: (CLI.store_position,5),
    ord('1'): (CLI.recall_position,1),
    ord('2'): (CLI.recall_position,2),
    ord('3'): (CLI.recall_position,3),
    ord('4'): (CLI.recall_position,4),
    ord('5'): (CLI.recall_position,5),
    Keys.INSERT: (CLI.store_position,0),
    Keys.HOME: (CLI.recall_position,0),
    ord('q'): (CLI.query_position,),
    ord('g'): (CLI.goto_position,),
    ord('v'): (CLI.set_velocity,),
    ord('l'): (CLI.toggle_led,),
    ord('h'): (CLI.toggle_heater,),
    ord('t'): (CLI.enter_temperature_ramp,),
    ord('x'): (CLI.liftoff,),
    ord('c'): (CLI.toggle_camera,),
    ord('?'): (CLI.user_help,)
}


