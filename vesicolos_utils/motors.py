from python_st3215 import ST3215
import logging
import time
import threading


import ansi


class Motors:
    ST_STEPS = 4096
    ST_MAX_WRAPS = 7
    ST_MIDDLE = 2048
    mode_names = { 0: "position", 1: "wheel", 2: "PWM", 3: "stepper" }
    def __init__ (self, device, log, axes_map={}):
        self.controller = ST3215(device)
        self.log = log
        self.log.info("Scanning for servos.")
        old_level = self.controller.logger.level
        self.controller.logger.setLevel(logging.ERROR)
        self.axes = []
        self._servos = {}
        self.current_set_speed = {}
        self.servo_ids = self.controller.list_servos()
        if not self.servo_ids:
            self.log.error("no servos found")
        else:
            self.controller.logger.setLevel(old_level)
            self.log.info("found %d servos" % len(self.servo_ids))
            for servo_id in self.servo_ids:
                servo = self.controller.wrap_servo(servo_id)
                if servo_id in axes_map:
                    axis = axes_map[servo_id]
                    infostr = axis + " axis "
                else:
                    axis = None
                    infostr = ''
                self.log.info(f"{infostr}SERVO ID {servo_id} pos {servo.sram.read_current_location()} mode {self.mode_names.get(servo.eeprom.read_operating_mode(),'UNKNOWN')}")
                self.log.debug(f"SERVO ID {servo_id} U={servo.sram.read_current_voltage() / 10:.1f}V, T={servo.sram.read_current_temperature()}°C")
                self._servos[servo_id] = servo
                if axis:
                    self._servos[axis] = servo
                    self.current_set_speed[axis] = servo.sram.read_current_speed()
            found_all_axes = True
            for ax in sorted(list(set(axes_map.values()))):
                if ax in self._servos:
                    self.axes.append(ax)
                else:
                    found_all_axes = False
            if not found_all_axes:
                self.log.err("unexpected servo configuration found, continuing anyway")
        #DEBUG code to check what happens when we call a servo that isn't there
        #try:
        #    from python_st3215 import Servo
        #    self._servos[10] = Servo(controller=self.controller,servo_id=10)
        #except Exception as e:
        #    print(e)
        #    pass
        #print(self._servos)
    def __enter__ (self):
        return self
    def __exit__ (self, exc_type, exc_value, traceback):
        self.log.debug("stopping all motors")
        self.stop_all()
        self.wheel_mode()
        self.log.debug("closing motor controller")
        self.controller.close()
    def stop_all (self):
        """Stop all servos"""
        success = True
        self.controller.broadcast.sram.sync_write_running_speed(
                {servo.id: 0 for servo in self._servos.values()}
        )
        for ax in self.current_set_speed:
            vel = self._servos[ax].sram.read_current_speed()
            if vel is not None:
                self.current_set_speed[ax] = vel
            if vel != 0:
                success = False
        return success
        #return sum(map(abs,speeds))==0
    def wheel_mode (self, axis=-1):
        """Set motor specified by `axis` to wheel mode.
        If `axis==-1` (default), apply to all axes."""
        if axis<0:
            for ax in self.axes:
                self._servos[ax].eeprom.write_operating_mode(1)
        elif axis in self.axes:
            self._servos[axis].eeprom.write_operating_mode(1)


# the stuff that follows here would be ideally in the Servo API?
def ServoWheelMode (servo):
    servo.eeprom.write_operating_mode(1)
    #servo.sram.torque_enable()


# TODO the following needs updating 
# TODO FIXME we have a lot of motors._servos[ax] codes
# -> factor these into the Motors class?
# monitor for servo positions
# we also output signal status here for convenience
class ServoMonitor():
    def __init__ (self, motors, increment, silent=False):
        self.next_t = time.time()
        self.silent = silent
        self.done = False
        self.motors = motors
        self.increment = increment
        self.pos = { _:0 for _ in motors.axes }
        self.vel = { _:0 for _ in motors.axes }
        self.wrap = { _:0 for _ in motors.axes }
        self._run()
    def _run (self):
        if not self.done:
            success = self.update_pos()
            if not self.silent:
                if success: es=''
                else: es='EE'
                print(ansi.cursor.save_cursor()+ansi.cursor.goto(10,10)+es+" -- position",self.pos,self.wrap,self.vel,"--",ansi.cursor.load_cursor(0),end='')
                # TODO FIXME
                #log.info(str(status)+" T="+str(temp_sensor.temperature))
            self.next_t += self.increment
            threading.Timer(self.next_t - time.time(), self._run).start()
    # update: query the motor positions, try to detect wrap-arounds
    def update_pos (self,detect_wrap=True):
        success = True
        newpos = { _:0 for _ in self.motors.axes }
        newvel = { _:0 for _ in self.motors.axes }
        for ax in self.motors.axes:
            newpos[ax] = self.motors._servos[ax].sram.read_current_location()
            if newpos[ax] is None:
                success = False
            newvel[ax] = self.motors._servos[ax].sram.read_current_speed()
            if newvel[ax] is None:
                success = False
            #TODO FIXME how to read comm error?
            #success &= motorDriver.success(comm, err)
        if success and detect_wrap:
            for ax in self.motors.axes:
                set_vel = self.motors.current_set_speed[ax]
                if set_vel>0 and self.pos[ax] > Motors.ST_STEPS/2 and newpos[ax] < self.pos[ax]:
                    self.wrap[ax] += 1
                if set_vel<0 and self.pos[ax] < Motors.ST_STEPS/2 and newpos[ax] > self.pos[ax]:
                    self.wrap[ax] -= 1
                if set_vel==0:
                    if newpos[ax] < 100 and self.pos[ax] > Motors.ST_STEPS-100:
                        self.wrap[ax] += 1
                    if newpos[ax] > Motors.ST_STEPS-100 and self.pos[ax] < 100:
                        self.wrap[ax] -= 1
                self.pos[ax] = newpos[ax]
                self.vel[ax] = newvel[ax]
        return success
    def stop (self):
        self.done = True
    def start (self, silent=False):
        self.next_t = time.time()
        self.silent = silent
        self.done = False
        self._run()


#TODO the following should be part of Motors class
# and be called not with a poskey, but a position already
# (or rather a map ax:(pos,wrap) to move all axes, and a separate ax:wrap
# to tell the motors which wrap we want to assume for them)

# a key point here is to be able to move the motors, store some
# positions of interest, and be able to return to them automatically
# that latter part is a bit tricky, it is wrapped in its own function:
def move_to_stored_position (poskey):
    # to go to a defined position:
    # 1. calculate delta in wheel mode
    # 2. go to servo mode, set current as middle position 2048
    # 3. move to 2048+delta
    #    possibly first a multiple of 7 turns if delta too large
    # 4. go to wheel mode
    try:
        success = monitor.update_pos()
        if not success:
            raise Exception("failed updating position")
        current_pos = monitor.pos
        target_pos = { _: POSITIONS[poskey][_][0] for _ in axes }
        current_wrap = monitor.wrap
        target_wrap = { _: POSITIONS[poskey][_][1] for _ in axes }
        print('current',current_pos,current_wrap)
        print('target ',target_pos,target_wrap)
        for ax in axes:
            scs_id = SERVOS[ax]['ID']
            motorDriver.WheelMode(scs_id,False)
            time.sleep(0.2)
            comm, err = motorDriver.SetMiddle(scs_id)
            if not motorDriver.success(comm, err):
                raise Exception("failed setting motor reference pos")
            delta_pos = target_pos[ax] - current_pos[ax]
            delta_wrap = target_wrap[ax] - current_wrap[ax]
            print('need delta',delta_pos,delta_wrap)
            if 'MAX_WRAP' in SERVOS[ax] \
                and abs(delta_wrap) > SERVOS[ax]['MAX_WRAP']:
                log.err(f"axis {ax} should not move by {delta_wrap} turns, limiting")
                if delta_wrap > 0:
                    delta_wrap = SERVOS[ax]['MAX_WRAP']
                else:
                    delta_wrap = -SERVOS[ax]['MAX_WRAP']
            if abs(delta_wrap) >= ST_MAX_WRAPS:
                direction = 1
                if delta_wrap < 0:
                    direction = -1
                time.sleep(0.2)
                success = motorDriver.GotoPos(scs_id, \
                          ST_MIDDLE + direction*ST_STEPS*(ST_MAX_WRAPS-1))
                if not success:
                    raise Exception("failed unwrapping")
                comm, err = motorDriver.SetMiddle(scs_id)
                if not motorDriver.success(comm, err):
                    raise Exception("failed re-setting motor reference pos")
                delta_wrap -= direction*(ST_MAX_WRAPS-1)
            dpos = delta_pos + ST_STEPS*delta_wrap
            success = motorDriver.GotoPos(scs_id, ST_MIDDLE+dpos)
            if not success:
                raise Exception("failed moving motor by delta steps")
            time.sleep(0.2)
            motorDriver.WheelMode(scs_id,True)
            time.sleep(0.2)
            monitor.update_pos()
            monitor.wrap[ax] = target_wrap[ax]
            if monitor.pos[ax] < 100 and target_pos[ax] > ST_STEPS-100:
                monitor.wrap[ax] += 1
            if monitor.pos[ax] > ST_STEPS-100 and target_pos[ax] > ST_STEPS-100:
                monitor.wrap[ax] -= 1                        
    except Exception as err:
        log.err(str(err))


