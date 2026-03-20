from python_st3215 import ST3215
import logging
import time
import threading

from . import Word16

import ansi


class Motors:
    ST_STEPS = 4096
    ST_MAX_WRAPS = 7
    ST_MIDDLE = 2048


class MotorController:
    mode_names = { 0: "position", 1: "wheel", 2: "PWM", 3: "stepper" }
    def __init__ (self, device, log, axes_map={}, motorconf={}):
        self.log = log
        self.current_set_speed = {}
        try:
            self.scan(device, axes_map, motorconf)
        except Exception as e:
            self.log.error("motor init failed: "+str(e))
            self.controller = None
            self.axes = []
            self._servos = {}
    def scan (self, device, axes_map, motorconf):
        self.controller = ST3215(device)
        self.log.info("Scanning for servos.")
        old_level = self.controller.logger.level
        self.controller.logger.setLevel(logging.ERROR)
        self.axes = []
        self._servos = {}
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

        self.motorconf = motorconf
        # sanitize motorconf
        motorconf_default = self.motorconf.get('_default_',{})
        for ax in self.axes:
            if not ax in self.motorconf:
                self.motorconf[ax] = {}
            for key in ['SPEED_INC']: # we rely on the presence of those
                if not key in self.motorconf[ax]:
                    self.motorconf[ax][key] = motorconf_default.get(key,0)

    def __enter__ (self):
        return self
    def __exit__ (self, exc_type, exc_value, traceback):
        self.log.debug("stopping all motors")
        self.stop_all()
        self.wheel_mode()
        if self.controller:
            self.log.debug("closing motor controller")
            self.controller.close()
    def stop_all (self):
        """Stop all servos"""
        if not self.controller: return False
        success = True
        self.controller.broadcast.sram.sync_write_running_speed(
                {servo.id: 0 for servo in self._servos.values()}
        )
        # use broadcast sync_read_current_speed for return?
        for ax in self.current_set_speed:
            self.current_set_speed[ax] = 0
            vel = self._servos[ax].sram.read_current_speed()
            if vel is None or vel != 0:
                #self.current_set_speed[ax] = vel
                self.log.error("motor set speed 0 failed?"+str(vel))
                success =False
        return success
        #return sum(map(abs,speeds))==0
    def set_speed (self, axis, vel, return_read=False):
        if axis in self.axes:
            #self._servos[axis].sram.write_acceleration(0)
            #self._servos[axis].sram.write_target_location(0)
            #self._servos[axis].sram.write_runtime(0)
            #res = self._servos[axis].sram.write_running_speed(vel)
            lo, hi = Word16(vel,bigendian=True,safe_bound=True).to_bytes()
            res = self._servos[axis]._write_memory(0x2E, [low, high])
            # TODO FIXME try copying what WriteSpec did
            # write [acc, 0, 0, 0, 0, lobyte(speed), hibyte(speed)]
            # where speed is a signed word
            if res and not res['error']:
                self.current_set_speed[axis] = vel
        read_vel = None
        if return_read:
            read_vel = self._servos[axis].sram.read_current_speed()
        return res, read_vel
    def get_speed (self, axis):
        if axis in self.axes:
            vel = self._servos[axis].sram.read_current_speed()
            return vel
        return None
    # TODO
    def goto_position (self, axis, pos, return_read=False):
        # TODO FIXME: this should be just the sram call to set the position
        # similar to set_speed
        # the old code would use WritePosEx
        # wrting [acc, lo(pos), hi(pos), 0, 0, lo(speed), hi(speed)]
        # with speed set to zero I guess
        # and negative positions were written as
        # pos = (-pos) | (1<<15)
        # which I do not understand yet
        pass
    def read_position (self, axis=-1):
        if axis<0:
            pos = {}
            for ax in self.axes:
                pos[ax] = self._servos[ax].sram.read_current_location()
            return pos
        elif axis in self.axes:
            pos = self._servos[axis].sram.read_current_location()
            return pos
    def wheel_mode (self, axis=-1, wheel=True):
        """Set motor specified by `axis` to wheel mode.
        If `axis==-1` (default), apply to all axes."""
        mode = 1 if wheel else 0
        if axis<0:
            for ax in self.axes:
                self._servos[ax].eeprom.write_operating_mode(mode)
        elif axis in self.axes:
            self._servos[axis].eeprom.write_operating_mode(mode)
    # TODO FIXME
    def set_middle (self, axis):
        pass
    def move_to_position (self, target_pos, wrap):
        """Move all motors to the positions given in `target_pos`, taking into
        account the wrap-around counters `wrap`.
        `target_pos` needs to be a dict with keys corresponding to the
        axes, each storing a tuple of `(pos,wrap)` targets.
        `wrap` needs to be a dict with axes as keys, indicating the
        current wrap counter for that axis."""
        # to go to a defined position:
        # 1. calculate delta in wheel mode
        # 2. go to servo mode, set current as middle position 2048
        # 3. move to 2048+delta
        #    possibly first a multiple of 7 turns if delta too large
        # 4. go to wheel mode
        try:
            success = self.monitor.update_pos()
            if not success:
                raise Exception("failed updating position")
            current_pos = self.monitor.pos
            current_wrap = wrap
            target_pos = { _: target_pos[_][0] for _ in self.axes }
            target_wrap = { _: target_pos[_][1] for _ in self.axes }
            print('current',current_pos,current_wrap)
            print('target ',target_pos,target_wrap)
            for ax in axes:
                self.wheel_mode(ax, wheel=False)
                time.sleep(0.2)
                # TODO FIXME handle exception
                #comm, err = motorDriver.SetMiddle(scs_id)
                #if not motorDriver.success(comm, err):
                #    raise Exception("failed setting motor reference pos")
                delta_pos = target_pos[ax] - current_pos[ax]
                delta_wrap = target_wrap[ax] - current_wrap[ax]
                print('need delta',delta_pos,delta_wrap)
                if 'MAX_WRAP' in self.motorconf[ax]:
                    max_wrap = self.motorconf[ax]['MAX_WRAP']
                    if abs(delta_wrap) > self.motorconf[ax]['MAX_WRAP']:
                        self.log.error(f"axis {ax} should not move by {delta_wrap} turns, limiting to {max_wrap}")
                    if delta_wrap > 0:
                        delta_wrap = max_wrap
                    else:
                        delta_wrap = -max_wrap
                if abs(delta_wrap) >= Motors.ST_MAX_WRAPS:
                    direction = 1
                    if delta_wrap < 0:
                        direction = -1
                    time.sleep(0.2)
                    #FIXME
                    #success = motorDriver.GotoPos(scs_id, \
                    #      ST_MIDDLE + direction*ST_STEPS*(ST_MAX_WRAPS-1))
                    #if not success:
                    #    raise Exception("failed unwrapping")
                    self.set_middle(ax) # FIXME
                    #if not motorDriver.success(comm, err):
                    #    raise Exception("failed re-setting motor reference pos")
                    delta_wrap -= direction*(Motors.ST_MAX_WRAPS-1)
                dpos = delta_pos + Motors.ST_STEPS*delta_wrap
                self.goto_position(ax,Motors.ST_MIDDLE+dpos)
                #if not success: FIXME
                #    raise Exception("failed moving motor by delta steps")
                time.sleep(0.2)
                self.wheel_mode(ax, wheel=True)
                time.sleep(0.2)
                self.monitor.update_pos()
                self.monitor.wrap[ax] = target_wrap[ax]
                if self.monitor.pos[ax] < 100 and target_pos[ax] > Motors.ST_STEPS-100:
                    self.monitor.wrap[ax] += 1
                if self.monitor.pos[ax] > Motors.ST_STEPS-100 and target_pos[ax] > Motors.ST_STEPS-100:
                    self.monitor.wrap[ax] -= 1                        
        except Exception as err:
            log.error('move to target: '+str(err))


# TODO the following needs updating 
# TODO FIXME we have a lot of motors._servos[ax] codes
# -> factor these into the Motors class?
# -> not this is a general monitor also for temperature
# monitor for servo positions
# we also output signal status here for convenience
# also output temperature readings here for convenience -> this is just
# a global "monitor", maybe move to own file
class ServoMonitor():
    def __init__ (self, motors, increment, silent=False, state={}):
        self.next_t = time.time()
        self.silent = silent
        self.done = False
        self.motors = motors
        self.log = self.motors.log # TODO FIXME create own logger?
        self.increment = increment
        self.pos = state.get('motor.pos',{})
        self.wrap = state.get('motor.wrap',{})
        success, pos, self.vel = self.read_pos_vel()
        # for pos, check if we have stored state variables
        # if we do, the positions must match, else the hardware is not
        # in a state that the state variables think it is, and this is
        # potentially dangerous
        self.state_valid = True
        for ax in self.motors.axes:
            if ax in self.pos:
                if success and not (self.pos[ax] == pos[ax]):
                    self.log.error(f"{ax} axis mismatch of position: restart {self.pos[ax]} / current {pos[ax]}")
                    self.pos[ax] = pos[ax]
                    self.wrap[ax] = 0
                    self.state_valid = False
            else:
                self.pos[ax] = pos[ax]
            if not ax in self.wrap:
                self.wrap[ax] = 0
        self._run()
    def _run (self):
        if not self.done:
            success = self.update_pos()
            if not self.silent:
                if success: es=''
                else: es='EE'
                #print(ansi.cursor.save_cursor()+ansi.cursor.goto(10,10)+es+" -- position",self.pos,self.wrap,self.vel,"--",ansi.cursor.load_cursor(0),end='')
                print(" -- pos,wrap",self.pos,self.wrap,"--")
                print("  - vel,set",self.vel,self.motors.current_set_speed,"--")
                # TODO FIXME
                #log.info(str(status)+" T="+str(temp_sensor.temperature))
            while self.next_t < time.time():
                self.next_t += self.increment
            threading.Timer(self.next_t - time.time(), self._run).start()
    def read_pos_vel (self):
        success = True
        newpos = { _:0 for _ in self.motors.axes }
        newvel = { _:0 for _ in self.motors.axes }
        for ax in self.motors.axes:
            try:
                newpos[ax] = self.motors._servos[ax].sram.read_current_location()
                newvel[ax] = self.motors._servos[ax].sram.read_current_speed()
            except:
                success = False
            if newpos[ax] is None or newvel[ax] is None:
                success = False
            #TODO FIXME how to read comm error?
            #success &= motorDriver.success(comm, err)
            #how to handle this
        return success, newpos, newvel
    # update: query the motor positions, try to detect wrap-arounds
    def update_pos (self,detect_wrap=True):
        success, newpos, newvel = self.read_pos_vel()
        if success:
            for ax in self.motors.axes:
                if detect_wrap:
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
        print("servo monitor stopped")
    def start (self, silent=False):
        self.next_t = time.time()
        self.silent = silent
        self.done = False
        self._run()




