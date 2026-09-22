from python_st3215 import ST3215, Servo, ServoNotRespondingError
from python_st3215.instructions import Instruction
from typing import Optional, Sequence
import logging
import time
import threading

from serial import PortNotOpenError
from . import Word16

#import ansi

class myServo(Servo):
    MODE = 0x21
    mode_names = { 0: "position", 1: "wheel", 2: "PWM", 3: "stepper" }
    MODE_SERVO = 0
    MODE_WHEEL = 1
    ACC = 0x29
    GOAL_SPEED = 0x2E
    PRESENT_POSITION = 0x38
    PRESENT_SPEED = 0x3A
    PRESENT_LOAD = 0x3C
    PRESENT_VOLTAGE = 0x3E
    PRESENT_TEMPERATURE = 0x3F
    MOVING = 0x42
    PRESENT_CURRENT = 0x45
    def WriteRunningSpeed(self, vel):
        return self.sram.write_running_speed(vel)
    def SyncWriteRunningSpeed(self, servo_data: dict[int, int]) -> None:
        return self.controller.broadcast.sram.sync_write_running_speed(servo_data)
    def ReadPresentSpeed(self):
        return self.sram.read_current_speed()
    def SyncReadPresentSpeed(self, servo_ids):
        return self.controller.broadcast.sram.sync_read_current_speed(servo_ids)
    def ReadPresentPosition(self):
        return self.sram.read_current_location()
    def SyncReadPresentPosition(self, servo_ids):
        return self.controller.broadcast.sram.sync_read_current_location(servo_ids)
    def ReadPresentLoad(self):
        return self.sram.read_current_load()
    def SyncReadPresentLoad(self, servo_ids):
        return self.controller.broadcast.sram.sync_read_current_load(servo_ids)

    def WriteTargetPosition(self, pos):
        # copied this from the old code, but maybe this was not needed
        # and we simply need to wait for the movement to finish
        # problem: negative positions were not what I think they were
        acc = 50
        lo, hi = Word16(pos,bitsigned=True,safe_bound=True,bigendian=True).to_bytes()
        return self._write_memory(self.ACC,[acc,lo,hi,0,0,0,0])
        return self.sram.write_target_location(pos)
    def ReadPresentVoltage(self):
        """Return current voltage in V."""
        b = self._read_memory(self.PRESENT_VOLTAGE, 1)
        if b is not None:
            return int(b)/10.
        return None
    def ReadPresentTemperature(self):
        b = self._read_memory(self.PRESENT_TEMPERATURE, 1)
        if b is not None:
            return int(b)
        return None
    def isMoving(self):
        b = self._read_memory(self.MOVING, 1)
        if b is not None:
            return b>0
        return None
    def getOperatingMode(self):
        b = self._read_memory(self.MODE, 1)
        if b is not None:
            return self.mode_names.get(int(b),'UNKNOWN')
        return None
    def setWheelMode(self, wheel=True):
        mode = self.MODE_WHEEL if wheel else self.MODE_SERVO
        return self._write_memory(self.MODE, mode)
    def setMiddle(self):
        return self.sram.correct_position_to_2048()
    def enableTorque(self):
        return self.sram.torque_enable()



class myST3215(ST3215):
    def __init__(self, port: str, baudrate: int = 1000000,
                 read_timeout: float = 0.002) -> None:
        super().__init__(port, baudrate, read_timeout)
        self.broadcast = myServo(self, 254)

    def wrap_servo(self, servo_id: int) -> myServo:
        """
        Create a Servo instance for the given servo ID after verifying it responds to ping.
        Returns:
            Servo: An instance of the myServo class for the given ID.
        Raises:
            ServoNotRespondingError: If the servo does not respond to ping.
        """
        parsed = self.ping(servo_id)
        if not parsed or parsed.get("error") != 0:
            raise ServoNotRespondingError(
                f"Servo ID {servo_id} did not respond to ping."
            )
        return myServo(self, servo_id)

class Motors:
    ST_STEPS = 4096
    ST_MAX_WRAPS = 7
    ST_MIDDLE = 2048


class MotorController:
    def __init__ (self, device, log, axes_map={}, motorconf={}, max_id=253):
        self.log = log
        self.serial_lock = threading.Lock()
        self.current_set_speed = {}
        try:
            self.scan(device, axes_map, motorconf, max_id)
        except Exception as e:
            self.log.error("motor init failed: "+str(e))
            self.controller = None
            self.axes = []
            self._servos = {}
    def scan (self, device, axes_map, motorconf, max_id):
        self.controller = myST3215(device)
        self.log.info("Scanning for servos.")
        old_level = self.controller.logger.level
        self.controller.logger.setLevel(logging.ERROR)
        self.axes = []
        self.axes_map = {}
        self._servos = {}
        self.servo_ids = self.controller.list_servos(end_id=max_id)
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
                self.log.info(f"{infostr}SERVO ID {servo_id} pos {servo.ReadPresentPosition()} mode {servo.getOperatingMode()}")
                self.log.debug(f"SERVO ID {servo_id} U={servo.ReadPresentVoltage():.1f}V, T={servo.ReadPresentTemperature()}°C")
                self._servos[servo_id] = servo
                if axis:
                    self._servos[axis] = servo
                    self.current_set_speed[axis] = servo.ReadPresentSpeed()
            found_all_axes = True
            for ax in sorted(list(set(axes_map.values()))):
                if ax in self._servos:
                    self.axes.append(ax)
                    self.axes_map[self._servos[ax].id] = ax
                else:
                    found_all_axes = False
            if not found_all_axes:
                self.log.error("unexpected servo configuration found, continuing anyway")
            else:
                self.log.debug("axes map "+str(self.axes_map))
        #DEBUG code to check what happens when we call a servo that isn't there
        #try:
        #    from python_st3215 import Servo
        #    self._servos[10] = Servo(controller=self.controller,servo_id=10)
        #except Exception as e:
        #    print(e)
        #    pass
        #print(self._servos)

        self.motorconf = motorconf
        # sanitize motorconf: copy over defaults to the individual axes
        # unless they override it
        motorconf_default = self.motorconf.get('_default_',{})
        for ax in self.axes:
            if not ax in self.motorconf:
                self.motorconf[ax] = {}
            for key in ['SPEED_INC', 'MAX_TORQUE']:
                if not key in self.motorconf[ax]:
                    self.motorconf[ax][key] = motorconf_default.get(key,0)
        # set torque limit for the motors
        self.torque_control(enable=True)

    def __enter__ (self):
        return self
    def __exit__ (self, exc_type, exc_value, traceback):
        self.log.debug("stopping all motors")
        self.stop_all()
        if self.controller:
            self.wheel_mode()
            self.torque_control(enable=False)
            self.log.debug("closing motor controller")
            self.controller.close()
    def stop_all (self):
        """Stop all servos"""
        if not self.controller: return False
        success = True
        with self.serial_lock:
            self.controller.broadcast.SyncWriteRunningSpeed(
                {servo.id: 0 for servo in self._servos.values()}
            )
            # use broadcast sync_read_current_speed for return?
            for ax in self.current_set_speed:
                self.current_set_speed[ax] = 0
                vel = self._servos[ax].ReadPresentSpeed()
                if vel is None or vel != 0:
                    #self.current_set_speed[ax] = vel
                    self.log.error("motor set speed 0 failed?"+str(vel))
                    success =False
        return success
        #return sum(map(abs,speeds))==0
    def set_speed (self, axis, vel, return_read=False):
        if not axis in self.axes:
            return None, None
        with self.serial_lock:
            res = self._servos[axis].WriteRunningSpeed(vel)
            if res and not res['error']:
                self.current_set_speed[axis] = vel
            read_vel = None
            if return_read:
                read_vel = self._servos[axis].ReadPresentSpeed()
        return res, read_vel
    def get_speed (self, axis=''):
        if not self.controller:
            return None
        if not axis:
            with self.serial_lock:
                vel = self.controller.broadcast.SyncReadPresentSpeed(self.axes_map.keys())
            vel = {self.axes_map[i]: vel[i] for i in vel}
            return vel
        elif axis in self.axes:
            with self.serial_lock:
                vel = self._servos[axis].ReadPresentSpeed()
            return vel
        return None
    def goto_position (self, axis, pos, return_read=False, wait_moving=True):
        if not axis in self.axes:
            return None, None
        self.torque_control(axis,enable=False)
        with self.serial_lock:
            res = self._servos[axis].WriteTargetPosition(pos)
            if res and not res['error']:
                pass
            if wait_moving:
                timeout = 120
                start_time = time.time()
                while self._servos[axis].isMoving():
                    if time.time() - start_time > timeout:
                        print("moving timeout")
                        break
                    time.sleep(0.05)
            read_pos = None
            if return_read:
                read_pos = self._servos[axis].ReadPresentPosition()
        self.torque_control(axis,enable=True)
        return res, read_pos
    def read_position (self, axis=''):
        if not self.controller:
            return None
        if not axis:
            with self.serial_lock:
                pos = self.controller.broadcast.SyncReadPresentPosition(self.axes_map.keys())
            pos = {self.axes_map[i]: pos[i] for i in pos}
            return pos
        elif axis in self.axes:
            with self.serial_lock:
                pos = self._servos[axis].ReadPresentPosition()
            return pos
        return None
    def read_torque (self, axis=''):
        if not self.controller:
            return None
        if not axis:
            with self.serial_lock:
                torque = self.controller.broadcast.SyncReadPresentLoad(self.axes_map.keys())
            torque = {self.axes_map[i]: torque[i] for i in torque}
            return torque
        elif axis in self.axes:
            with self.serial_lock:
                torque = self._servos[axis].ReadPresentLoad()
            return torque
        return None
    def wheel_mode (self, axis='', wheel=True):
        """Set motor specified by `axis` to wheel mode.
        If `axis==''` (default), apply to all axes."""
        mode = 1 if wheel else 0
        if not axis:
            with self.serial_lock:
                # TODO broadcast
                for ax in self.axes:
                    self._servos[ax].setWheelMode(wheel)
        elif axis in self.axes:
            with self.serial_lock:
                self._servos[axis].setWheelMode(wheel)
    def torque_control (self, axis='', enable=True):
        """Set motor specified by `axis` to torque control."""
        if not axis:
            with self.serial_lock:
                for ax in self.axes:
                    if enable:
                        self._servos[ax].sram.torque_enable()
                        self._servos[ax].sram.write_torque_limit(
                                self.motorconf[ax]['MAX_TORQUE'])
                    else:
                        self._servos[ax].sram.torque_disable()
        elif axis in self.axes:
            with self.serial_lock:
                if enable:
                    self._servos[axis].sram.torque_enable()
                    self._servos[axis].sram.write_torque_limit(
                            self.motorconf[axis]['MAX_TORQUE'])
                else:
                    self._servos[axis].sram.torque_disable()
    def set_middle (self, axis=''):
        """Reset motor position specified by `axis` to middle."""
        if not axis:
            with self.serial_lock:
                # TODO broadcast?
                for ax in self.axes:
                    self._servos[ax].setMiddle()
        elif axis in self.axes:
            with self.serial_lock:
                self._servos[axis].setMiddle()
    def move_to_position (self, target_pos, wrap):
        # FIXME TODO DEBUG
        axes_ = ['Z'] # axes
        """Move all motors to the positions given in `target_pos`, taking into
        account the wrap-around counters.
        `target_pos` needs to be a dict with keys corresponding to the
        axes, each storing a tuple of `(pos,wrap)` targets."""
        # current and target positions are in wheel mode
        # we can also get the current position in servo mode
        # to go to a defined position:
        # 1. calculate delta(target,current) in wheel mode
        # 2. go to servo mode, set current as middle position 2048
        #    this does not change the wheel-mode position
        # 3. move to 2048+delta
        #    possibly first a multiple of 7 turns if delta too large
        # 4. go to wheel mode
        try:
            current_pos = self.read_position()
        except Exception as err:
            self.log.error("move_to_position0: cannot read position: "+str(err))
            return None
        current_wrap = wrap
        target_wrap = { _: target_pos[_][1] for _ in self.axes }
        target_pos = { _: target_pos[_][0] for _ in self.axes }
        print('current',current_pos,current_wrap)
        print('target ',target_pos,target_wrap)
        for ax in axes_:
            try:
                print("set servo mode")
                self.wheel_mode(ax, wheel=False)
                print("servo mode done")
                time.sleep(0.2)
                print("set middle")
                self.set_middle(ax)
                print("set middle done")
            except Exception as err:
                self.log.error(f"move_to_position {ax} failed middle: "+str(err))
                continue
            delta_pos = target_pos[ax] - current_pos[ax]
            delta_wrap = target_wrap[ax] - current_wrap[ax]
            print('need delta',delta_pos,delta_wrap)
            if 'MAX_WRAP' in self.motorconf[ax]:
                max_wrap = self.motorconf[ax]['MAX_WRAP']
                if abs(delta_wrap) > self.motorconf[ax]['MAX_WRAP']:
                    self.log.error(f"axis {ax} should not move by {delta_wrap} turns, limiting to {max_wrap}")
                if delta_wrap > 0:
                    delta_wrap = max_wrap
                elif delta_wrap < 0:
                    delta_wrap = -max_wrap
            if abs(delta_wrap) >= Motors.ST_MAX_WRAPS:
                direction = 1
                if delta_wrap < 0:
                    direction = -1
                time.sleep(0.2)
                try:
                    self.goto_position(ax,Motors.ST_MIDDLE+direction*Motors.ST_STEPS*(Motors.ST_MAX_WRAPS-1))
                    time.sleep(0.2)
                    self.set_middle(ax)
                except Exception as err:
                    self.log.error(f"move_to_position failed unwrapping {ax}"+str(err))
                    continue
                delta_wrap -= direction*(Motors.ST_MAX_WRAPS-1)
            print('need delta',delta_pos,delta_wrap)
            dpos = delta_pos + Motors.ST_STEPS*delta_wrap
            try:
                print("at position",self.read_position(ax))
            except Exception as err:
                pass
            print("dpos",dpos)
            try:
                print("goto_position",Motors.ST_MIDDLE+dpos)
                self.goto_position(ax,Motors.ST_MIDDLE+dpos)
                time.sleep(0.2)
                print("set wheel mode")
                self.wheel_mode(ax, wheel=True)
                print("done")
                time.sleep(0.2)
            except Exception as err:
                self.log.error(f"move_to_position {ax} failed: "+str(err))
        try:
            newpos = self.read_position()
            newwrap = target_wrap
            for ax in axes_:
                if newpos[ax] < 100 and target_pos[ax] > Motors.ST_STEPS-100:
                    newwrap[ax] += 1
                if newpos[ax] > Motors.ST_STEPS-100 and target_pos[ax] > Motors.ST_STEPS-100:
                    newwrap[ax] -= 1                        
            return target_wrap
        except Exception as err:
            self.log.error('move to target: '+str(err))
        return None


# TODO the following needs updating 
# this is a general monitor also for temperature
# monitor for servo positions
# we also output signal status here for convenience
# also output temperature readings here for convenience -> this is just
# a global "monitor", maybe move to own file
class ServoMonitor():
    def __init__ (self, motors, temp_sensor, led, heater, log, increment, silent=False, state={}, global_status={}):
        self.t_init = time.time()
        self.next_t = self.t_init
        self.t_init_str = time.ctime(self.t_init)
        self.silent = silent
        self.done = False
        self.motors = motors
        self.temp_sensor = temp_sensor
        self.heater = heater
        self.led = led
        self.log = log
        self.increment = increment
        self.pos = state.get('motor.pos',{})
        self.wrap = state.get('motor.wrap',{})
        success, pos, self.vel, self.torque = self.read_pos_vel()
        # for pos, check if we have stored state variables
        # if we do, the positions must match, else the hardware is not
        # in a state that the state variables think it is, and this is
        # potentially dangerous
        self.state_valid = True
        for ax in self.motors.axes:
            if ax in self.pos:
                if success and not (abs(self.pos[ax]-pos[ax])<5):
                    self.motors.log.error(f"{ax} axis mismatch of position: restart {self.pos[ax]} / current {pos[ax]}")
                    self.pos[ax] = pos[ax]
                    self.wrap[ax] = 0
                    self.state_valid = False
            else:
                self.pos[ax] = pos[ax]
            if not ax in self.wrap:
                self.wrap[ax] = 0
        self.status = global_status
        self.statusbar = print
        self._run()
    def _run (self):
        if not self.done:
            success = self.update_pos()
            if not self.silent:
                if self.heater and self.heater.is_active:
                    es = 'HEAT'
                else:
                    es = ''
                if self.led and self.led.is_active:
                    es += ' LED'
                if not success:
                    es += ' ERR'
                try:
                    with open('/sys/class/thermal/thermal_zone0/temp','r') as f:
                        cpu_temp = int(f.read())
                except:
                    cpu_temp = '-1000'
                cpu_temp = float(cpu_temp/1000.)
                if self.temp_sensor is not None:
                    sample_temp = self.temp_sensor.temperature
                else:
                    sample_temp = -1
                if self.pos is not None:
                    posinfo = '  '.join([ ax + f' {self.pos[ax] or -1:4d} ({self.wrap[ax]:2d})' for ax in self.pos ])
                else:
                    posinfo = 'ERROR'
                if self.vel is not None:
                    velinfo = '  '.join([ ax + f' {vel}' for ax,vel in self.vel.items() ])
                else:
                    velinfo = 'ERROR'
                velsetinfo = '  '.join([ ax + f' {vset:4d}' for ax,vset in self.motors.current_set_speed.items()])
                torqueinfo = '  '.join([ ax + f' {trq:4d}' for ax,trq in self.torque.items() ])
                flags = ' '.join([f"{k} {int(v)}" for k,v in self.status.items()])
                self.statusbar(
                        f"| CPU T={cpu_temp:.2f} SAMPLE T={sample_temp:.2f} D {self.t_init_str} +{int(self.next_t-self.t_init):5d}s {flags} {es}\n"
                        f"| POS {posinfo} TRQ {torqueinfo}\n"
                        f"| VEL {velinfo} SET {velsetinfo}"
                )
                self.log.info(f"{sample_temp} {posinfo} {velinfo} {flags}")
            while self.next_t < time.time():
                self.next_t += self.increment
            threading.Timer(self.next_t - time.time(), self._run).start()
    def read_pos_vel (self):
        success = True
        try:
            newpos = self.motors.read_position()
            newvel = self.motors.get_speed()
            newtrq = self.motors.read_torque()
        except PortNotOpenError as e:
            success = False
            newpos = { _:None for _ in self.motors.axes }
            newvel = { _:None for _ in self.motors.axes }
            newtrq = { _:None for _ in self.motors.axes }
            pass
        return success, newpos, newvel, newtrq
    # update: query the motor positions, try to detect wrap-arounds
    def update_pos (self,detect_wrap=True):
        success, newpos, newvel, newtrq = self.read_pos_vel()
        if success:
            for ax in self.motors.axes:
                if detect_wrap:
                    if self.pos[ax] > Motors.ST_STEPS-Motors.ST_STEPS/4 and newpos[ax] < Motors.ST_STEPS/4:
                        self.wrap[ax] += 1
                    elif self.pos[ax] < Motors.ST_STEPS/4 and newpos[ax] > Motors.ST_STEPS-Motors.ST_STEPS/4:
                        self.wrap[ax] -= 1
                self.pos[ax] = newpos[ax]
                self.vel[ax] = newvel[ax]
                self.torque[ax] = newtrq[ax]
        return success
    def stop (self):
        self.done = True
        print("servo monitor stopped")
    def start (self, silent=False):
        self.next_t = time.time()
        self.silent = silent
        self.done = False
        self._run()




