from python_st3215 import ST3215, Servo, ServoNotRespondingError
from python_st3215.instructions import Instruction
from typing import Optional, Sequence
import logging
import time
import threading

from . import Word16

import ansi

class myServo(Servo):
    MODE = 0x21
    mode_names = { 0: "position", 1: "wheel", 2: "PWM", 3: "stepper" }
    MODE_SERVO = 0
    MODE_WHEEL = 1
    GOAL_SPEED = 0x2E
    PRESENT_POSITION = 0x38
    PRESENT_SPEED = 0x3A
    PRESENT_VOLTAGE = 0x3E
    PRESENT_TEMPERATURE = 0x3F
    MOVING = 0x42
    PRESENT_CURRENT = 0x45
    def SyncReadWord(self, servo_ids, cmd):
        responses: dict[int, dict[str, Any] | None]= self._sync_read(
                cmd, 2, servo_ids
        )
        results: dict[int, int | None] = {}
        for servo_id, response in responses.items():
            if response and isinstance(response,dict) and response.get("parameters"):
                data: list[int] | bytes = response["parameters"]
                results[servo_id] = int(Word16(0, bitsigned=True, bigendian=True).from_bytes(data[0],data[1]))
            else:
                results[servo_id] = None
        return results
    def WriteRunningSpeed(self, vel):
        lo, hi = Word16(vel,bigendian=True,bitsigned=True,safe_bound=True).to_bytes()
        return self._write_memory(self.GOAL_SPEED, [lo, hi])
    def SyncWriteRunningSpeed(self, servo_data: dict[int, int]) -> None:
        formatted_data: dict[int, list[int]] = {}
        for servo_id, value in servo_data.items():
            lo, hi = Word16(value,bigendian=True,bitsigned=True,safe_bound=True).to_bytes()
            formatted_data[servo_id] = [lo, hi]
        return self._sync_write(self.GOAL_SPEED, 2, formatted_data) # type: ognore
    def ReadPresentSpeed(self):
        b = self._read_memory(self.PRESENT_SPEED, 2)
        if b is not None:
            return int(Word16(b, bitsigned=True))
        return None
    def SyncReadPresentSpeed(self, servo_ids):
        return self.SyncReadWord(servo_ids, self.PRESENT_SPEED)
    def ReadPresentPosition(self):
        b = self._read_memory(self.PRESENT_POSITION, 2)
        if b is not None:
            return int(b)
    def SyncReadPresentPosition(self, servo_ids):
        return self.SyncReadWord(servo_ids, self.PRESENT_POSITION)
        return None
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



class myST3215(ST3215):
    def __init__(self, port: str, baudrate: int = 1000000,
                 read_timeout: float = 0.002) -> None:
        super().__init__(port, baudrate, read_timeout)
        self.broadcast = myServo(self, 254)

    def list_servos(self, max_id: int = 253) -> list[int]:
        """
        Scan for connected servos by pinging all possible IDs (0-253)
        or up to the given `max_id`.
        Returns:
            List of servo IDs that responded to the ping.
        """
        found = []
        for servo_id in range(0, min(max_id+1,254)):
            try:
                self.wrap_servo(servo_id)
                found.append(servo_id)
            except ServoNotRespondingError:
                continue
        return found
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
    # the original implementation is buggy
    def _sync_read(self, address: int, data_length: int,
                   servo_ids: Sequence[int]) -> dict[int,Optional[dict[str,object]]]:
        self.logger.debug(
                f"SYNC READ from address {address:#02x}, length {data_length} "
                f"for servos {servo_ids}"
        )
        parameters = [address, data_length, *servo_ids]
        packet = self.send_instruction(0xFE, Instruction.SYNC_READ, parameters)
        responses: dict[int, Optional[dict[str, object]]] = {}
        rxpacket = self.read_response(packet)
        bytelist = iter(rxpacket)
        response = {}
        pkcomplete = False
        try:
            while next(bytelist)==0xFF and next(bytelist)==0xFF:
                pkcomplete = False
                scs_id = next(bytelist)
                retlen = next(bytelist)
                if retlen != data_length+2:
                    response[scs_id] = None
                    break
                err = next(bytelist)
                data = [next(bytelist) for _ in range(data_length)]
                crc = ~(sum(data) + retlen + scs_id + err) & 0xFF
                rcrc = next(bytelist)
                #if crc != rcrc:
                #    response[scs_id] = None
                response[scs_id] = data
                response[scs_id] = {
                        "header": [0xFF, 0xFF],
                        "id": scs_id,
                        "length": retlen,
                        "error": err,
                        "parameters": data,
                        "received_checksum": rcrc,
                        "calculated_checksum": crc,
                        "checksum_valid": (rcrc==crc)
                }
                pkcomplete = True
        except StopIteration:
            pass
        # TODO can inspect pkcomplete==True for success
        return response


class Motors:
    ST_STEPS = 4096
    ST_MAX_WRAPS = 7
    ST_MIDDLE = 2048


class MotorController:
    def __init__ (self, device, log, axes_map={}, motorconf={}, max_id=253):
        self.log = log
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
        self.servo_ids = self.controller.list_servos(max_id=max_id)
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
        if axis in self.axes:
            res = self._servos[axis].WriteRunningSpeed(vel)
            if res and not res['error']:
                self.current_set_speed[axis] = vel
        read_vel = None
        if return_read:
            read_vel = self._servos[axis].ReadPresentSpeed()
        return res, read_vel
    def get_speed (self, axis=''):
        if not axis:
            vel = {}
            vel = self.controller.broadcast.SyncReadPresentSpeed(self.axes_map.keys())
            vel = {self.axes_map[i]: vel[i] for i in vel}
            return vel
        elif axis in self.axes:
            vel = self._servos[axis].ReadPresentSpeed()
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
    def read_position (self, axis=''):
        if not axis:
            pos = {}
            pos = self.controller.broadcast.SyncReadPresentPosition(self.axes_map.keys())
            pos = {self.axes_map[i]: pos[i] for i in pos}
            return pos
        elif axis in self.axes:
            pos = self._servos[axis].ReadPresentPosition()
            return pos
        return None
    def wheel_mode (self, axis='', wheel=True):
        """Set motor specified by `axis` to wheel mode.
        If `axis==-1` (default), apply to all axes."""
        mode = 1 if wheel else 0
        if not axis:
            # TODO broadcast
            for ax in self.axes:
                self._servos[ax].setWheelMode(wheel)
        elif axis in self.axes:
            self._servos[axis].setWheelMode(wheel)
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
# this is a general monitor also for temperature
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
        #newpos = { _:0 for _ in self.motors.axes }
        #newvel = { _:0 for _ in self.motors.axes }
        #for ax in self.motors.axes:
        #    try:
        #        newpos[ax] = self.motors.read_position(ax)
        #        newvel[ax] = self.motors.get_speed(ax)
        #    except:
        #        success = False
        #    if newpos[ax] is None or newvel[ax] is None:
        #        success = False
        newpos = self.motors.read_position()
        newvel = self.motors.get_speed()
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




