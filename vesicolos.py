import sys, os, tty, termios
import time
from threading import Thread

import scservo_sdk

## global settings

ST_BAUDRATE = 1000000          # UART baud rate for ST servo driver
ST_DEVICE = '/dev/ttyAMA0'     # UART device (Raspberry 5)
#ST_DEVICE = '/dev/ttyS0'       # UART device (Raspberry 4)
ST_STEPS = 4096                # steps per full turn in the servo
ST_MAX_WRAPS = 7               # how many turns the servo can resolve
ST_MIDDLE = 2048               # middle position set by zeroing the servo
ST_MAX_ID = 10                 # maximum servo ID to scan for
ST_MOVING_ACC = 50             # default servo acceleration

# servo configuration for the three axes
# values with lower-case names will be modified by the program
SERVOS = {
  'X': { 'ID': 0, 'DEFAULT_SPEED': 2400, 'SPEED_INC': 200, 'current_speed': 0 },
  'Y': { 'ID': 0, 'DEFAULT_SPEED': 2400, 'SPEED_INC': 200, 'current_speed': 0 },
  'Z': { 'ID': 1, 'DEFAULT_SPEED':  200, 'SPEED_INC':  20, 'current_speed': 0 },
}
SERVO_CMDS = {
  'UP':     { 'axis': 'Y', 'dir': +1 },
  'DOWN':   { 'axis': 'Y', 'dir': -1 },
  'LEFT':   { 'axis': 'X', 'dir': +1 },
  'RIGHT':  { 'axis': 'X', 'dir': -1 },
  'PGUP':   { 'axis': 'Z', 'dir': +1 },
  'PGDOWN': { 'axis': 'Z', 'dir': -1 }
}

## methods to deal with keyboard input

key_mapping = {
  9: 'tab', 10: 'return', 27: 'esc', 32: 'space',
  (91,65): 'up', (91,66): 'down', (91,67): 'right', (91,68): 'left',
  (79,80): 'f1', (79,81): 'f2', (79,82): 'f3', (79,83): 'f4',
  (91,53): 'pgup', (91,54): 'pgdown',
  127: 'backspace'
}

def getkey():
    """Read key from keyboard using low-level os.read()
       Return string describing the key.
       Adapted from https://stackoverflow.com/a/47197390/5802289"""
    global key_mapping
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            b = os.read(sys.stdin.fileno(), 4).decode()
            print("L",len(b))
            for i in range(len(b)):
                print("I",ord(b[i]),end=' ')
            print("\n",len(b))
            if len(b) >= 3:
                k = ord(b[2])
                return key_mapping.get((ord(b[1]),k), chr(k))
            else:
                k = ord(b)
            print (k)
            return key_mapping.get(k, chr(k))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

## logging

class Logger():
    def __init__ (self):
        self.logfile = open("vesicolos.log", 'w')
        self.logfile.write("*** START *** "+self.tstamp())
    def __del__ (self):
        self.logfile.write("*** END   *** "+self.tstamp())
        self.logfile.close()
    def tstamp (self):
        return time.strftime("%Y-%m-%d %H:%M:%S")
    def write (self, msg):
        self.logfile.write(self.tstamp()+" "+msg)
        print(msg)
    def error (self, msg):
        self.logfile.write("*** ERR *** "+self.tstamp()+" "+msg)
        print("*** ERR ***",msg)

## motor driver

class MotorDriver (scservo_sdk.sms_sts):
    def __init__ (self, devicename, baudrate):
        self.portHandler = scservo_sdk.portHandler(devicename)
        scservo_sdk.sms_sts.__init__ (self, self.portHandler)
        if not portHandler.openPort():
            raise Exception("failed to open servo port")
        if not portHandler.setBaudRate(baudrate):
            raise Exception("failed to set servo baud rate")
    def success (self, comm, err, log=None, strict=False):
        _success = True
        if comm != scservo_sdk.sms_sts..COMM_SUCCESS:
            if not log is None:
                log.write("TxRx  " + self.getTxRxResult(comm)
            if strict:
                _success = False
        if err != 0:
            if not log is None:
                log.write("RxERR " + self.getRxPacketError(err)
            _success = False
        return _success
    def WaitMoving (self, scs_id):
        moving = True
        while moving:
            time.sleep(0.2)
            mov, comm, err = self.ReadMoving(scs_id)
            if self.success(comm, err):
                moving = mov
        return
    # the following add to the underlying driver some calls that
    # are for some reason not implemented
    def WheelMode (self, scs_id, wheel=True):
        return self.write1ByteTxRx(scs_id, scservo_sdk.SMS_STS_MODE, int(wheel))
    def SetMiddle (self, scs_id):
        return self.write1ByteTxRx(scs_id, scservo_sdk.SMS_STS_TORQUE_ENABLE, 128)
    # the following fixes a real bug in the python SDK
    def WritePosEx (self, scs_id, position, speed, acc):
        pos = position
        if pos < 0:
            pos = (-pos) | (1<<15)
        txpacket = [acc, self.scs_lobyte(pos), self.scs_hibyte(pos), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, SMS_STS_ACC, len(txpacket), txpacket)
    # convenience wrapper for moving
    def GotoPos (self, scs_id, position):
        comm, err = self.WritePosEx(scs_id, position, 0, ST_MOVING_ACC)
        _success = self.success(comm, err)
        if _success:
            self.WaitMoving(scs_id)
        return _sucess


## global state variables

stop = False
lift_off = False
soe_on = False
manual_lift_off = False
debug = False

# global objects

log = Logger()

try:
    motorDriver = MotorDriver (ST_DEVICE, ST_BAUDRATE)
except Exception as err:
    log.error(str(err))

detected_servos = []
for SCS_ID in range(ST_MAX_ID+1):
    model, comm, err = motorDriver.ping(SCS_ID)
    if motorDriver.success(comm, err, log, strict=True):
        print("[ID:%03d] found SCServo model %d" % (SCS_ID, model))
        detected_servos.append(SCS_ID)
log.write("found %d servos" % len(detected_servos))

axes = []
found_all_servos = True
for ax in sorted(SERVOS.keys()):
    if SERVOS[ax]['ID'] in detected_servos:
        axes.append(ax)
        MotorDriver.WheelMode(SERVOS[ax]['ID'], True)
    else:
        found_all_servos = False
if not found_all_servos:
    log.err("unexpected servo configuration found")

## monitor for servo positions

class ServoMonitor():
    def __init__ (self, increment, silent=False):
        self.next_t = time.time()
        self.silent = silent
        self.done = False
        self.increment = increment
        self.pos = { _:0 for _ in axes }
        self.wrap = { _:0 for _ in axes }
        self._run()
    def _run (self):
        if not self.done:
            success = self.update_pos()
            if success and not silent:
                print("-- position",self.pos,self.wrap,"--\r")
            self.next_t += self.increment
            threading.Timer(self.next_t - time.time(), self._run).start()
    def update_pos (self):
        success = True
        newpos = { _:0 for _ in axes }
        for ax in axes:
            newpos[ax], comm, err = motorDriver.ReadPos(SERVOS[ax]['ID'])
            success &= motorDriver.success(comm, err)
        if success:
            for ax in axes:
                if newpos[ax] < 100 and self.pos[ax] > ST_STEPS-100:
                    self.wrap[ax] += 1
                if newpos[ax] > ST_STEPS-100 and self.pos[ax] < 100:
                    self.wrap[ax] -= 1
                self.pos[ax] = newpos[ax]
        return success
    def stop (self):
        self.done = True
    def start (self, increment, silent=False):
        self.next_t = time.time()
        self.silent = silent
        self.done = False
        self._run()

def stop_all_servos ():
    success = True
    for ax in axes:
        comm, err = MotorDriver.WriteSpec(SERVOS[ax]['ID'], 0, ST_MOVING_ACC)
        success &= MotorDriver.success (comm, err)
    return success



def wait_for_lo ():
    global lift_off
    print("waiting for LO")
    time.sleep(10) # should be while loop checking gpio.input(PIN_LO)
    print("LO")
    lift_off = True
def wait_for_soe ():
    global soe_on 
    print("waiting for SOE")
    time.sleep(15) # should be while loop checking gpio.input(PIN_LO)
    print("SOE")
    soe_on = True



Thread(target=wait_for_lo).start()
Thread(target=wait_for_soe).start()

monitor = ServoMonitor(increment=2)

while True:
    ch = getkey().upper()
    match ch:
        case 'ESC':
            print("BYE")
            break
        case 'UP' | 'DOWN' | 'LEFT' | 'RIGHT' | 'PGUP' | 'PGDOWN':
            ax = SERVO_CMDS[ch]['axis']
            direction = SERVO_CMDS[ch]['dir']
            vel = SERVOS[ax]['current_speed'] \
                + direction*SERVOS[ax]['SPEED_INC']
            comm, err = MotorDriver.WriteSpec(SERVOS[ax]['ID'], vel, ST_MOVING_ACC)
            if Motor.Driver.success(comm, err):
                SERVOS[ax]['current_speed'] = vel
        case '0':
            if stop_all_servos():
                print ("stop")
        case 'H' | 'F1' | 'F2' | 'F3' | 'F4':
            stop_all_servos()
            if ch == 'H':
                poskey = 'homepos'
            else:
                poskey = 'savepos'+ch[1]
            success = ServoMonitor.update_pos()
            if success:
                for ax in axes:
                    SERVOS[ax][poskey] = (monitor.pos[ax],monitor.wrap[ax])
                print ('saved',savepos,monitor.pos,monitor.wrap)
        case 'R' | '1' | '2' | '3' | '4':
            if ch == 'R':
                poskey = 'homepos'
            else:
                poskey = 'savepos'+ch
            foundpos = True
            for ax in axes:
                foundpos &= (poskey in SERVOS[ax])
            if not foundpos:
                print ('no position',poskey,'saved')
            else:
                # to go to a defined position:
                # 1. calculate delta in wheel mode
                # 2. go to servo mode, set current as middle position 2048
                # 3. move to 2048+delta
                #    possibly first a multiple of 7 turns if wrap too large
                # 4. go to wheel mode
                stop_all_servos()
                time.sleep(0.2)
                try:
                    success = ServoMonitor.update_pos()
                    if not success:
                        raise Exception("failed updating position")
                    current_pos = monitor.pos
                    target_pos = { _: SERVOS[_][poskey][0] for _ in axes }
                    current_wrap = monitor.wrap
                    target_wrap = { _: SERVOS[_][poskey][1] for _ in axes }
                    for ax in axes:
                        scs_id = SERVOS[ax]['ID']
                        MotorDriver.WheelMode(scs_id,False)
                        time.sleep(0.2)
                        comm, err = MotorDriver.SetMiddle(scs_id)
                        if not MotorDriver.success(comm, err):
                            raise Exception
                        delta_pos = target_pos[ax] - current_pos[ax]
                        delta_wrap = target_wrap[ax] - current_wrap[ax]
                        if abs(delta_wrap) >= ST_MAX_WRAPS:
                            direction = 1
                            if delta_wrap < 0:
                                direction = -1
                            time.sleep(0.2)
                            success = MotorDriver.GotoPos(scs_id, ST_MIDDLE + direction*ST_STEPS*ST_MAX_WRAPS)
                            if not success:
                                raise Exception
                            comm, err = MotorDriver.SetMiddle(scs_id)
                            if not MotorDriver.success(comm, err):
                                raise Exception
                            delta_wrap -= direction*ST_MAX_WRAPS
                            time.sleep(0.2)
                        dpos = delta_pos + ST_STEPS*delta_wraps
                        success = MotorDriver.GotoPos(scs_id, ST_MIDDLE+dpos)
                        if not success:
                            raise Exception
                        time.sleep(0.2)
                        MotorDriver.WheelMode(scs_id,True)
        case 'G':
            stop_all_servos()
            ax = input('axis? ')
            posstr = input('position (servo mode)? ')
            if not ax in axes:
                print ("axis not found")
                continue
            try:
                pos = int(posstr)
            except ValueError:
                print ("illegal position")
                continue
            MotorDriver.WheelMode(SERVOS[ax]['ID'], False)
            time.sleep(0.2)
            MotorDriver.GotoPos(SERVOS[ax]['ID'], pos)
            time.sleep(0.2)
            MotorDriver.WheelMode(SERVOS[ax]['ID'], True)
        case 'W':
            stop_all_servos()
            wheelpos = { _: 0 for _ in axes }
            servopos = { _: 0 for _ in axes }
            for ax in axes:
                scs_id = SERVOS[ax]['ID']
                wheelpos[ax], comm, err = MotorDriver.ReadPos(scs_id)
                time.sleep(0.2)
                MotorDriver.WheelMode(scs_id, False)
                servopos[ax], comm, err = MotorDriver.ReadPos(scs_id)
                time.sleep(0.2)
                MotorDriver.WheelMode(scs_id, True)
            print ("current position (wheel)",wheelpos)
            print ("current position (servo)",servopos)
        case '?':
            print ("menu:")
            print ("g : goto position (one axis, in servo mode)")
            print ("w : print current positions")

# shutdown:
# stop all servos, close port

stop_all_servos()
for ax in axes:
    MotorDriver.WheelMode(SERVOS[ax]['ID'], True)

monitor.stop()

