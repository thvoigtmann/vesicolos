import sys, os, tty, termios, select, psutil
import time
import threading, signal
import json

import RPi.GPIO as GPIO
import gpiozero                            # used for PWM (LED and heater)
import board, digitalio, adafruit_max31865 # used for temperature sensor
GPIO.setmode(GPIO.BCM)

import picamera2

# local repositories
import scservo_sdk                         # used for motor driver

## global settings

# timeout settings, adapted to expected flight trajectory
# for MAPHEUS-16 these are the values roughly matching the MOSAIC timeline
SOE_TIMEOUT = 67               # timeout to start if no mug signal comes
EXP_TIMEOUT = 400              # timeout for duration of experiment
MUG_STICKY = True              # if true, keep mug status ON once set

# hardware settings
# GPIO pin layout used
STATUS_PINS = { 'LO': 17, 'mug': 27 } # GPIO pins used for signals
GPIO_LED = 13                  # GPIO pin used for LED (PWM)
GPIO_HEATER = 12               # GPIO pin used for heater (PWM)
GPIO_TEMP = board.D5           # cable select for temp sensor (SPI) on GPIO 5
# SPI bus defaults, values here are not used in the code below for now:
#GPIO_MISO = 9                 # MISO signal for SPI bus
#GPIO_MOSI = 10                # MOSI signal for SPI bus
#GPIO_CLK = 11                 # CLK signal for SPI bus
# settings for temperature sensor
RTD_NOMINAL = 1000             # temp sensor is a PT1000
RTD_REFERENCE = 4300           # MAX31865 board uses 4300ohm reference
RTD_WIRES = 2                  # temp sensor is attached in 2-wire setup
# motor settings
ST_BAUDRATE = 1000000          # UART baud rate for ST servo driver
ST_DEVICE = '/dev/ttyAMA0'     # UART device (Raspberry 5)
#ST_DEVICE = '/dev/ttyS0'      # UART device (Raspberry 4)
ST_STEPS = 4096                # steps per full turn in the servo
ST_MAX_WRAPS = 7               # how many turns the servo can resolve
ST_MIDDLE = 2048               # middle position set by zeroing the servo
ST_MAX_ID = 10                 # maximum servo ID to scan for
ST_MOVING_ACC = 50             # default servo acceleration
ST_MOVING_ACC_SLOW = 10        # servo acceleration for slow movements
MONITOR_INTERVAL = 1           # interval in seconds for the motor monitor
# for zstack: we have 1 turn = 4096 steps = 100mu
# aim for slices 1mu apart => stepsize = 40 steps = 0.98mu
# 50 such steps (half below, half above target) => scan depth 2000steps=48.8mu
# we also specify the waiting time on each z position in the stack
MOTOR_DZ_STEPSIZE = 40         # in steps, 4096 steps = 100mu
MOTOR_DZ_STEPS = 50            # number of steps, scan depth = steps*stepsize
MOTOR_DZ_WAIT = 0.1            # in seconds, wait time at each step
# if we loose internet connection we don't want the motors to move
# indefinitely, so there is a timeout in interactive mode
MOTOR_TIMEOUT = 3              # seconds until motor stop in unattended UI mode

# servo configuration for the three axes
# values with lower-case names will be modified by the program
# the IDs are hard-coded in the motors
SERVOS = {
  'X': { 'ID': 0, 'DEFAULT_SPEED': 2400, 'SPEED_INC': 200, 'current_speed': 0 },
  'Y': { 'ID': 9, 'DEFAULT_SPEED': 2400, 'SPEED_INC': 200, 'current_speed': 0 },
  'Z': { 'ID': 1, 'DEFAULT_SPEED':  200, 'SPEED_INC':  40, 'MAX_WRAP': 1, 'current_speed': 0 },
}
SERVO_CMDS = {
  'LEFT':   { 'axis': 'Y', 'dir': +1 },
  'RIGHT':  { 'axis': 'Y', 'dir': -1 },
  'DOWN':   { 'axis': 'X', 'dir': -1 },
  'UP':     { 'axis': 'X', 'dir': +1 },
  'PGUP':   { 'axis': 'Z', 'dir': -1 },
  'PGDOWN': { 'axis': 'Z', 'dir': +1 }
}

# user-entry variables: the values here will be affected by the user interaction
# these are stored in a restart file and re-read from there if possible
# user-saved positions will be stored here
POSITIONS = {}
# for each user-saved position we can define our own temperature ramps
# defined by Tmin, Tmax, dt (ramp time) and a start time ts
# (where here all times are measured in reference to the time where
# the temperature control of this profile became active)
# via T(t) = Tmin + (Tmax-Tmin)*(t-ts)/dt
# we also define a maximum time to spend in this ramp
# this is only used as a waiting time, not by the temperature controller
TEMPERATURES = {
  'default': { 'Tmin': 25, 'Tmax': 40, 'dt': 30, 'ts': 10, 'tmax': 90 }
}
def T(t,Tmin,Tmax,dt,ts):
    tau = t-ts
    if tau <= 0:
        return Tmin
    if tau >= dt:
        return Tmax
    return Tmin + (Tmax - Tmin) * tau/dt


# filenames for output
tstamp = time.strftime("%Y-%m-%d-%H-%M-%S")
restartfile = 'vesicolos-restart.json'
logfile = tstamp+'/vesicolos.log'
temperature_logfile = tstamp+'/temperature.log'
camfile = tstamp+'/capture-{pos}.h264' # could use {frame:06d}
ptsfile = tstamp+'/capture-{pos}-pts.txt'
# make output directory
try:
    os.mkdir(tstamp)
except:
    print ("ERROR cannot write output")


# LO/mug signal configuration
status = { _: False for _ in STATUS_PINS }
for pin in STATUS_PINS.values():
    GPIO.setup(pin, GPIO.IN)


## global state variables

stop = False
manual_lift_off = False
debug = False
motor_moving = False



## methods to deal with keyboard input

# these are the scan codes that we get from the ANSI terminal
# (or at least enough bytes after the esc byte for the special keys
# in order to recognize them reliably enough)
key_mapping = {
  9: 'tab', 10: 'return', 27: 'esc', 32: 'space',
  (91,65): 'up', (91,66): 'down', (91,67): 'right', (91,68): 'left',
  (79,80): 'f1', (79,81): 'f2', (79,82): 'f3', (79,83): 'f4',
  (91,53): 'pgup', (91,54): 'pgdown',
  127: 'backspace'
}

# method to read keyboard input
# adapted to make this non-blocking (i.e., return after specified timeout)
# this allows the main loop waiting for user input to recognize
# if a status variable changed even if the user doesn't do anything
def getkey(timeout=1):
    """Read key from keyboard using low-level os.read()
       Return string describing the key.
       Adapted from https://stackoverflow.com/a/47197390/5802289"""
    global key_mapping
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        r, w, e = select.select([ sys.stdin ], [], [], timeout)
        if not sys.stdin in r:
            res = None
        else:
            b = os.read(sys.stdin.fileno(), 4).decode()
            if len(b) >= 3:
                k = ord(b[2])
                res = key_mapping.get((ord(b[1]),k), chr(k))
            else:
                k = ord(b)
                res = key_mapping.get(k, chr(k))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return res

# we want to be able to kill the camera preview program that might be running
# in the interactive phase, otherwise our recordings would fail
rpicam_process = 'rpicam-vid'
def kill_proc_by_name(name):
   # find processes by name using pstuil
   processes = [proc for proc in psutil.process_iter(['name']) \
                if proc.info['name'] == name]
   for proc in processes:
      try:
         proc.kill()
         time.sleep(0.2) # give it time to die
      except psutil.NoSuchProcess:
         print(f"No such process: {proc.pid}")
      except psutil.AccessDenied:
         print(f"Access denied to {proc.pid}")


## logging

class Logger():
    def __init__ (self, logfile):
        self.logfile = open(logfile, 'w')
        self.logfile.write("*** START *** "+self.tstamp()+'\n')
        self.logfile.flush()
    def __del__ (self):
        self.logfile.write("*** END   *** "+self.tstamp()+'\n')
        self.logfile.close()
    def tstamp (self):
        return time.strftime("%Y-%m-%d %H:%M:%S")
    def write (self, msg):
        self.logfile.write(self.tstamp()+" "+msg+'\n')
        self.logfile.flush()
        print(msg)
    def err (self, msg):
        self.logfile.write("*** ERR *** "+self.tstamp()+" "+msg+'\n')
        self.lofgile.flush()
        print("*** ERR ***",msg)

## motor driver and motor handling stuff

# wrapper around the scservo_sdk from waveshare
# adds some convenience functions and fixes some functionality bugs

class MotorDriver (scservo_sdk.sms_sts):
    def __init__ (self, devicename, baudrate):
        self.portHandler = scservo_sdk.PortHandler(devicename)
        scservo_sdk.sms_sts.__init__ (self, self.portHandler)
        if not self.portHandler.openPort():
            raise Exception("failed to open servo port")
        if not self.portHandler.setBaudRate(baudrate):
            raise Exception("failed to set servo baud rate")
    # comm,err are a tuple usually returned by the waveshare code functions
    # as convenience wrapper, check for errors, inform user,
    # and return True/False as success status for motor actions
    def success (self, comm, err, log=None, strict=False):
        _success = True
        if comm != scservo_sdk.COMM_SUCCESS:
            if not log is None:
                log.write("TxRx  " + self.getTxRxResult(comm))
                _success = False
        if err != 0:
            if not log is None:
                log.write("RxERR " + self.getRxPacketError(err))
            _success = False
        return _success
    # setting a position makes the servo move there, but the call is
    # non-blocking; if we want to make sure the servo arrived, we need
    # to query its "moving" status and wait
    def WaitMoving (self, scs_id):
        moving = True
        while moving:
            time.sleep(0.2) # seemed crucial for timing issues
            mov, comm, err = self.ReadMoving(scs_id)
            if self.success(comm, err):
                moving = mov
        return
    # the following add to the underlying driver some calls that
    # are for some reason not implemented
    # wave share dirver has a function to put us _into_ wheel mode,
    # but not out of it...
    def WheelMode (self, scs_id, wheel=True):
        return self.write1ByteTxRx(scs_id, scservo_sdk.SMS_STS_MODE, int(wheel))
    # resetting the servo internal position counter to the middle position
    # is implemented in the Arduino C library but not in python library
    def SetMiddle (self, scs_id):
        return self.write1ByteTxRx(scs_id, scservo_sdk.SMS_STS_TORQUE_ENABLE, 128)
    # the following fixes a real bug in the python SDK
    # the handling of negative positions was wrong, the code below is
    # adapted from the Arduino C library
    def WritePosEx (self, scs_id, position, speed, acc):
        pos = position
        if pos < 0:
            pos = (-pos) | (1<<15)
        txpacket = [acc, self.scs_lobyte(pos), self.scs_hibyte(pos), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, scservo_sdk.SMS_STS_ACC, len(txpacket), txpacket)
    # convenience wrapper for moving
    def GotoPos (self, scs_id, position):
        comm, err = self.WritePosEx(scs_id, position, 0, ST_MOVING_ACC)
        _success = self.success(comm, err)
        if _success:
            self.WaitMoving(scs_id)
        return _success

# monitor for servo positions
# we also output signal status here for convenience
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
            if success and not self.silent:
                print("-- position",self.pos,self.wrap,"--\r")
                log.write(str(status)+" T="+str(temp_sensor.temperature))
            self.next_t += self.increment
            threading.Timer(self.next_t - time.time(), self._run).start()
    # update: query the motor positions, try to detect wrap-arounds
    def update_pos (self,detect_wrap=True):
        success = True
        newpos = { _:0 for _ in axes }
        for ax in axes:
            newpos[ax], comm, err = motorDriver.ReadPos(SERVOS[ax]['ID'])
            success &= motorDriver.success(comm, err)
        if success and detect_wrap:
            for ax in axes:
                vel = SERVOS[ax]['current_speed']
                if vel>0 and self.pos[ax] > ST_STEPS/2 and newpos[ax] < self.pos[ax]:
                    self.wrap[ax] += 1
                if vel<0 and self.pos[ax] < ST_STEPS/2 and newpos[ax] > self.pos[ax]:
                    self.wrap[ax] -= 1
                if vel==0:
                    if newpos[ax] < 100 and self.pos[ax] > ST_STEPS-100:
                        self.wrap[ax] += 1
                    if newpos[ax] > ST_STEPS-100 and self.pos[ax] < 100:
                        self.wrap[ax] -= 1
                self.pos[ax] = newpos[ax]
        return success
    def stop (self):
        self.done = True
    def start (self, silent=False):
        self.next_t = time.time()
        self.silent = silent
        self.done = False
        self._run()

# find all servos that are connected, return axes array
# listing those axes ('X', 'Y', 'Z') that we can control
def find_all_servos():
    detected_servos = []
    for SCS_ID in range(ST_MAX_ID+1):
        model, comm, err = motorDriver.ping(SCS_ID)
        if motorDriver.success(comm, err, log, strict=True):
            print("[ID:%03d] found SCServo model %d" % (SCS_ID, model))
            detected_servos.append(SCS_ID)
    log.write("found %d servos" % len(detected_servos))
    # see if we can map the servos that we found to the axes we expect
    axes = []
    found_all_servos = True
    for ax in sorted(SERVOS.keys()):
        if SERVOS[ax]['ID'] in detected_servos:
            axes.append(ax)
            motorDriver.WheelMode(SERVOS[ax]['ID'], True)
        else:
            found_all_servos = False
    if not found_all_servos:
        log.err("unexpected servo configuration found, continuing anyway")
    return axes

# set all motor velocities to zero
def stop_all_servos ():
    global axes, motor_moving
    success = True
    for ax in axes:
        comm, err = motorDriver.WriteSpec(SERVOS[ax]['ID'], 0, ST_MOVING_ACC)
        if motorDriver.success (comm, err):
            SERVOS[ax]['current_speed'] = 0
        else:
            success = False
    if success:
        motor_moving = False
    return success



## global objects

# LED uses hardware PWM, taken care of by the gpiozero module
led = gpiozero.LED(GPIO_LED)
led.off()

# heater uses hardware PWM, taken care of by the gpiozero module
heater = gpiozero.PWMOutputDevice(GPIO_HEATER)
heater.off()

# temperature sensor, SPI on a MAX31865 board, taken care of by adafruit module
spi = board.SPI()
cs = digitalio.DigitalInOut(GPIO_TEMP)
temp_sensor = adafruit_max31865.MAX31865(spi, cs, \
    rtd_nominal=RTD_NOMINAL, ref_resistor=RTD_REFERENCE, wires=RTD_WIRES)

log = Logger(logfile)
temperature_log = Logger(temperature_logfile)

# motor driver, UART taken care of by the WaveShare SCServo kit (patched)
try:
    motorDriver = MotorDriver (ST_DEVICE, ST_BAUDRATE)
except Exception as err:
    log.error(str(err))

axes = find_all_servos()
stop_all_servos()

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
        print("cam recording",imgfile)
        # should have some sort of interruptible loop here if we write
        # single frames ourselves... (because our stop() method should
        # be callable)
    def stop (self):
        self.stop_ = True
        self.picam.stop_recording()
        self.picam.close()



# implement waiting for lift-off
# a separate thread checks the GPIO pin every second, writes status variable
# the part of the main program that runs before LO waits for user input
# and exits if the status variable changes
def wait_for_lo ():
    global status, manual_lift_off
    global stop
    t0 = time.time()
    while (not GPIO.input(STATUS_PINS['LO'])) \
        and (not stop) and (not manual_lift_off):
        log.write("waiting for lift off")
        time.sleep(1)
    if not stop:
        status['LO'] = True
        log.write("*** LIFT OFF ***")


threading.Thread(target=wait_for_lo).start()
monitor = ServoMonitor(increment=MONITOR_INTERVAL)

# try re-reading saved parameters on restart
# to catch power cycles
# this is done here so that we can adjust our wraparound counter as well
try:
    with open(restartfile, 'r') as f:
        data = json.load(f)
        TEMPERATURES = data['TEMPERATURES']
        POSITIONS = data['POSITIONS']
        monitor.wrap = data['wrap']
    print("re-loaded from restart file")
except:
    pass
# every 10 seconds, write current temperature and position settings
# to the restart file
prog_end = False
def save_restart ():
    global prog_end, monitor
    while not prog_end:
        with open(restartfile, 'w') as f:
            json.dump({'TEMPERATURES': TEMPERATURES, 'POSITIONS': POSITIONS, \
                       'wrap':monitor.wrap}, f, sort_keys=True, indent=4)

        time.sleep(10)
        os.sync()
threading.Thread(target=save_restart).start()



# FIRST PHASE

# before lift off we are connected via umbilical
# allow user to remote control with keyboard commands

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


# this is the main before-lift-off loop for user interaction

last_savepos = None
camera = None
recordings = []
motor_timeout = MOTOR_TIMEOUT
while not status['LO']:
    ch = None
    ch = getkey()
    if ch is None:
        if motor_moving and motor_timeout > 0:
            motor_timeout -= 1
            if motor_timeout <= 0:
                if stop_all_servos():
                    motor_timeout = MOTOR_TIMEOUT
        continue
    ch = ch.upper()
    match ch:
        case 'ESC':
            stop = True
            print("BYE")
            break
        case 'UP' | 'DOWN' | 'LEFT' | 'RIGHT' | 'PGUP' | 'PGDOWN':
            ax = SERVO_CMDS[ch]['axis']
            direction = SERVO_CMDS[ch]['dir']
            vel = SERVOS[ax]['current_speed'] \
                + direction*SERVOS[ax]['SPEED_INC']
            comm, err = motorDriver.WriteSpec(SERVOS[ax]['ID'], vel, ST_MOVING_ACC)
            if motorDriver.success(comm, err):
                SERVOS[ax]['current_speed'] = vel
            if not (vel==0):
                motor_moving = True
        case '0':
            if stop_all_servos():
                print ("stop")
        case 'F1' | 'F2' | 'F3' | 'F4':
            stop_all_servos()
            if ch == 'H': # this is for the moment not used
                poskey = 'homepos'
                reset_wrap = True
            else:
                poskey = 'savepos'+ch[1]
                reset_wrap = False
            success = monitor.update_pos()
            if success:
                POSITIONS[poskey] = {}
                for ax in axes:
                    if reset_wrap:
                        monitor.wrap[ax] = 0
                    POSITIONS[poskey][ax] = (monitor.pos[ax],monitor.wrap[ax])
                print ('saved',poskey,monitor.pos,monitor.wrap)
                last_savepos = poskey
        case '1' | '2' | '3' | '4':
            if ch == 'R': # this is for the moment not used
                poskey = 'homepos'
            else:
                poskey = 'savepos'+ch
            if not (poskey in POSITIONS):
                print ('no position',poskey,'saved')
            else:
                stop_all_servos()
                time.sleep(0.2)
                monitor.stop()
                move_to_stored_position(poskey)
                monitor.start()
                last_savepos = poskey
        case 'G':
            stop_all_servos()
            ax = input('axis? ').upper()
            posstr = input('position (servo mode)? ')
            if not ax in axes:
                print ("axis not found")
                continue
            try:
                pos = int(posstr)
            except ValueError:
                print ("illegal position")
                continue
            motorDriver.WheelMode(SERVOS[ax]['ID'], False)
            time.sleep(0.2)
            motorDriver.GotoPos(SERVOS[ax]['ID'], pos)
            time.sleep(0.2)
            motorDriver.WheelMode(SERVOS[ax]['ID'], True)
        case 'W':
            stop_all_servos()
            wheelpos = { _: 0 for _ in axes }
            servopos = { _: 0 for _ in axes }
            for ax in axes:
                scs_id = SERVOS[ax]['ID']
                wheelpos[ax], comm, err = motorDriver.ReadPos(scs_id)
                time.sleep(0.2)
                motorDriver.WheelMode(scs_id, False)
                servopos[ax], comm, err = motorDriver.ReadPos(scs_id)
                time.sleep(0.2)
                motorDriver.WheelMode(scs_id, True)
            print ("current position (wheel)",wheelpos)
            print ("current position (servo)",servopos)
        case 'L':
            led.toggle()
            log.write('LED {}'.format(['OFF','ON'][led.is_active]))
        case 'H':
            heater.toggle()
            log.write('HEATER PWM active {} value {}'\
                      .format(heater.is_active,heater.value))
        case 'T':
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
        case 'X':
            log.write("MANUAL LIFT OFF")
            manual_lift_off = True
            status['LO'] = True
        case 'C':
            if camera is not None:
                camera.stop()
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
        case '?':
            print ("menu:")
            print ("left/right/up/down: change x/y velocity")
            print ("pgup/pgdown: change z velocity (focus)")
            print ("0: stop motors")
            print ("g : Goto position (one axis, in servo mode)")
            print ("w : Where are we? print current positions")
            print ('l : LED toggle')
            print ('h : Heater toggle')
            print ('t : Temperature set points')
            print ('c : Start camera')
            print ('x : manual liftoff')
            print ("F1-F4: store positions")
            print ("1-4: recall positions")
            for i in range(1,5):
                poskey = 'savepos'+str(i)
                if poskey in POSITIONS:
                    print (' ',poskey,POSITIONS[poskey])


#  SECOND PHASE

# we have a lift-off, so remote control is off now

if not stop:
    stop_all_servos()
    if not manual_lift_off:
        # this is a real lift off, we wait for mug now
        t0 = time.time()
        # we record the ascent for sure, but if the user started before
        # we do not interrupt them
        if camera is None:
            # kill external previewer app if running, we want the cam ours now
            kill_proc_by_name(rpicam_process)
            try:
                camera = CameraController(camfile,pts=ptsfile,keys={'pos':'liftoff'})
                threading.Thread(target=camera.record).start()
                led.on()
            except RuntimeError as e:
                camera = None
                log.err("camera error "+str(e))
        while not GPIO.input(STATUS_PINS['mug']):
            log.write("waiting for microgravity")
            time.sleep(0.5)
            if time.time() - t0 >= SOE_TIMEOUT:
                log.write("SOE by timeout")
                break
        if camera is not None:
            camera.stop()
        status['mug'] = True



## THIRD PHASE: MICROGRAVITY EXPERIMENT


# microgravity timeout: handled by UNIX ALRM signal
# but the loop here polls the mug pin and sends the signal once that
# flag goes off, cancelling possibly before
def microgravity_timeout ():
    global status
    #signal.alarm(EXP_TIMEOUT)
    t0 = time.time()
    while MUG_STICKY or GPIO.input(STATUS_PINS['mug']) or manual_lift_off:
        time.sleep(1)
        if time.time() - t0 >= EXP_TIMEOUT:
            log.write("SOE OFF by timeout")
            break
    status['mug'] = MUG_STICKY or bool(GPIO.input(STATUS_PINS['mug'])) \
                    or manual_lift_off
    log.write("END OF MUG")
    signal.raise_signal(signal.SIGALRM)

# the SIGALRM handler is responsible for raising the exception that will
# interrupt the microgravity_experiment() function
class MicrogravityTimeout (Exception):
    pass
def mug_timeout_handler (signum, frame):
    global status
    if status['mug']:
        # mug flag still on, thus we have been raised by timeout
        raise MicrogravityTimeout("END OF EXPERIMENT by timeout")
    else:
        raise MicrogravityTimeout("END OF EXPERIMENT")


class TemperatureController ():
    def __init__ (self):
        self.set_profile (TEMPERATURES['default'])
    def __del__ (self):
        if heater is not None:
            heater.off()
    def set_profile (self, profile):
        self.Tmin, self.Tmax, self.dt, self.ts = \
            profile['Tmin'], profile['Tmax'], profile['dt'], profile['ts']
        self.t0 = time.time()
    def control (self):
        while heater is not None:
            t = time.time()
            Ttarget = T(t-self.t0,self.Tmin,self.Tmax,self.dt,self.ts)
            Tcurrent = temp_sensor.temperature
            if Tcurrent < Ttarget:
                heater.on()
            elif Tcurrent >= Ttarget:
                # we cannot currently cool
                # we could use PWM to heat less once we get near target
                # but this is not implemented yet
                heater.off()
            temperature_log.write(\
                "t0 = {}, t = {}, T = {}, Ttarget = {}, heat {}" \
                .format(self.t0,t,Tcurrent,Ttarget,heater.is_active))
            time.sleep(1)

Tcontrol = TemperatureController()



# CORE MICROGRAVITY EXPERIMENT PROCEDURE

def microgravity_experiment ():
    positions = sorted(POSITIONS.keys() or ['default'])
    log.write("mug sequence: positions "+" / ".join(positions))
    led.on()
    while status['mug'] or manual_lift_off:
        for pos in positions:
            ckey = pos
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
                log.err('camera error '+str(e))
            if not pos == 'default':
                move_to_stored_position (pos)
                do_zstack = True
            else:
                do_zstack = False
            if pos in TEMPERATURES:
                Tprofile = TEMPERATURES[pos]
            else:
                Tprofile = TEMPERATURES['default']
            Tcontrol.set_profile (Tprofile)
            # take some time, do z-stacks
            tmax = Tprofile['tmax'] + time.time()
            if 'Z' in axes:
                scs_id = SERVOS['Z']['ID']
                comm, err = motorDriver.WheelMode(scs_id, False)
                # do_zstack shall only be true if we don't have
                # motor errors
                do_zstack &= motorDriver.success(comm, err)
                time.sleep(0.2)
                motorDriver.SetMiddle(scs_id)
                do_zstack &= motorDriver.success(comm, err)
                time.sleep(0.2)
                # after set middle, motor is in servo mode, pos 2048
                # set zpos to highest position first
                zpos = 2048 + MOTOR_DZ_STEPSIZE*int(MOTOR_DZ_STEPS/2)
                zcnt = 0
                zdirection = -1
                motorDriver.GotoPos (scs_id, zpos)
                #motorDriver.WritePosEx (scs_id, zpos, 0, 0) # or ST_MOVING_ACC_SLOW
            while True:
                if do_zstack:
                    if zcnt >= MOTOR_DZ_STEPS:
                        zdirection = -zdirection
                        zcnt = 0
                    zcnt += 1
                    zpos += zdirection*MOTOR_DZ_STEPSIZE
                    motorDriver.GotoPos (scs_id, zpos) 
                    #motorDriver.WritePosEx (scs_id, zpos, 0, 0) # or ST_MOVING_ACC_SLOW
                time.sleep(MOTOR_DZ_WAIT)
                if time.time() > tmax:
                    break
            if 'Z' in axes:
                motorDriver.WheelMode(scs_id, True)
                time.sleep(0.2)
            if not camera is None:
                camera.stop()


if not stop:
    # now start the actual measurement program
    # timeout handling is done by SIGALRM here
    # we now also need temperature control
    signal.signal(signal.SIGALRM, mug_timeout_handler)
    threading.Thread(target=microgravity_timeout).start()
    temp_controller = TemperatureController ()
    threading.Thread(target=temp_controller.control).start()
    try:
        microgravity_experiment()
    except MicrogravityTimeout as msg:
        log.write(str(msg))
    signal.alarm(0) # clear any remaining ALRM signals



## SHUTDOWN

print('SHUTDOWN')
os.sync()

if camera is not None:
    camera.stop()
    camera = None
led.off()
heater.off()
# the above two lines should switch off the LED and the heater
# however, after program exit, they come on again magically
# the stuff below is an (unsuccessful) attempt to prevent that
led.close()
heater.close()
led = None
heater = None
try:
    GPIO.setup(GPIO_LED, GPIO.OUT)
    GPIO.setup(GPIO_HEATER, GPIO.OUT)
    GPIO.output(GPIO_LED, False)
    GPIO.output(GPIO_HEATER, False)
except Exception as err:
    print('ERR' + str(err))

# stop all servos, close port

stop_all_servos()
for ax in axes:
    motorDriver.WheelMode(SERVOS[ax]['ID'], True)

monitor.stop()
prog_end = True

print('END')
