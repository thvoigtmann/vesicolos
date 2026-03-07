import sys, os, psutil
import time
import threading, signal
import json
import logging, errno

import RPi.GPIO as GPIO
import gpiozero                            # used for PWM (LED and heater)
import board, digitalio, adafruit_max31865 # used for temperature sensor
GPIO.setmode(GPIO.BCM)

import picamera2

# local repositories
sys.path.append('python-st3215/src')
sys.path.append('.')
from python_st3215 import ST3215
from vesicolos_utils import getkey,Keys,kill_proc_by_name,DummyGPIO
import vesicolos_utils.motors as vm
import vesicolos_utils.cli


## GLOBAL SETTINGS

#
# the settings here can be adjusted without changing the hardware setup
#

# timeout settings for the expected flight trajectory
# these defaults are adapted to the MAPHEUS-16 flight and the
# MOSAIC timeline of that flight as plausible defaults
# the values can be overridden by a json configuration file
# TODO FIXME later on read a json with those values!
# read from flight_configuration.json ?
# or maybe implement cmdline handling, allow to call code like
# `python3 vesicolos.py HCD` or `python3 vesicolos.py TCD` or ... testing
SOE_TIMEOUT_DEFAULT = 67       # timeout to start if no mug signal comes
EXP_TIMEOUT_DEFAULT = 400      # timeout for duration of experiment
MUG_STICKY = True              # if true, keep mug status ON once set

# motor-related adjustable seetings
MONITOR_INTERVAL = 1           # interval in seconds for the motor monitor
ST_MOVING_ACC = 50             # default servo acceleration
ST_MOVING_ACC_SLOW = 10        # servo acceleration for slow movements
# for zstack: we have 1 turn = 4096 steps = 100mu
# aim for slices 1mu apart => stepsize = 40 steps = 0.98mu
# 50 such steps (half below, half above target) => scan depth 2000steps=48.8mu
# we also specify the waiting time on each z position in the stack
MOTOR_DZ_STEPSIZE = 40         # in steps, 4096 steps = 100mu
MOTOR_DZ_STEPS = 50            # number of steps, scan depth = steps*stepsize
MOTOR_DZ_WAIT = 0.1            # in seconds, wait time at each step
# if we loose internet connection we don't want the motors to move
# indefinitely, so there is a timeout in interactive mode
MOTOR_TIMEOUT = 30              # seconds until motor stop in unattended UI mode
# TODO also add a configurable torque limit, probably per axis (and direction?)
# TODO cleanup
SERVOS = {
  'X': { 'DEFAULT_SPEED': 2400, 'SPEED_INC': 200 },
  'Y': { 'DEFAULT_SPEED': 2400, 'SPEED_INC': 200 },
  'Z': { 'DEFAULT_SPEED':  200, 'SPEED_INC':  40, 'MAX_WRAP': 1 }
}
SERVO_CMDS = {
    Keys.LEFT:   { 'axis': 'Y', 'dir': +1 },
    Keys.RIGHT:  { 'axis': 'Y', 'dir': -1 },
    Keys.DOWN:   { 'axis': 'X', 'dir': -1 },
    Keys.UP:     { 'axis': 'X', 'dir': +1 },
    Keys.PGUP:   { 'axis': 'Z', 'dir': -1 },
    Keys.PGDOWN: { 'axis': 'Z', 'dir': +1 }
}


## HARDWARE SETTINGS

#
# these settings reflect the actual hardware configuration of VESCIOLOS
# there should be no need to change these once the hardware is fixed
#
# GPIO pin layout used
STATUS_PINS = { 'LO': 17, 'mug': 27 } # GPIO pins used for signals
GPIO_LED = 13                  # GPIO pin used for LED (PWM)
GPIO_HEATER = 12               # GPIO pin used for heater (PWM)
GPIO_TEMP = board.D5           # cable select for temp sensor (SPI) on GPIO 5
# SPI bus pin layout
# these are the defaults, values here are not used in the code below for now:
#GPIO_MISO = 9                 # MISO signal for SPI bus
#GPIO_MOSI = 10                # MOSI signal for SPI bus
#GPIO_CLK = 11                 # CLK signal for SPI bus
# settings for temperature sensor
RTD_NOMINAL = 1000             # temp sensor is a PT1000
RTD_REFERENCE = 4300           # MAX31865 board uses 4300ohm reference
RTD_WIRES = 2                  # temp sensor is attached in 2-wire setup
# motor settings
ST_DEVICE = { '_default_': '/dev/ttyS0', # UART device (Raspberry 4)
              '5': '/dev/ttyAMA0' }      # UART device (Raspberry 5)
ST_MAX_ID = 10                 # maximum servo ID to scan for (curr. not used)
# some servo configuration parameters are now part of Motors class
# servo configuration for the three axes
# values with lower-case names will be modified by the program
# the IDs are hard-coded in the motors
SERVO_AXIS_MAP = { 0: 'X', 9: 'Y', 1: 'Z' }


## TODO: positions and temperatures, probably somewhere else

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


## STARTUP CODE

# setup logging
def logSetup ():
    """Setup logging, first to console, then to file.
    Attempts to create a time-stamped directory.
    Returns:
    --------
    log : logging object
    path : directory for log and output files
    """
    log = logging.getLogger("VESICOLOS")
    if os.environ.get("DEBUG"):
        log.setLevel(logging.DEBUG)
    else:
        log.setLevel(logging.INFO)
    _log_formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
    _console_log = logging.StreamHandler()
    _console_log.setFormatter(_log_formatter)
    log.addHandler(_console_log)

    tstamp = time.strftime("%Y-%m-%d-%H-%M-%S")
    try:
        os.mkdir(tstamp)
    except:
        log.critical(f"cannot create output directory {tstamp}")
        sys.exit(errno.ENOENT)

    _file_log = logging.FileHandler(tstamp+'/vesicolos.log')
    _file_log.setFormatter(_log_formatter)
    log.addHandler(_file_log)
    return log, tstamp


log, path = logSetup()

restartfile = 'vesicolos-restart.json'
temperature_logfile = path+'/temperature.log'
camfile = path+'/capture-{pos}.h264' # could use {frame:06d}
ptsfile = path+'/capture-{pos}-pts.txt'
rpicam_process = 'rpicam-vid'


# global state variables

status = { _: False for _ in STATUS_PINS }
for pin in STATUS_PINS.values():
    GPIO.setup(pin, GPIO.IN)

stop = False
manual_lift_off = False
debug = False
motor_moving = False

# global objects

# TODO FIXME: for led and heater, reconsider setting to None on GPIO error
# the UI should handle this gracefully
# and the later code: could try to re-init if None

# LED uses hardware PWM, taken care of by the gpiozero module
try:
    led = gpiozero.LED(GPIO_LED)
    led.off()
except Exception as e:
    log.error('LED init failed: '+str(e))
    log.error('LED externally controlled')
    led = DummyGPIO(log,'LED')
    led.off()

# heater uses hardware PWM, taken care of by the gpiozero module
try:
    heater = gpiozero.PWMOutputDevice(GPIO_HEATER)
    heater.off()
except Exception as e:
    log.error('HEATER init failed: '+str(e))
    log.error('heater externally controlled')
    heater = DummyGPIO(log,'HEATER')
    heater.off()


# temperature sensor, SPI on a MAX31865 board, taken care of by adafruit module
spi = board.SPI()
cs = digitalio.DigitalInOut(GPIO_TEMP)
temp_sensor = adafruit_max31865.MAX31865(spi, cs, \
    rtd_nominal=RTD_NOMINAL, ref_resistor=RTD_REFERENCE, wires=RTD_WIRES)

# TODO: FIX THIS
#temperature_log = Logger(temperature_logfile)

# set UART device depending on raspberry model or environment variables
rpi_model = os.environ.get("RASPI_MODEL","_default_")
log.info(f"assuming Raspberry model {rpi_model}")
st_device = ST_DEVICE.get(rpi_model,ST_DEVICE['_default_'])
st_device = os.environ.get("ST_DEVICE",st_device)
log.info(f"using {st_device} as UART device")

global monitor
monitor = None


# implement waiting for lift-off
# a separate thread checks the GPIO pin every second, writes status variable
# the part of the main program that runs before LO waits for user input
# and exits if the status variable changes
# the stop_event can interrupt this waiting for LO,
# it will typically be triggered by the user exiting the UI
def wait_for_lo (stop_event):
    global status, log
    #t0 = time.time() # could implement LO timeout, but we don't need it
    while (not GPIO.input(STATUS_PINS['LO'])) \
        and not stop_event.is_set():
        log.debug("waiting for lift off")
        time.sleep(1)
    if not stop_event.is_set():
        status['LO'] = True
        log.info("*** LIFT OFF ***")


# TODO FIXME: save_restart should now be handled by the UI
# read_restart: not sure yet where to put it
# FIXME the save_restart should only write motors.wrap if the motors
# are currently not moving!
# if the program crashes while the motors are moving, we most likely
# don't know where we are, the wrap might be wrong, and thus we should
# not rely on being able to reuse the saved positions
# if we manage to switch the whole user-movement stuff to servo mode
# we could at least save target positions (where the motor would move
# to if we were cut off?)
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
        if monitor:
            with open(restartfile, 'w') as f:
                json.dump({'TEMPERATURES': TEMPERATURES, 'POSITIONS': POSITIONS, \
                           'wrap':monitor.wrap}, f, sort_keys=True, indent=4)

        time.sleep(10)
        os.sync()
threading.Thread(target=save_restart).start()



## MAIN


last_savepos = None
camera = None
recordings = []
motor_timeout = MOTOR_TIMEOUT

# TODO FIXME how to treat upper-case characters?
# do we want to distinguish, or do we want to be failsafe against capslock?
# (we probably are not anyway because of the F1... and 1... keys

# USER-INTERFACE FUNCTIONS
# the movement keys are defined above and will be automatically mapped
# move this up to declaration section? could in principle store in json
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
    ord('l'): (CLI.toggle_led,),
    ord('h'): (CLI.toggle_heater,),
    ord('t'): (CLI.enter_temperature_ramp,),
    ord('x'): (CLI.lifoff,),
    ord('c'): (CLI.camera,),
    ord('?'): (CLI.user_help,)
}


with vm.Motors(device=st_device, log=log, axes_map=SERVO_AXIS_MAP, motorconf=SERVOS) as motors:
    # TODO FIXME
    monitor = vm.ServoMonitor(motors,increment=MONITOR_INTERVAL)

    for ax in motors.axes:
        vm.ServoWheelMode(motors._servos[ax])
        print("max torque",ax,motors._servos[ax].eeprom.read_max_torque())
    motors.stop_all()
    motor_moving = False

    ## PHASE 1: USER INTERACTION BEFORE LIFT OFF
    # this is the main before-lift-off loop for user interaction
    # before lift off we are connected via umbilical
    # allow user to remote control with keyboard commands
    if not status['LO']:
        log.info("PHASE 1: INTERACTIVE MODE - WAITING FOR LIFT OFF")
        stop_event = threading.Event()
        threading.Thread(target=wait_for_lo,args=[stop_event]).start()
        with vesicolos_utils.cli.CLI(motors,moitor,led,heater,camera,keymap,arrowmap) as cli:
            threading.Thread(target=cli.start,args=[stop_event]).start()
    
            while not status['LO']:
                # TODO FIXME
                # monitoring of stuff goes here while the UI is running
                #for ax in motors.axes:
                #    print("torque",ax,motors._servos[ax].sram.read_current_load())
                #print(motors.controller.broadcast.sram.sync_read_current_load(motors.servo_ids))
                if stop_event.is_set():
                    # stop_event will be set by UI if the user exits
                    # or by the wait_for_lo if that enconuters a timeout
                    # (which is currently not implemented)
                    break
            # the UI and the wait_for_lo thread will listen to this:
            stop_event.set()

            # TODO FIXME: now is the time to read from cli the stored positions

            manual_lift_off = not cli.stop and not status['LO']
            stop = cli.stop # True is user told us to stop
            if not stop and not status['LO']:
                # user didn't tell to stop, but LO signal didn't come
                # this is treated as a "manual lift-off"
                manual_lift_off = True
                status['LO'] = True

        # cleanup, only needed if we didn't have LO right away
        motors.stop_all()
        motors.wheel_mode()

    #  SECOND PHASE
    # TODO:FIXME from here
    
    # we have a lift-off, so remote control is off now
    
    if not stop:
        motors.stop_all()
        if not manual_lift_off:
            # this is a real lift off, we wait for mug now
            t0 = time.time()
            # we record the ascent for sure, but if the user started before
            # we do not interrupt them
            if camera is None:
                # kill external previewer app if running, we want the cam ours now
                kill_proc_by_name(rpicam_process, log)
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
    
    
    # TODO: FIXME move this to vesicolos_utils/temperature.py ?
    class TemperatureController ():
        # TODO: make this carry its own logfile handle, init with fname
        # this is not really a logger, it is just an open file
        # maybe pass just the fh here so that we can embed in with ... as fh
        # or with TemperatureController(logfile) as Temp:
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

# on shutdown, the motors should stop!
# this is handled by the Motors context handler

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
#try:
#    GPIO.setup(GPIO_LED, GPIO.OUT)
#    GPIO.setup(GPIO_HEATER, GPIO.OUT)
#    GPIO.output(GPIO_LED, False)
#    GPIO.output(GPIO_HEATER, False)
#except Exception as err:
#    print('ERR' + str(err))

# stop all servos, close port

if monitor:
    monitor.stop()
prog_end = True

print('END')
log.info('EXIT')
