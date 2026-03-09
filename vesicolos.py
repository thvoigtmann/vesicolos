import sys, os, psutil
import time
import threading, signal
import json
import logging, errno

try:
    import RPi.GPIO as GPIO
except Exception as e:
    if os.environ.get("NO_GPIO"):
        import Mock.GPIO as GPIO
    else:
        raise e
import gpiozero                            # used for PWM (LED and heater)
import board, digitalio, adafruit_max31865 # used for temperature sensor
GPIO.setmode(GPIO.BCM)

try:
    import picamera2
except:
    print("picamera2 not available, continuing without camera control")

# local repositories
sys.path.append('python-st3215/src')
sys.path.append('.')
from python_st3215 import ST3215
from vesicolos_utils import getkey,Keys,kill_proc_by_name,DummyGPIO
from vesicolos_utils import logSetup, load_restart, save_restart
import vesicolos_utils.motors as vm
import vesicolos_utils.temperature as vt
from vesicolos_utils.cli import CLI, keymap


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
SERVOS = {
  '_default_': { 'SPEED_INC': 200 },
  'Z': { 'SPEED_INC': 40, 'MAX_WRAP': 1 }
}
# mapping of keys to control the Cartesian axes and their directions
# this is currently configured to work in inverted mode, so that arrow
# keys are intuitive if one watches the image on the camera
# one could revert the directions if the keys should correspond to the
# way the sample slide actually moves
SERVO_CMDS = {
    Keys.LEFT:   { 'axis': 'Y', 'dir': +1 },
    Keys.RIGHT:  { 'axis': 'Y', 'dir': -1 },
    Keys.DOWN:   { 'axis': 'X', 'dir': -1 },
    Keys.UP:     { 'axis': 'X', 'dir': +1 },
    Keys.PGUP:   { 'axis': 'Z', 'dir': -1 },
    Keys.PGDOWN: { 'axis': 'Z', 'dir': +1 }
}

# state variables are those that will be continuously saved to a restart file
# and read from there upon startup
# they reflect everything that the user or the hardware state could modify
# (stored positions, stored temperature profiles, wrap counter for motors)
# and that we want back after a program cycle
# these could be power cycles, so we try to catch what we can
# what is configurable here is the default temperature profile
STATE_VARS = {
    'user.positions': {},
    'user.temperatures': {
        'default': { 'Tmin': 25, 'Tmax': 40, 'dt': 30, 'ts': 10, 'tmax': 90 }
    },
    'motor.pos': {},
    'motor.wrap': {}
}
TMAX_DEFAULT = 50 # should not be used if we have 'tmax' keys
# the temperature profile is defined like this:
#     T(t) = Tmin                              for t < ts
#     T(t) = Tmin + (Tmax-Tmin)*(t-ts)/dt      for ts < t < ts+dt
#     T(t) = Tmax                              for ts+dt < t < tmax
# where t=0 is set by the time a stored position is first moved to
T_SIGNATURE = ["Tmin","Tmax","dt","ts","tmax"]
def T(t,Tmin,Tmax,dt,ts,tmax):
    if t>tmax:
        # should not happen anyway, but should make the heater go off
        return 0.
    tau = t-ts
    if tau <= 0:
        return Tmin
    if tau >= dt:
        return Tmax
    return Tmin + (Tmax - Tmin) * tau/dt

# files that will be written by the process
RESTARTFILE = 'vesicolos-restart.json'
# all these files will reside in a run-specific directory that is created
LOGPATH = '%Y-%m-%d-%H-%M-%S' # will be used within strftime
LOGFILE = 'vesicolos.log'
TEMPERATURE_LOG = 'temperature.log'
CAMFILE = 'capture-{pos}.h264' # could use {frame:06d} or something
PTSFILE = 'capture-{pos}-pts.txt'
RPICAM_PROCESS = 'rpicam-vid'


## HARDWARE SETTINGS

#
# these settings reflect the actual hardware configuration of VESCIOLOS
# there should be no need to change these once the hardware is fixed
#
# GPIO pin layout used
STATUS_PINS = { 'LO': 17, 'mug': 27 } # GPIO pins used for signals
GPIO_LED = 13                  # GPIO pin used for LED (PWM)
GPIO_HEATER = 12               # GPIO pin used for heater (PWM)
# TODO FIXME do we really need to use board.D5 or can we put num value?
try:
    GPIO_TEMP = board.D5           # cable select for temp sensor (SPI) on GPIO 5
except:
    GPIO_TEMP = 5
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




## STARTUP CODE

# TODO FIXME temperature log needs to be setup
log, path = logSetup(LOGPATH, LOGFILE, TEMPERATURE_LOG)
camfile = os.path.join(path,CAMFILE)
ptsfile = os.path.join(path,PTSFILE)

# stop: True is user requested exit (no mug program is run)
# manual_lift_off: True is user set LO instead of GPIO signal
stop = False
manual_lift_off = False

# GPIO signals
status = { _: False for _ in STATUS_PINS }
for pin in STATUS_PINS.values():
    GPIO.setup(pin, GPIO.IN)

# LED uses hardware PWM, taken care of by the gpiozero module
try:
    led = gpiozero.LED(GPIO_LED)
    led.off()
except Exception as e:
    log.error('LED init failed: '+str(e))
    log.error('LED externally controlled')
    led = None

# heater uses hardware PWM, taken care of by the gpiozero module
try:
    heater = gpiozero.PWMOutputDevice(GPIO_HEATER)
    heater.off()
except Exception as e:
    log.error('HEATER init failed: '+str(e))
    log.error('heater externally controlled')
    heater = None

# temperature sensor, SPI on a MAX31865 board, taken care of by adafruit module
try:
    spi = board.SPI()
    cs = digitalio.DigitalInOut(GPIO_TEMP)
    temp_sensor = adafruit_max31865.MAX31865(spi, cs, \
        rtd_nominal=RTD_NOMINAL, ref_resistor=RTD_REFERENCE, wires=RTD_WIRES)
except Exception as e:
    log.error('TEMPSENSOR init failed: '+str(e))
    log.error('no temperature sensor available')
    temp_sensor = None

# TODO: FIX THIS
#temperature_log = Logger(temperature_logfile)

# set UART device depending on raspberry model or environment variables
rpi_model = os.environ.get("RASPI_MODEL","_default_")
log.info(f"assuming Raspberry model {rpi_model}")
st_device = ST_DEVICE.get(rpi_model,ST_DEVICE['_default_'])
st_device = os.environ.get("ST_DEVICE",st_device)
log.info(f"using {st_device} as UART device")

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



## MAIN


camera = None
motor_timeout = MOTOR_TIMEOUT

# load restart file
# this is supposed to catch power cycles, in particular allowing for
# the case where some positions to search are stored ahead of
# integration into the rocket, or if lift-off causes a power cycle
# TODO FIXME if we crash while the motors are moving, what happens?
load_restart (STATE_VARS, RESTARTFILE, log)

prog_end = threading.Event()


with vm.MotorController(device=st_device, log=log, axes_map=SERVO_AXIS_MAP, motorconf=SERVOS) as motor_controller:
    # TODO FIXME not ServoMonitor, this is a general monitor
    # give it also the temperature sensor
    monitor = vm.ServoMonitor(motor_controller,increment=MONITOR_INTERVAL,state=STATE_VARS)
    # the monitor will set state_valid to False is the positions read
    # from the restart file don't match the ones read from the motors
    if not monitor.state_valid:
        # the monitor will have already warned in the log file
        # we invalidate any stored positions
        STATE_VARS["user.positions"] = {}

    threading.Thread(target=save_restart,args=[prog_end,RESTARTFILE,STATE_VARS]).start()

    motor_controller.stop_all()
    motor_controller.wheel_mode()
    for ax in motor_controller.axes:
        # TODO FIXME
        print("max torque",ax,motor_controller._servos[ax].eeprom.read_max_torque())

    ## PHASE 1: USER INTERACTION BEFORE LIFT OFF

    # this is the main before-lift-off loop for user interaction
    # before lift off we are connected via umbilical
    # allow user to remote control with keyboard commands

    if not status['LO']:
        log.info("PHASE 1: INTERACTIVE MODE - WAITING FOR LIFT OFF")
        stop_event = threading.Event()
        threading.Thread(target=wait_for_lo,args=[stop_event]).start()
        with CLI(motor_controller=motor_controller,monitor=monitor,led=led,heater=heater,keymap=keymap,movement_map=SERVO_CMDS,state=STATE_VARS) as cli:
            threading.Thread(target=cli.start,args=[stop_event]).start()
    
            try:
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
            except KeyboardInterrupt:
                cli.stop = True
            # the UI and the wait_for_lo thread will listen to this:
            stop_event.set()

            # now is the time to read from cli the stored positions
            # in fact, this shouldn't even be necessary because the cli
            # will update those STATE_VARS in place
            STATE_VARS['user.positions'], STATE_VARS['user.temperatures'] \
                = cli.read_user_settings()
            camera = cli.camera # maybe the user started a camera

            manual_lift_off = not cli.stop and not status['LO']
            stop = cli.stop # True means user told us to stop
            if not stop and not status['LO']:
                # user didn't tell to stop, but LO signal didn't come
                # this is treated as a "manual lift-off"
                manual_lift_off = True
                status['LO'] = True

        # cleanup, only needed if we didn't have LO right away
        motor_controller.stop_all()
        motor_controller.wheel_mode()


    ## PHASE 2: WAITING AFTER LIFT OFF

    # we have a lift-off, so remote control is off now
    # we now wait for either microgravity or for the corresponding timeout

    if not stop:
        motor_controller.stop_all()
        if not manual_lift_off:
            # this is a real lift off, we wait for mug now
            t0 = time.time()
            # we record the ascent for sure, but if the user started before
            # we do not interrupt them
            if camera is None:
                # kill external previewer app if running, we want the cam ours now
                kill_proc_by_name(RPICAM_PROCESS, log)
                try:
                    camera = CameraController(camfile,pts=ptsfile,keys={'pos':'liftoff_auto'})
                    threading.Thread(target=camera.record).start()
                    led.on()
                except RuntimeError as e:
                    camera = None
                    log.error("camera error "+str(e))
            while not GPIO.input(STATUS_PINS['mug']):
                log.info("waiting for microgravity")
                time.sleep(0.5)
                if time.time() - t0 >= SOE_TIMEOUT:
                    log.write("SOE by timeout")
                    break
            if camera is not None:
                camera.stop()
            status['mug'] = True
    
    
    ## PHASE 3: MICROGRAVITY EXPERIMENT

    # microgravity timeout: handled by UNIX ALRM signal
    # but the loop here polls the mug pin and sends the signal once that
    # flag goes off, cancelling possibly before

    def microgravity_timeout ():
        global status, manual_lift_off, log
        #signal.alarm(EXP_TIMEOUT)
        t0 = time.time()
        while MUG_STICKY or GPIO.input(STATUS_PINS['mug']) or manual_lift_off:
            time.sleep(1)
            if time.time() - t0 >= EXP_TIMEOUT:
                log.info("SOE OFF by timeout")
                break
        status['mug'] = MUG_STICKY or bool(GPIO.input(STATUS_PINS['mug'])) \
                        or manual_lift_off
        log.info("END OF MUG")
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
    
    
    if not stop: # TODO FIXME move this down?
        Tcontrol = TemperatureController()



    # CORE MICROGRAVITY EXPERIMENT PROCEDURE

    def microgravity_experiment ():
        positions = sorted(STATE_VARS['user.positions'].keys() or ['default'])
        log.info("mug sequence: positions "+" / ".join(positions))
        led.on()
        recordings = []
        while status['mug'] or manual_lift_off:
            for pos in positions:
                ckey = make_camera_key(recordings, pos, pre='mug_')
                recordings.append(ckey)
                try:
                    camera = CameraController(camfile,pts=ptsfile,keys={'pos':ckey})
                    threading.Thread(target=camera.record).start()
                except RuntimeError as e:
                    camera = None
                    log.error('camera error '+str(e))
                if not pos == 'default':
                    motor_controller.move_to_stored_position (STATE_VARS['user.positions'][pos], monitor.wrap)
                    do_zstack = True
                else:
                    do_zstack = False
                Tcontrol.set_profile (pos)
                # take some time, do z-stacks
                tmax = STATE_VARS['user.temperatures'].get(pos,{}).get('tmax',TMAX_DEFAULT)
                tmax = tmax + time.time()
                if 'Z' in axes:
                    motor_contoller.wheel_mode('Z',wheel=False)
                    # do_zstack shall only be true if we don't have
                    # motor errors
                    # TODO FIXME
                    #do_zstack &= motorDriver.success(comm, err)
                    time.sleep(0.2)
                    motor_controller.set_middle('Z')
                    #do_zstack &= motorDriver.success(comm, err)
                    time.sleep(0.2)
                    # after set middle, motor is in servo mode, pos 2048
                    # set zpos to highest position first
                    # TODO FIXME FIXME this relies on the fact that we
                    # can fiddle with the position in servo mode but this
                    # doesn't destroy the wheel mode positions that we
                    # use for recalling stored positions
                    # do we really need this???
                    # can we not just read the current position and
                    # rely on the fact that it will be within 4096
                    # and thus the following code should work if we
                    # replace 2048 by the current position??
                    zpos = 2048 + MOTOR_DZ_STEPSIZE*int(MOTOR_DZ_STEPS/2)
                    zcnt = 0
                    zdirection = -1
                    motor_controller.goto_position('Z',zpos)
                while True:
                    if do_zstack:
                        if zcnt >= MOTOR_DZ_STEPS:
                            zdirection = -zdirection
                            zcnt = 0
                        zcnt += 1
                        zpos += zdirection*MOTOR_DZ_STEPSIZE
                        motor_controller.goto_position('Z',zpos)
                    time.sleep(MOTOR_DZ_WAIT)
                    if time.time() > tmax:
                        break
                if 'Z' in axes:
                    motor_controller.wheel_mode('Z',wheel=True)
                    time.sleep(0.2)
                if not camera is None:
                    camera.stop()
    
    
    if not stop:
        # now start the actual measurement program
        # timeout handling is done by SIGALRM here
        # we now also need temperature control
        signal.signal(signal.SIGALRM, mug_timeout_handler)
        threading.Thread(target=microgravity_timeout).start()

        with vt.TemperatureController(heater,temp_sensor,STATE_VARS['user.temperatures'],T,T_SIGNATURE,log) as Tcontrol:

            threading.Thread(target=Tcontrol.start).start()
            try:
                microgravity_experiment()
            except MicrogravityTimeout as msg:
                log.info(str(msg))

        signal.alarm(0) # clear any remaining ALRM signals

    monitor.stop()


## SHUTDOWN

print('SHUTDOWN')
prog_end.set()
os.sync()

# on shutdown, the motors should stop!
# this is handled by the MotorController context handler
# that should have by now also closed the serial port

if led is not None:
    led.off()
    led.close()
    led = None
if heater is not None:
    heater.off()
    heater.close()
    heater = None
if camera is not None:
    camera.stop()
    camera = None

# TODO FIXME the LED and the heater seem to come back on once the code
# quits, is there some GPIO fiddling that we can do to prevent that?
#try:
#    GPIO.setup(GPIO_LED, GPIO.OUT)
#    GPIO.setup(GPIO_HEATER, GPIO.OUT)
#    GPIO.output(GPIO_LED, False)
#    GPIO.output(GPIO_HEATER, False)
#except Exception as err:
#    print('ERR' + str(err))

print('END')
log.info('EXIT')
