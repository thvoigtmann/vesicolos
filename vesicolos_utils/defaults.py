## GLOBAL SETTINGS

from vesicolos_utils import Keys

#
# the settings here can be adjusted without changing the hardware setup
# see below for hardware-related settings
#

# timeout settings for the expected flight trajectory
# the defaults hard-coded here came from the MAPHEUS-16 flight
# and the MOSAIC timeline of that flight
# they will be overwritten if json files are found in CONFPATH
CONFPATH = 'flight_config'
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

# maximum time to spend on one T-controlled sample position
TMAX_DEFAULT = 50


# files that will be written by the process
RESTARTFILE = 'vesicolos-restart.json'
# all these files will reside in a run-specific directory that is created
LOGPATH = '%Y-%m-%d-%H-%M-%S' # will be used within strftime
LOGFILE = 'status.log'
TEMPERATURE_LOG = 'vesicolos.log'
CAMFILE = 'capture-{pos}.h264' # could use {frame:06d} or something
PTSFILE = 'capture-{pos}-pts.txt'
# name of rpicam process to kill if the user started it separately
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
import board
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
ST_MAX_ID = 10                 # maximum servo ID to scan for
# some servo configuration parameters are now part of Motors class
# servo configuration for the three axes
# values with lower-case names will be modified by the program
# the IDs are hard-coded in the motors
SERVO_AXIS_MAP = { 0: 'X', 9: 'Y', 1: 'Z' }


