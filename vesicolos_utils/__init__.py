import time, os, sys
import logging
import json

from .getkey import getkey, Keys

def kill_proc_by_name(name, log):
    """Kill a UNIX process given its name, using `psutil`."""
    import psutil, time
    # find processes by name using pstuil
    processes = [proc for proc in psutil.process_iter(['name']) \
                 if proc.info['name'] == name]
    for proc in processes:
        try:
            proc.kill()
            time.sleep(0.2) # give it time to die
        except psutil.NoSuchProcess:
            log.error(f"No such process: {proc.pid}")
        except psutil.AccessDenied:
            log.error(f"Access denied to {proc.pid}")

# setup logging
def logSetup (path_template, logfile, templogfile):
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

    path = time.strftime(path_template)
    try:
        os.mkdir(path)
    except:
        log.critical(f"cannot create output directory {path}")
        sys.exit(errno.ENOENT)

    _file_log = logging.FileHandler(os.path.join(path,logfile))
    _file_log.setFormatter(_log_formatter)
    log.addHandler(_file_log)
    # TODO FIXME SETUP temperature log here!? TEMPERATURE_LOG
    return log, path 


def load_restart (state, restartfile, log):
    try:
        with open(restartfile, 'r') as f:
            data = json.load(f)
            for k in state:
                 if k in data:
                     state[k] = data[k]
        log.info("re-loaded state from restart file")
    except FileNotFoundError:
        pass

# save restart file every 10 seconds
# intended to be called from its own subthread
def save_restart (prog_end, restartfile, state):
    cnt = 0
    while not prog_end.is_set():
        if cnt >= 5:
            with open(restartfile, 'w') as f:
                json.dump(state, f, sort_keys=True, indent=4)
            cnt = 0
        cnt += 1
        time.sleep(2)
        os.sync()
    # at program end, write again for sure
    with open(restartfile, 'w') as f:
        json.dump(state, f, sort_keys=True, indent=4)
    os.sync()

def make_camera_key (recordings, key, pre=''):
    """Generate a unique key for a new camera recording, avoiding those
    already present in `recordings`, by appending numerical values to
    `key`, prefixed with `pre`."""
    ckey = pre + key + '_{i:02d}'
    i = 1
    while ckey.format(i=i) in recordings and i<99:
        i += 1
    return ckey.format(i=i)


# this is currently no longer in used but can be used instead of None
# for led, heater when the initialization fails
class DummyGPIO(object):
    """A drop-in replacement for gpiozero LED and PWMOutputDevice
    objects that we can use to do nothing when the GPIO init fails."""
    def __init__(self,log,name):
        self.name = name
        self.log = log
        self.is_active = False
    def on(self):
        self.log.warn(f"{self.name} on = noop (GPIO init error)")
    def off(self):
        self.log.warn(f"{self.name} off = noop (GPIO init error)")
    def toggle(self):
        self.log.warn(f"{self.name} toggle = noop (GPIO init error)")
    def close(self):
        pass


# the following was developed for testing arithmetic with different
# signed words, as there was confusion how to represent negative numbers
# in the version of servos/driver we use, they use 15 significant bits
# and the leading bit is a sign bit, and words are represented in bigendian
# order - in the end, the code will use that directly, this class is
# just if you want to play it safe and understandable

class Word16:
    def __init__ (self, value=0, signed=True, bitsigned=False, bigendian=False, safe_bound=False):
        """Represent a 16bit word.

        Parameters:
        -----------
        value : int
            Value stored in this 16-bit word.
        signed : bool (default: True)
            A signed word stores [-32768,32767], an unsigned word
            stores [0,65535].
        bitsigned : bool (default: False)
            A word with a sign bit, stores the sign in the most significant
            bit, accepts ranges [-32767,32767] with ambiguity at zero.
            Mutually exclusive with `signed`. If `bitsigned` is set to
            `True`, this takes precedence.
            Note that `bitsigned` and assignment with negative numbers
            gives strange results on machines that natively use the standard
            signed-word convention; in that case, it might be advisable to
            set `safe_bound=True` as well.
        bigendian : bool (default: False)
            A big-endian representation of the word returns the sequence
            `lo, hi` when `to_bytes()` is called.
            A little-endian representation returns `hi, lo`.
            Here, `lo` and `hi` are values in the range [0,255].
        safe_bound : bool (default: False)
            Determine how `value` is treated if it is outside the permissible
            range. If set to `False`, the value is taken `& 0xFFFF`, so
            that outside-range values wrap around. If set to `True`, the
            value is gently clipped to the maximum/minimum representable
            values in the 16-bit word (accounting for signednees).
        """
        self.bigendian = bigendian
        if bitsigned:
            signed = False
        self.__dict__['signed'] = signed
        self.__dict__['bitsigned'] = bitsigned
        self.safe_bound = safe_bound
        self._set_value (value)
    def _set_value (self, value):
        if not self.safe_bound:
            val = value & 0xFFFF
            if val >= (1<<15) and self.signed:
                val = val - 65536
            if self.bitsigned and (val & 0x8000):
                val = -(val & 0x7FFF)
        else:
            if self.signed:
                val = max(min(value,(1<<15)-1),-(1<<15))
            elif self.bitsigned:
                val = max(min(value,(1<<15)-1),1-(1<<15))
            else:
                val = min(value,(1<<16)-1)
        self.__dict__['value'] = val
    def __setattr__(self, name, value):
        """If `w` is a `Word16`, we allow `w.value = int` to set the value,
        representing the settings of safe clipping and signedness that
        `w` was created with. We also allow `w.value = a, b` setting the
        word value from a byte sequence, respecting the endianness set for
        `w`.

        If `w.signed` is assigned a new value, the stored values is converted
        in its signedness."""
        if name == 'value':
            if type(value) == tuple:
                self.from_bytes(*value)
            else:
                self._set_value(value)
        elif name == 'signed':
            self.__dict__[name] = value
            if self.signed:
                self.__dict__['bitsigned'] = False
            self.from_bytes(*self.to_bytes())
        elif name == 'bitsigned':
            self.__dict__[name] = value
            if self.bitsigned:
                self.__dict__['signed'] = False
            self.from_bytes(*self.to_bytes())
        else:
            self.__dict__[name] = value
    def from_bytes (self, a, b):
        """Assign the 16-bit value from a two-byte sequence.
        If `self` is a big-endian word, the values are `lo, hi`.
        If `self` is a little-endian word, the values are `hi, lo`."""
        if self.bigendian:
            self.value = (a & 0xFF) | ((b & 0xFF) << 8)
        else:
            self.value = (b & 0xFF) | ((a & 0xFF) << 8)
        if self.value >= (1<<15) and self.signed:
            self.value = self.value - 65536
        if self.bitsigned and (self.value & 0x8000):
            self.value = -(self.value & 0x7FFF)
        return self
    def to_bytes (self):
        """Return the value represented as a two-byte sequence.
        If `self` is big-endian, returns `lo, hi`, else `hi, lo`."""
        if not self.bitsigned:
            lo, hi = self.value & 0xFF, (self.value >> 8) & 0xFF
        else:
            lo, hi = abs(self.value) & 0xFF, (abs(self.value) >> 8) & 0x7F
            if self.value < 0:
                hi |= 0x80
        if self.bigendian:
            return lo, hi
        else:
            return hi, lo
    def swap_endianness (self):
        """Change the endianness of the word. Does not change the value,
        but the interpretation of the byte sequences in `from_bytes`
        and `to_bytes`."""
        self.bigendian = not self.bigendian
        return self
    def __int__ (self):
        return int(self.value)
