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
