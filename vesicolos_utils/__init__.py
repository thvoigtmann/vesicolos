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
