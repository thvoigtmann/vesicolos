import signal, threading, time

running = False

def handler(signum, frame):
    global running
    print("Forever is over!")
    running = False
    raise Exception("end of time")

def loop_forever():
    import time
    while 1:
        print("sec")
        time.sleep(1)

signal.signal(signal.SIGALRM, handler)

def timerjob ():
    global running
    signal.alarm(10)
    t0 = time.time()
    while running:
        time.sleep(1)
        print("tic toc")
        if time.time() > t0 + 17:
            print("simulated external event")
            signal.raise_signal(signal.SIGALRM)
            break
    print("DING")
    signal.alarm(0)


running = True
threading.Thread(target=timerjob).start()
try:
    loop_forever()
except Exception as exc:
    print(exc)

signal.alarm(0)
