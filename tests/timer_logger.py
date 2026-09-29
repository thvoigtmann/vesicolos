import threading
import time

class Test:
    def __init__ (self, status={}):
        self.increment = 1
        self.s = status
        self.done = False
        self.next_t = time.time()
        self._run()
    def _run (self):
        if not self.done:
            print ("S",self.s)
            while self.next_t < time.time():
                self.next_t += self.increment
            threading.Timer(self.next_t - time.time(), self._run).start()
    def stop (self):
        self.done = True
    def start (self):
        self.done = False
        self.next_t = time.time()
        self._run()

status = {'LO':0 }

monitor = Test(status)

time.sleep(12)
status['LO'] = 1
time.sleep(8)

monitor.stop()
