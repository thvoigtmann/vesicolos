import threading

import sys

import time

class MyTimeoutError(Exception):

    # exception for our timeouts

    pass

def timeout_func(func, args=None, kwargs=None, timeout=30, default=None):

    """This function will spawn a thread and run the given function

    using the args, kwargs and return the given default value if the

    timeout is exceeded.

    http://stackoverflow.com/questions/492519/timeout-on-a-python-function-call

    """

    class InterruptableThread(threading.Thread):

        def __init__(self):

            threading.Thread.__init__(self)

            self.result = default

            self.exc_info = (None, None, None)

        def run(self):

            try:

                self.result = func(*(args or ()), **(kwargs or {}))

            except Exception as err:

                self.exc_info = sys.exc_info()

        def suicide(self):

            raise MyTimeoutError(

                "{0} timeout (taking more than {1} sec)".format(func.__name__, timeout)

            )

    it = InterruptableThread()

    it.start()

    it.join(timeout)

    if it.exc_info[0] is not None:

        a, b, c = it.exc_info

        raise Exception(a, b, c)  # communicate that to caller

    if it.is_alive():

        print ("is alive, suicide")
        it.suicide()

        raise RuntimeError

    else:

        return it.result

def _fit_distribution():

    # some long operation which run for undetermined time

    for i in range(50):

        print(i)
        time.sleep(i)

try:

    timeout_func(_fit_distribution, timeout=2)

except MyTimeoutError as ex:

    print(ex)
