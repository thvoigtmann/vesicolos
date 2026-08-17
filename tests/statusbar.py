import time
import bottombar as bb

with bb.add('CPU TEMP, SAMPLE TEMP') as item:
    with bb.add('line2') as item2:
        for i in range(60):
            time.sleep(1)
            print ("hello",i)
