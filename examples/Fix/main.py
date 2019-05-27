# L76
# Created at 2019-03-26 09:45:18.145367

import streams
from quectel.l76 import l76


streams.serial()
try:
    gnss = l76.L76(SERIAL4)
    print("Starting...")
    gnss.start(D59)
    gnss.set_rate(1000)
    while True:
        #print(".")
        utc = gnss.utc()
        print("UTC",utc)
        if gnss.has_fix():
            fix = gnss.fix()
            if fix:
                for x in fix:
                    print("Fix",x)
            print("Pausing")
            gnss.pause()
            sleep(10000)
            print("Resuming")
            gnss.resume()
        sleep(1000)
except Exception as e:
    print("EXC!")
    print(e)
