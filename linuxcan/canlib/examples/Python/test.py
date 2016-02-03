import sys
#sys.path.append("C:/temp/Canlib_SDK_v5.9/Samples/Python")

import canlib

cl = canlib.canlib()
print "canlib version: %s" % cl.getVersion()

channel = 0
handle1 = cl.openChannel(channel, canlib.canOPEN_ACCEPT_VIRTUAL)
print "Using channel: %s, EAN: %s" % (handle1.getChannelData_Name(),
                                      handle1.getChannelData_EAN())

handle1.setBusOutputControl(canlib.canDRIVER_NORMAL)
handle1.setBusParams(canlib.canBITRATE_1M)
handle1.busOn()

