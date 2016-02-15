from threading import Thread
import time
import logging

class RadarDataParser(Thread):
    """ Listens for new Radar messages over CAN and parses for the dispatcher.

    This parser reads messages from the CAN Bus using the Kvaser USB Python SKD
    and formats Radar information into a python object. Then we send the data
    along to the event dispatcher.
    """
    def __init__(self, callback = None, log = True):
        """ Initialize the data parser, connect to the can bus. """
        Thread.__init__(self)
        self.callback = callback
        self.log = log
        self.logger = logging.getLogger('radar')

    def run(self):
        """ Start reading data from the CAN Bus and sending full objects to the dispatcher. """
        # NOTE currently this just generates random numbers and sends them to the
        # dispatcher at random intervals
	import canlib
	cl = canlib.canlib()
	channels = cl.getNumberOfChannels()
	ch = 0; #Hard-coded, might need to change!
	if ch >= channels:
	    print("Invalid channel number")
	    sys.exit()
	
	try:
            ch1 = cl.openChannel(ch, canlib.canOPEN_ACCEPT_VIRTUAL)
            print("Using channel: %s, EAN: %s" % (ch1.getChannelData_Name(),
                                              ch1.getChannelData_EAN()))

            ch1.setBusOutputControl(canlib.canDRIVER_NORMAL)
            ch1.setBusParams(canlib.canBITRATE_500K)
            ch1.busOn()
    	except (canlib.canError) as ex:
            print(ex)

	while True:
            try:
            	msgId, msg, dlc, flg, time = ch1.read()
            	print("%9d  %9d  0x%02x  %d  %s" % (msgId, time, flg, dlc, msg))
            	for i in range(dlc):
                    msg[i] = (msg[i]+1) % 256
                    print(msg, ''.join('{:02x}'.format(x) for x in msg))
            	ch1.write(msgId, msg, flg)
            except (canlib.canNoMsg) as ex:
            	None
            except (canlib.canError) as ex:
            	print(ex)
	'''
        import random
        time.sleep(random.random()*2)
        while True:
            data = [random.randint(0,100)]
            if self.log:
                # goes the the Radar log file
                self.logger.debug(data)

            self.callback(self, data)
            time.sleep(random.random()*2)
	'''
