from threading import Thread
import time
import logging

class RadarDataParser(Thread):
    """ Listens for new Radar messages over CAN and parses for the dispatcher

    This parser reads messages from the CAN Bus using the Kvaser USB Python SKD
    and formats Radar information into a python object. Then we send the data
    along to the event dispatcher.
    """
    def __init__(self, callback = None, log = True):
        """Initialize the data parser, connect to the can bus"""
        Thread.__init__(self)
        self.callback = callback
        self.log = log

    def run(self):
        """Start reading data from the CAN Bus and sending full objects to the dispatcher"""
        # NOTE currently this just generates random numbers and sends them to the
        # dispatcher at random intervals
        import random
        time.sleep(random.random()*2)
        while True:
            data = [random.randint(0,100)]
            if self.log:
                logging.debug(data)

            self.callback(self, data)
            time.sleep(random.random()*2)
