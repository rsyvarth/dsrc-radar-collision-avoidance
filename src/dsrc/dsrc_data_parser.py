from threading import Thread
import time, logging, json


class DsrcDataParser(Thread):
    """ Poll DSRC API for new data, when new data is found pass along.

    This parser handles making calls to the DSRC to get updated information about
    the vehicle state. If the information retrieved is different from the previously
    obtained information we send the data along to the event dispatcher.
    """
    def __init__(self, callback=None, log=True):
        """ Initialize the data parser. """
        Thread.__init__(self)
        self.callback = callback
        self.log = log
        self.logger = logging.getLogger('dsrc')

    def run(self):
        """ Start reading data from the DSRC API. """
        # NOTE currently this just generates random numbers and sends them to the
        # dispatcher at random intervals
        import random
        time.sleep(random.random()*2)
        while True:
            data = [random.randint(0,100)]
            if self.log:
                # sends JSON data to dsrc log file
                self.logger.debug(str(
                    json.dumps(data,
                    separators=(',',':')))
                )

            self.callback(self, data)
            time.sleep(random.random()*2)

