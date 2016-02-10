from threading import Thread
import time

class LogParser(Thread):
    """ Base log parsing class to hold common parsing functions. """

    def __init__(self, callback=None):
        """ Setup the log parser. """
        Thread.__init__(self)
        self.callback = callback
        self.logs = []

    def run(self):
        """ Emit the log data in real-time. """
        time.sleep(self.logs[0]['time']) # Sleep for the first log

        for key in range(len(self.logs)): # TODO there is probably a better way to do this loop
            self.callback(self, self.logs[key]['data'])

            # Calculate how long elapsed between now and the next log entry so we can delay for the right amount of time
            if key < len(self.logs) - 1:
                time.sleep(self.logs[key+1]['time'] - self.logs[key]['time'])
