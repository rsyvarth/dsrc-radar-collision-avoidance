from threading import Thread

class LogParser(Thread):
    def __init__(self, callback=None):
        Thread.__init__(self)
        self.callback = callback
        self.log_file = log_file

        self.logs = []

    def run(self):
        for key in range(len(self.logs)): # TODO there is probably a better way to do this loop
            self.callback(self, self.logs[key]['data'])

            if key < len(self.logs) - 1:
                time.sleep(self.logs[key+1]['time'] - self.logs[key]['time'])
