
from dsrc.dsrc_data_parser import DsrcDataParser
from dsrc.dsrc_log_parser import DsrcLogParser

class DsrcEventDispatcher(object):
    def __init__(self, callback, log=True, log_file=None):
        self.callback = callback
        if not log_file:
            self.provider = DsrcDataParser(log=log, callback=self.on_message)
        else:
            self.provider = DsrcLogParser(log_file=log_file, callback=self.on_message)

    def start(self):
        self.provider.daemon = True # Kills the thread on ^C
        self.provider.start()

    def on_message(self, thread, data):
        self.callback(data)
