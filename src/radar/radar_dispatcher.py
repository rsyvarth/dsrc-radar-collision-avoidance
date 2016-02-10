from radar.radar_data_parser import RadarDataParser
from radar.radar_log_parser import RadarLogParser

class RadarEventDispatcher(object):
    def __init__(self, callback, log=True, log_file=None):
        self.callback = callback
        if not log_file:
            self.provider = RadarDataParser(log=log, callback=self.on_message)
        else:
            self.provider = RadarLogParser(log_file=log_file, callback=self.on_message)

    def start(self):
        self.provider.daemon = True # Kills the thread on ^C
        self.provider.start()

    def on_message(self, thread, data):
        # print 'RadarEventDispatcher::on_message() ', thread, data
        self.callback(data)
