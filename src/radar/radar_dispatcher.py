
from radar.radar_data_parser import RadarDataParser
from radar.radar_log_parser import RadarLogParser

class RadarEventDispatcher(object):
    """Radar Event Dispatcher - The interface between our data parsers and the combiner

    The dispatcher instantiates the proper parser for the current environment.
    Once the correct parser has been created we manage the parser thread which
    runs separately from the rest of the application to reduce the overhead of
    I/O on the application's performance. The dispatcher also passes data back
    to the callback function (Combiner) when new data is available.
    """
    def __init__(self, callback, log=True, log_file=None):
        """Initialize dispatcher, instantiate the proper parser for the current environment"""
        self.callback = callback
        if not log_file:
            self.provider = RadarDataParser(log=log, callback=self.on_message)
        else:
            self.provider = RadarLogParser(log_file=log_file, callback=self.on_message)

    def start(self):
        """Start running the parser's thread (After this data starts flowing)"""
        self.provider.daemon = True # Kills the thread on ^C
        self.provider.start()

    def on_message(self, thread, data):
        """Pass messages from the parser to the callback function"""
        # Maybe we will want to do some data parsing here at some point?
        self.callback(data)
