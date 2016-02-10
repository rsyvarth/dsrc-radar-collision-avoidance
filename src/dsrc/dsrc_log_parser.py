
from util.logger import LogParser

class DsrcLogParser(LogParser):
    """DSRC Log Parser - Reads the provided log file and emits the logs in real-time

    Extends: util.logger.LogParser
    """
    def __init__(self, callback = None, log_file=None):
        """Init log parser, read log file into memory"""
        LogParser.__init__(self, callback=callback)

        # read from log file here and parse, for now we have a fancy static array
        self.logs = [
            {'time': 0, 'data': [{'car_id': 10, 'source': 'dsrc', 'speed': 100}]},
            {'time': 0.01, 'data': [{'car_id': 11, 'source': 'dsrc', 'speed': 100}]},
            {'time': 0.05, 'data': [{'car_id': 12, 'source': 'dsrc', 'speed': 100}]},
            {'time': 0.1, 'data': [{'car_id': 13, 'source': 'dsrc', 'speed': 100}, {'car_id': 12, 'source': 'dsrc', 'speed': 100}]},
            {'time': 0.8, 'data': [{'car_id': 14, 'source': 'dsrc', 'speed': 100}]},
            {'time': 1, 'data': [{'car_id': 15, 'source': 'dsrc', 'speed': 100}]},
            {'time': 1.5, 'data': [{'car_id': 16, 'source': 'dsrc', 'speed': 100},{'car_id': 15, 'source': 'dsrc', 'speed': 100}]},
            {'time': 3, 'data': [{'car_id': 17, 'source': 'dsrc', 'speed': 100}]},
            {'time': 5, 'data': [{'car_id': 18, 'source': 'dsrc', 'speed': 100}]}
        ]

    def run(self):
        """Start emitting log events based on their timestamps"""
        LogParser.run(self)
