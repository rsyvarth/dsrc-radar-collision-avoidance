
from util.logger import LogParser

class RadarLogParser(LogParser):
    """Radar Log Parser - Reads the provided log file and emits the logs in real-time

    Extends: util.logger.LogParser
    """
    def __init__(self, callback=None, log_file=None):
        """Init log parser, read log file into memory"""
        LogParser.__init__(self, callback=callback)

        # read from log file here and parse, for now we have a fancy static array
        self.logs = [
            {'time': 0.02, 'data': [{'car_id': 1, 'source': 'radar', 'distance': 100}]},
            {'time': 0.08, 'data': [{'car_id': 2, 'source': 'radar', 'distance': 100}]},
            {'time': 0.1, 'data': [{'car_id': 3, 'source': 'radar', 'distance': 100}]},
            {'time': 0.27, 'data': [{'car_id': 4, 'source': 'radar', 'distance': 100},{'car_id': 3, 'source': 'radar', 'distance': 100},{'car_id': 2, 'source': 'radar', 'distance': 100}]},
            {'time': 0.58, 'data': [{'car_id': 5, 'source': 'radar', 'distance': 100}]},
            {'time': 1.2, 'data': [{'car_id': 6, 'source': 'radar', 'distance': 100}]},
            {'time': 1.8, 'data': [{'car_id': 7, 'source': 'radar', 'distance': 100},{'car_id': 5, 'source': 'radar', 'distance': 100}]},
            {'time': 2.5, 'data': [{'car_id': 8, 'source': 'radar', 'distance': 100}]},
            {'time': 5.5, 'data': [{'car_id': 9, 'source': 'radar', 'distance': 100}]},
            {'time': 6, 'data': [{'car_id': 'LAST LOG, ^C NOW', 'distance': 100}]},
        ]

    def run(self):
        """Start emitting log events based on their timestamps"""
        LogParser.run(self)
