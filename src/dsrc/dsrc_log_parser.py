
from util.logger import LogParser

class DsrcLogParser(LogParser):
    def __init__(self, callback = None, log_file=None):
        LogParser.__init__(callback=callback)

        # read from log file here
        # and parse
        self.logs = [
            {'time': 0, 'cars': [{'car_id': 10, 'speed': 100}]},
            {'time': 0.01, 'cars': [{'car_id': 11, 'speed': 100}]},
            {'time': 0.05, 'cars': [{'car_id': 12, 'speed': 100}]},
            {'time': 0.1, 'cars': [{'car_id': 13, 'speed': 100}]},
            {'time': 0.8, 'cars': [{'car_id': 14, 'speed': 100}]},
            {'time': 1, 'cars': [{'car_id': 15, 'speed': 100}]},
            {'time': 1.5, 'cars': [{'car_id': 16, 'speed': 100}]},
            {'time': 3, 'cars': [{'car_id': 17, 'speed': 100}]},
            {'time': 5, 'cars': [{'car_id': 18, 'speed': 100}]}
        ]
