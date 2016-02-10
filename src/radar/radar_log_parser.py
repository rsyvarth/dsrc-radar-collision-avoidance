from util.logger import LogParser

class RadarLogParser(LogParser):
    def __init__(self, callback=None, log_file=None):
        LogParser.__init__(callback=callback)

        # read from log file here
        # and parse
        self.logs = [
            {'time': 0.02, 'data': [{'car_id': 1, 'speed': 100}]},
            {'time': 0.08, 'data': [{'car_id': 2, 'speed': 100}]},
            {'time': 0.1, 'data': [{'car_id': 3, 'speed': 100}]},
            {'time': 0.27, 'data': [{'car_id': 4, 'speed': 100}]},
            {'time': 0.58, 'data': [{'car_id': 5, 'speed': 100}]},
            {'time': 1.2, 'data': [{'car_id': 6, 'speed': 100}]},
            {'time': 1.8, 'data': [{'car_id': 7, 'speed': 100}]},
            {'time': 2.5, 'data': [{'car_id': 8, 'speed': 100}]},
            {'time': 5.5, 'data': [{'car_id': 9, 'speed': 100}]}
        ]
