from multiprocessing import Process
import time, json, logging
import datetime as dt
import signal, os


class LogParser(Process):
    """ Base log parsing class to hold common parsing functions. """

    def __init__(self, callback=None, log_file=None):
        """ Setup the log parser. """
        Process.__init__(self)
        self.callback = callback
        self.logs = self.generate_logs(log_file)
        # self.logger = logging.getLogger('debug')
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signal, frame):
        print 'You pressed Ctrl+C!'
        # self.terminate()

    def validate_log_line(self, line):
        """
        Returns true if a log line is valid, false otherwise.
        For now, just checks to make sure the timestamp is valid
        # represents a comment in the logs
        """
        if line.startswith('#'):
            return False
        date_obj = dt.datetime.strptime(
                   line.split('; ')[0],
                   '%Y-%m-%d %H:%M:%S.%f'
        )
        return True if date_obj else False

    def generate_logs(self, log_file):
        """
        Takes logs from an input file param and converts logs into
        a list of dictionaries where each dict represents a line from
        the log file. Assumes that logs from the log_file are saved
        in json format.
        """
        current_logs = []
        for line in open(log_file, 'r'):
            # only look at valid lines
            if self.validate_log_line(line):
                line_split = line.split('; ')
                date_str = line_split[0]
                data = json.loads(line_split[1])
                date_obj = dt.datetime.strptime(
                    date_str,
                    '%Y-%m-%d %H:%M:%S.%f'
                )
                data_dict = {
                    "time": date_obj,       # entire datetime obj
                    "data": data            # data from original msg
                }
                current_logs.append(data_dict)
        return current_logs

    def run(self):
        """ Emit the log data in real-time. """

        # iterate throuhg a list of dictionaries
        for i, d in enumerate(self.logs):
            self.callback(d['data'])
            if i < len(self.logs) - 1:
                time.sleep((self.logs[i + 1]['time'] - self.logs[i]['time']).total_seconds())
