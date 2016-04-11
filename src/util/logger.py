from multiprocessing import Process
import time, json, logging
import datetime as dt
import signal, os


class LogParser(Process):
    """ Base log parsing class to hold common parsing functions. """

    def __init__(self, callback=None, log_file=None, log_config=None):
        """ Setup the log parser. """
        Process.__init__(self)
        self.callback = callback
        self.log_config = self.parse_config(log_config)
        self.log_file = log_file
        #Currently log parsing adds around 0.01 sec of overhead so we need to adjust our sleeps for this
        #self.log_overhead_adjustment = 0.05

        # self.logger = logging.getLogger('debug')
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signal, frame):
        print 'You pressed Ctrl+C!'
        self.terminate()

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

        if not date_obj:
            return False

        if 'video_start' in self.log_config and date_obj < self.log_config['video_start']:
            return False

        return True

    def parse_config(self, config_file):
        if not config_file:
            return {}

        with open(config_file, 'r') as content_file:
            content = content_file.read()
            data = json.loads(content)

            if 'video_start' in data:
                data['video_start'] = dt.datetime.strptime(data['video_start'], '%Y-%m-%d %H:%M:%S.%f')

            print data
            return data

    def generate_logs(self, log_file):
        """
        Takes logs from an input file param and converts logs into
        a list of dictionaries where each dict represents a line from
        the log file. Assumes that logs from the log_file are saved
        in json format.
        """
        # current_logs = []
        for line in open(log_file, 'r'):
            # only look at valid lines
            if not self.validate_log_line(line):
                continue;

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
            yield data_dict
        #     current_logs.append(data_dict)
        # return current_logs

    def run(self):
        """ Emit the log data in real-time. """

        first_run = True
        # iterate throuhg a list of dictionaries
        for log in self.generate_logs(self.log_file):
            if first_run:
                start_real = dt.datetime.now()
                start_log = log['time']
                diff = (start_real - start_log).total_seconds()
                print 'Diff', diff
                first_run = False

            while True:
                sec_diff = (dt.datetime.now() - log['time']).total_seconds()

                if sec_diff >= diff:
                    self.callback(log['data'])
                    break
                else:
                    time.sleep(0.005)
