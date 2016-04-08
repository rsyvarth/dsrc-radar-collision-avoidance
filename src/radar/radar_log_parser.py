
from util.logger import LogParser
import logging

class RadarLogParser(LogParser):
    """ Reads the provided log file and emits the logs in real-time

    Extends: util.logger.LogParser
    """
    def __init__(self, callback=None, log_file=None, log_config=None):
        """Init log parser, read log file into memory"""
        LogParser.__init__(self, callback=callback, log_file=log_file, log_config=log_config)

    def run(self):
        """Start emitting log events based on their timestamps"""
        LogParser.run(self)
