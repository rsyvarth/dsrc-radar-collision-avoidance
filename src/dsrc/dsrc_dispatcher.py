
from dsrc.dsrc_data_parser import DsrcDataParser
from dsrc.dsrc_log_parser import DsrcLogParser

class DsrcEventDispatcher(object):
    """ The interface between our data parsers and the combiner.

    The dispatcher instantiates the proper parser for the current environment.
    Once the correct parser has been created we manage the parser thread which
    runs separately from the rest of the application to reduce the overhead of
    I/O on the application's performance. The dispatcher also passes data back
    to the callback function (Combiner) when new data is available.
    """
    def __init__(self, queue, log=True, log_file=None, log_level="DEBUG"):
        """ Initialize dispatcher, instantiate the proper parser for the current environment. """
        self.queue = queue
        if not log_file:
            self.provider = DsrcDataParser(log=log, callback=self.on_message, log_level=log_level)
        else:
            self.provider = DsrcLogParser(log_file=log_file, callback=self.on_message)

    def start(self):
        """ Start running the parser's thread (After this data starts flowing). """
        self.provider.start()


    def on_message(self, data):
        """ Pass messages from the parser to the callback function. """
        # Maybe we will want to do some data parsing here at some point?
        self.queue.put({'from': 'dsrc', 'data': data})

    def is_alive(self):
        return self.provider.is_alive()

    def terminate(self):
        return self.provider.terminate()
