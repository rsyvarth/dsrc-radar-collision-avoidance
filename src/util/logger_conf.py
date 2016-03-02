"""
logging framework configuration

Here are all the possible DEBUG levels and their numerical values NOTSET(0),
DEBUG(10), INFO(20), WARNING(30), ERROR(40), CRITICAL(50).  To use this
framework, follow the following syntax:
import logging

logger = logging.getLogger('dsrc')
logger = logging.getLogger('radar')
logger = logging.getLogger('combined')
logger = logging.getLogger('debug')

then use logger.debug(msg), logger.info(msg), etc...

"""

import os, sys, logging
import logging.config
import datetime as dt


class USecFormatter(logging.Formatter):
    """
    Custom Formatter for using microsecond timestamps
    Also replaces , used by ascii for msecs with a .
    """
    converter=dt.datetime.fromtimestamp
    def formatTime(self, record, datefmt=None):
        ct = self.converter(record.created)
        if datefmt:
            s = ct.strftime(datefmt)
        else:
            t = ct.strftime("%Y-%m-%d %H:%M:%S")
            s = "%s.%03d" % (t, record.msecs)
        return s


def configure_logs(log_level):
    """ Configures the system logs wrt to our combined DSRC/radar system """
    if not isinstance(log_level, int):
        raise ValueError('Invalid log level %s' % loglevel)

    formatter_simple = USecFormatter(
        fmt='%(asctime)s; %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S.%f'
    )
    formatter_debug = USecFormatter(
        fmt='%(asctime)s; %(levelname)s; %(filename)s; %(lineno)s; %(message)s;',
        datefmt='%Y-%m-%d %H:%M:%S.%f'
    )
    h_radar = logging.handlers.RotatingFileHandler(
        'logs/radar.log',
        mode='w',
        # backupCount=5,               # If these are enabled, then logs must append
        # maxBytes=10485760,           # If these are enabled, then logs must append
    )
    h_dsrc = logging.handlers.RotatingFileHandler(
        'logs/dsrc.log',
        mode='w',
        # backupCount=5,               # If these are enabled, then logs must append
        # maxBytes=10485760,           # If these are enabled, then logs must append
    )
    h_combined = logging.handlers.RotatingFileHandler(
        'logs/combined.log',
        mode='w',
        # backupCount=5,               # If these are enabled, then logs must append
        # maxBytes=10485760,           # If these are enabled, then logs must append
    )
    h_debug = logging.handlers.RotatingFileHandler(
        'logs/debug.log',
        mode='w',
        # backupCount=5,               # If these are enabled, then logs must append
        # maxBytes=10485760,           # If these are enabled, then logs must append
    )
    h_console = logging.StreamHandler(
        stream=sys.stdout
    )

    # set handler levels
    h_radar.setLevel(logging.DEBUG)
    h_dsrc.setLevel(logging.DEBUG)
    h_combined.setLevel(logging.DEBUG)
    h_debug.setLevel(logging.DEBUG)
    h_console.setLevel(log_level)

    # set formatters
    h_radar.setFormatter(formatter_simple)
    h_dsrc.setFormatter(formatter_simple)
    h_combined.setFormatter(formatter_simple)
    h_debug.setFormatter(formatter_debug)
    h_console.setFormatter(formatter_debug)

    # get every logger
    logger_dsrc = logging.getLogger('dsrc')
    logger_radar = logging.getLogger('radar')
    logger_combined = logging.getLogger('combined')
    logger_debug = logging.getLogger('debug')

    # logs don't propogate through to console
    logger_dsrc.propagate = False
    logger_radar.propagate = False
    logger_combined.propagate = False
    logger_debug.propagate = False

    # set logger levels
    logger_dsrc.setLevel(logging.DEBUG)
    logger_radar.setLevel(logging.DEBUG)
    logger_combined.setLevel(logging.DEBUG)
    logger_debug.setLevel(log_level)

    # add the file specific handlers
    logger_dsrc.addHandler(h_dsrc)
    logger_radar.addHandler(h_radar)
    logger_combined.addHandler(h_combined)
    logger_debug.addHandler(h_debug)

    # add the console handlers
    logger_dsrc.addHandler(h_console)
    logger_radar.addHandler(h_console)
    logger_combined.addHandler(h_console)
    logger_debug.addHandler(h_console)

