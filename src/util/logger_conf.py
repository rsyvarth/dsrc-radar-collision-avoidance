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
        fmt='%(asctime)s; %(levelname)s; %(filename)s; %(lineno)s; %(message)s',
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
    h_debug_dsrc = logging.handlers.RotatingFileHandler(
        'logs/debug_dsrc.log',
        mode='w',
        # backupCount=5,               # If these are enabled, then logs must append
        # maxBytes=10485760,           # If these are enabled, then logs must append
    )
    h_debug_radar = logging.handlers.RotatingFileHandler(
        'logs/debug_radar.log',
        mode='w',
        # backupCount=5,               # If these are enabled, then logs must append
        # maxBytes=10485760,           # If these are enabled, then logs must append
    )
    h_debug_combined = logging.handlers.RotatingFileHandler(
        'logs/debug_combined.log',
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
    h_radar.setLevel(logging.INFO)
    h_dsrc.setLevel(logging.INFO)
    h_combined.setLevel(logging.INFO)
    h_debug.setLevel(log_level)
    h_console.setLevel(log_level)
    h_debug_dsrc.setLevel(log_level)
    h_debug_radar.setLevel(log_level)
    h_debug_combined.setLevel(log_level)

    # set formatters
    h_radar.setFormatter(formatter_simple)
    h_dsrc.setFormatter(formatter_simple)
    h_combined.setFormatter(formatter_simple)
    h_debug.setFormatter(formatter_debug)
    h_console.setFormatter(formatter_debug)
    h_debug_dsrc.setFormatter(formatter_debug)
    h_debug_radar.setFormatter(formatter_debug)
    h_debug_combined.setFormatter(formatter_debug)

    # get every logger
    l_dsrc = logging.getLogger('dsrc')
    l_radar = logging.getLogger('radar')
    l_combined = logging.getLogger('combined')
    l_debug = logging.getLogger('debug')
    l_debug_dsrc = logging.getLogger('debug_dsrc')
    l_debug_radar = logging.getLogger('debug_radar')
    l_debug_combined = logging.getLogger('debug_combined')

    # logs don't propogate through to console
    l_dsrc.propagate = False
    l_radar.propagate = False
    l_combined.propagate = False
    l_debug.propagate = False
    l_debug_dsrc.propagate = False
    l_debug_radar.propagate = False
    l_debug_combined.propagate = False

    # set logger levels
    l_dsrc.setLevel(logging.DEBUG)
    l_radar.setLevel(logging.DEBUG)
    l_combined.setLevel(logging.DEBUG)
    l_debug.setLevel(log_level)
    l_debug_dsrc.setLevel(log_level)
    l_debug_radar.setLevel(log_level)
    l_debug_combined.setLevel(log_level)

    # add the file specific handlers
    l_dsrc.addHandler(h_dsrc)
    l_radar.addHandler(h_radar)
    l_combined.addHandler(h_combined)
    l_debug.addHandler(h_debug)
    l_debug_dsrc.addHandler(h_debug_dsrc)
    l_debug_radar.addHandler(h_debug_radar)
    l_debug_combined.addHandler(h_debug_combined)

    # add the console handlers
    # l_dsrc.addHandler(h_console)
    # l_radar.addHandler(h_console)
    # l_combined.addHandler(h_console)
    l_debug.addHandler(h_console)
    l_debug_dsrc.addHandler(h_console)
    l_debug_radar.addHandler(h_console)
    l_debug_combined.addHandler(h_console)

