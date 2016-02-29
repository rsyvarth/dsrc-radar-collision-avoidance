import logging
import itertools as it
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

"""
logging framework, useful for debugging
Here are all the possible DEBUG levels and there numerical values
NOTSET(0), DEBUG(10), INFO(20), WARNING(30), ERROR(40), CRITICAL(50)
To use this framework, include the following two lines at the top of any file
import logging

logger = logging.getLogger('dsrc')
logger = logging.getLogger('radar')
logger = logging.getLogger('combined')
logger = logging.getLogger('debug')

then use logger.debug(msg), logger.info(msg), etc...

"""

LOGGING = {
    'version': 1,
    'disable_existing_loggers': False,
    'formatters': {
        'debug': {
            'format': "[%(asctime)s] %(levelname)s [%(name)s: %(filename)s: %(lineno)s] %(message)s", # UNUSED
        },
        'simple': {
            'format': '%(asctime)s %(message)s', # UNUSED
        },
    },
    'handlers': {
        'dsrc_file': {
            'class': 'logging.handlers.RotatingFileHandler',
            'filename': './logs/dsrc.log',
            'mode': 'w',
            'formatter': 'simple',
            'level': 'DEBUG',                   # write all logs to mysite.log
            'backupCount': 5,
            'maxBytes': 10485760,               # 10MB max
        },
        'radar_file': {
            'class': 'logging.handlers.RotatingFileHandler',
            'filename': './logs/radar.log',
            'mode': 'w',
            'formatter': 'simple',
            'level': 'DEBUG',                   # write all logs to mysite.log
            'backupCount': 5,
            'maxBytes': 10485760,               # 10MB max
        },
        'combined_file': {
            'class': 'logging.handlers.RotatingFileHandler',
            'filename': './logs/combined.log',
            'mode': 'w',
            'formatter': 'simple',
            'level': 'DEBUG',                   # write all logs to mysite.log
            'backupCount': 5,
            'maxBytes': 10485760,               # 10MB max
        },
        'debug_file': {
            'class': 'logging.handlers.RotatingFileHandler',
            'filename': './logs/debug.log',
            'mode': 'w',
            'formatter': 'debug',
            'level': 'DEBUG',                   # write all logs to mysite.log
            'backupCount': 5,
            'maxBytes': 10485760,               # 10MB max
        },
        'console': {
            'class': 'logging.StreamHandler',
            'formatter': 'debug',
            'level': 'WARNING',
            'stream': 'ext://sys.stdout',
        },
    },
    'loggers': {
        'dsrc': {
            'handlers': ['dsrc_file', 'console'],
            'level':'DEBUG',                    # change this level to change which levels are written to file
            'mode': 'w',
            'propagate': False,
        },
        'radar': {
            'handlers': ['radar_file', 'console'],
            'level': 'DEBUG',                   # change this level to change which levels are written to file
            'mode': 'w',
            'propagate': False,
        },
        'combined': {
            'handlers': ['combined_file', 'console'],
            'level': 'DEBUG',                   # change this level to change which levels are written to file
            'mode': 'w',
            'propagate': False,
        },
        'debug': {
            'handlers': ['debug_file', 'console'],
            'level': 'DEBUG',                   # change this level to change which levels are written to file
            'mode': 'w',
            'propagate': False,
        },
        'root': {                               # logger that is used when no args are passed into GetLogger()
            'handlers': ['debug_file', 'console'],
            'level': 'DEBUG',                   # change this level to change which levels are written to file
            'propagate': False,
        },
    }
}
def configure_logs():
    """ Configures the system logs wrt to our combined DSRC/radar system """
    logging.config.dictConfig(LOGGING)

    # get every logger
    logger_dbg = logging.getLogger('debug')
    logger_dsrc = logging.getLogger('dsrc')
    logger_radar = logging.getLogger('radar')
    logger_combined = logging.getLogger('combined')

    # get list of handlers for each logger
    handler_dbg = logger_dbg.handlers
    handler_dsrc = logger_dsrc.handlers
    handler_radar = logger_radar.handlers
    handler_combined = logger_combined.handlers

    # instantiate custom formatter
    formatter_simple = USecFormatter(
        fmt='%(asctime)s; %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S.%f')
    formatter_debug = USecFormatter(
        fmt='%(asctime)s; %(levelname)s; %(filename)s; %(lineno)s; %(message)s;',
        datefmt='%Y-%m-%d %H:%M:%S.%f')

    # update handlers with new formatter
    for h_dbg, h_dsrc, h_radar, h_combined in it.izip_longest(logger_dbg.handlers,
                                                              logger_dsrc.handlers,
                                                              logger_radar.handlers,
                                                              logger_combined.handlers):
        if h_dsrc is not None:
            h_dsrc.setFormatter(formatter_simple)
        if h_radar is not None:
            h_radar.setFormatter(formatter_simple)
        if h_combined is not None:
            h_combined.setFormatter(formatter_simple)
        if h_dbg is not None:
            h_dbg.setFormatter(formatter_debug)
