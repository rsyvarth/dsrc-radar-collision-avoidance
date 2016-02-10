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
        'verbose': {
            'format' : "[%(asctime)s] %(levelname)s [%(name)s: %(filename)s: %(lineno)s] %(message)s",
            # 'datefmt' : "%d/%b/%Y %H:%M:%S"
        },
        'simple': {
            'format': '%(levelname)s %(message)s'
        },
    },
    'handlers': {
        'dsrc_file': {
            'class': 'logging.handlers.RotatingFileHandler',
            'backUpCount': 5,
            'maxBytes': 10485760,               # 10MB max
            'filename': 'logs/dsrc.log',
            'formatter': 'verbose',
            'level': 'DEBUG',                   # write all logs to mysite.log
        },
        'radar_file': {
            'class': 'logging.handlers.RotatingFileHandler',
            'backUpCount': 5,
            'maxBytes': 10485760,               # 10MB max
            'filename': 'logs/radar.log',
            'formatter': 'verbose',
            'level': 'DEBUG',                   # write all logs to mysite.log
        },
        'combined_file': {
            'class': 'logging.handlers.RotatingFileHandler',
            'backUpCount': 5,
            'maxBytes': 10485760,               # 10MB max
            'filename': 'logs/combined.log',
            'formatter': 'verbose',
            'level': 'DEBUG',                   # write all logs to mysite.log
        },
        'debug_file': {
            'class': 'logging.handlers.RotatingFileHandler',
            'backUpCount': 5,
            'maxBytes': 10485760,               # 10MB max
            'filename': 'logs/debug.log',
            'formatter': 'verbose',
            'level': 'DEBUG',                   # write all logs to mysite.log
        },
    },
    'loggers': {
        'dsrc': {
            'handlers':['dsrc_file'],
            'level':'DEBUG',                    # change this level to change which levels are written to file
        },
        'radar': {
            'handlers': ['radar_file'],
            'level': 'DEBUG',                   # change this level to change which levels are written to file
        },
        'combined': {
            'handlers': ['combined_file'],
            'level': 'DEBUG',                   # change this level to change which levels are written to file
        },
        'debug': {
            'handlers': ['debug_file'],
            'level': 'DEBUG',                   # change this level to change which levels are written to file
        },
        'root': {                               # logger that is used when no args are passed into GetLogger()
            'handlers': ['debug_file'],
            'level': 'DEBUG',                   # change this level to change which levels are written to file
        },

    }
}
