import canlib
import logging
import logging.config
import sys

debug = True;

if __name__ == '__main__':
    if debug is True:
        logging.config.fileConfig('logger.ini')
        logger = logging.getLogger();

    cl = canlib.canlib();
    channels = cl.getNumberOfChannels()

    dbg_str = "canlib version: %s" % cl.getVersion()
    logging.debug(dbg_str)
    logging.info("you should see the version number in logs/canlib_all.log!")
    logging.debug("this should not print to command line")
    logging.info("this should print to command line")
    logging.warning("this is a test warning")
    logging.error("this is a test error")
