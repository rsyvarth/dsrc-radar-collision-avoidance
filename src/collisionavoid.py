"""Description on one line

More details and usage instructions
"""
import time
import canlib
import logging
import logging.config
import argparse

from combiner.combiner_base import Combiner
from collision import collision_avoid

def main():
    args = parse_args()

    logging.config.fileConfig('logger.ini')
    print_header()
    # test_logger()

    # Setup the Combiner to call collision_avoid.new_data_handler every time new data is available!
    combiner = Combiner(collision_avoid.new_data_handler, args.log_dsrc, args.log_radar, args.dsrc_log_file, args.radar_log_file)
    combiner.start()

    # Keep the program running until ^C, sleep so we don't use too much CPU
    while True:
        time.sleep(100)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-log-dsrc', dest='log_dsrc', action='store_false', help="Disable radar logging (only affects live data)")
    parser.add_argument('--no-log-radar', dest='log_radar', action='store_false', help="Disable radar logging of (only affect live data)")

    parser.add_argument('--load-dsrc-log', dest='dsrc_log_file', help="Path to the dsrc log file to use for emulation (If not passed we use live data)")
    parser.add_argument('--load-radar-log', dest='radar_log_file', help="Path to the radar log file to use for emulation (If not passed we use live data)")
    parser.set_defaults(log_radar=True, log_dsrc=True)

    return parser.parse_args()

def print_header():
    cl = canlib.canlib();
    channels = cl.getNumberOfChannels()
    logging.info("/-------------------------------------------------\\")
    logging.info("| Booting DSRC+Radar Collision Avoidance")
    logging.info("| Canlib running version %s with %s channels" % (cl.getVersion(), channels))
    logging.info("\-------------------------------------------------/\n")

def test_logger():
    cl = canlib.canlib();
    logging.debug("canlib version: %s" % cl.getVersion())
    logging.info("you should see the version number in logs/canlib_all.log!")
    logging.debug("this should not print to command line")
    logging.info("this should print to command line")
    logging.warning("this is a test warning")
    logging.error("this is a test error")

if __name__ == "__main__":
    main()
