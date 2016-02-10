"""DSRC+Radar Collision Avoidance

This is the main application interface for the DSRC+Radar collision avoidance
application. The purpose of this application is to provide a demonstration as
to how the combined use of DSRC and Radar data can create more accurate model
of a vehicle's environment and ultimately lead to improved safety through
advanced collision avoidance software.

Usage:
    There are many external software and hardware requirements in order to
    correctly use this software in its current form. For details on those
    requirements please go to the README.md file in root of this repository.

    For details on how to use this application in particular please try running
    `python shell.py -h` to display the help text.

Software Overview:
    This application is split into multiple parts in order to facilitate
    component swapping and reuse. An overview of each component is provided
    below.

    Shell - Simple setup script for our application. It manages initializing our
    packages and parsing of some command line arguments.

    Collision - This package is included mainly for the purpose of demonstration.
    It takes the information provided by our Combiner and calculates any predicted
    collisions displaying warnings in a simple UI.

    Combiner - The Combiner package is where most of our core technology resides.
    This package is responsible for taking data from each source, normalizing the
    format, and combining that information into a single model of the car's
    environment. Once the combination is complete the new model gets passed to
    the registered callback function, which in our case feeds data into our collision
    avoidance system.

    DSRC - The DSRC package is split up into 3 main components. There are 2 parsers:
    the data parser which handles polling the DSRC REST API for new information
    and the log parser which reads previously logged events and replays them in
    real time. The log parser is useful in application testing since it lets us
    recreate a sequence of events in our code that we previously recorded using
    the data parser in a real-world test. Sitting in front of both of these parsers
    is the dispatcher which instantiates the proper parser for the current environment
    and acts as a standard interface to the combiner, passing messages to the
    combiner as they are recieved.

    Radar - The structure in this package is quite similar to that in the DSRC
    package. Please read that section for details on the general architecture.
    Note that the Radar utilizes the Kvaser Canlib Python SDK in order to retrieve
    information from the CAN Bus about the radar.

    Logs - This directory stores logs which are produced during the application
    runtime. We also have archived several logs which include raw DSRC & Radar
    information from different real-world experiments for the purpose of testing
    our software's performance.

    Util - This is simply a package which holds all of the utility classes which
    we use throughout the codebase.

Authors:
    Bryce Arden
    Phillip Lemons
    Ryan Meek
    Jon Reynolds
    Kevin Rosen
    Robert Syvarth
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
