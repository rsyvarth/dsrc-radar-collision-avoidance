from multiprocessing import Process
import time, logging, json
from util.logger_conf import configure_logs
import socket
from xml.etree import ElementTree

class DsrcDataParser(Process):
    """ Poll DSRC API for new data, when new data is found pass along.

    This parser handles making calls to the DSRC to get updated information about
    the vehicle state. If the information retrieved is different from the previously
    obtained information we send the data along to the event dispatcher.
    """
    def __init__(self, callback=None, log=True, log_level="DEBUG"):
        """ Initialize the data parser. """
        Process.__init__(self)
        self.callback = callback
        self.log = log
        self.log_level = log_level

    def hex_to_int(self, h, d):
        i = int(h, 16)
        if i >= 2**(d-1):
            i -= 2**d
        return i

    def hex_to_uint(self, h):
        return int(h, 16)

    def run(self):
        """ Start reading data from the DSRC API. """
        # Setup the UDP socket that we are listening on
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5005))

        configure_logs(getattr(logging, self.log_level, None))
        self.logger = logging.getLogger('debug_dsrc')
        # These are working on Bryce's Machine
        # self.logger.info( "welcome to the debug_dsrc logger!")
        logging.getLogger('debug_dsrc').info("This is a test message for dsrc!")

        remote_messages = []

        while True:

            data, addr = self.sock.recvfrom(1024)

            # print "recieved message:", data
            first_line, xml = data.split('\n', 1)

            root = ElementTree.fromstring(xml)
            message_bytes = root[1].text.split() # Split the bytes on whitespace

            raw_message = {
                "message_id": self.hex_to_uint(message_bytes[0]), # 1 byte message id
                "tmp_id": self.hex_to_uint(''.join(message_bytes[1:5])), # 4 byte tmp id
                "current_second": self.hex_to_uint(''.join(message_bytes[5:7])), # 2 byte current seconds

                "lat": self.hex_to_int(''.join(message_bytes[7:11]), 32), # 4 byte lat
                "long": self.hex_to_int(''.join(message_bytes[11:15]), 32), # 4 byte long
                "elevation": self.hex_to_int(''.join(message_bytes[15:17]), 16), # 2 byte elevation
                "accuracy": self.hex_to_uint(''.join(message_bytes[17:21])), # 4 byte accuracy

                "speed": self.hex_to_uint(''.join(message_bytes[21:23])), # 2 byte speed
                "heading": self.hex_to_uint(''.join(message_bytes[23:25])), # 2 byte heading
                "wheel_angle": self.hex_to_int(''.join(message_bytes[25:26]), 8), # 1 byte wheel angle

                "accel_long": self.hex_to_int(''.join(message_bytes[26:28]), 16), # 2 byte accel long
                "accel_lat_set": self.hex_to_int(''.join(message_bytes[28:30]), 16), # 2 byte lateral accel
                "accel_vert": self.hex_to_int(''.join(message_bytes[30:31]), 8), # 1 byte accel vert
                "accel_yaw": self.hex_to_int(''.join(message_bytes[31:33]), 16), # 2 byte accel yaw

                "brakes": self.hex_to_uint(''.join(message_bytes[33:35])), # 2 byte brake status

                "vehicle_size": self.hex_to_uint(''.join(message_bytes[35:38])) # 3 byte vehicle size
            }

            message = {
                "message_id": raw_message["message_id"], # to number
                "tmp_id": raw_message["tmp_id"], # unknown
                "current_second": raw_message["current_second"], # to s

                "lat": float(raw_message["lat"]) / 1e7, # to minutes.seconds
                "long": float(raw_message["long"]) / 1e7, # to minutes.seconds
                "elevation": float(raw_message["elevation"]) / 1e1, # to m
                "accuracy": raw_message["accuracy"], # unknown

                "speed": raw_message["speed"] & 0x1FFF, # to km/h
                # "heading": float(raw_message["heading"]) / 8e1, # unknown
                # "wheel_angle": float(raw_message["wheel_angle"]) * 0.3,

                # "accel_long": float(raw_message["message_id"]) / 14e3, # to m/s^2
                # "accel_lat_set": float(raw_message["accel_lat_set"]) / 3e3, # to m/s^2
                # "accel_vert": float(raw_message["accel_vert"]) / 1e3, # to G's
                # "accel_yaw": float(raw_message["accel_yaw"]) / 175e2, # to degrees/s
                #
                # "brakes": {
                #     "aux_brakes": raw_message["brakes"] & 0x03,
                #     "brake_boost": (raw_message["brakes"] >> 2) & 0x03,
                #     "scs": (raw_message["brakes"] >> 4) & 0x03,
                #     "abs": (raw_message["brakes"] >> 6) & 0x03,
                #     "traction": (raw_message["brakes"] >> 8) & 0x03,
                #     "spare_bit": (raw_message["brakes"] >> 10) & 0x01,
                #     "wheel_brakes_unavailable": (raw_message["brakes"] >> 11) & 0x01,
                #     "wheel_brakes": (raw_message["brakes"] >> 12) & 0x0F
                # },
                #
                # "vehicle_size": {
                #     "length": raw_message["vehicle_size"] & 0x3FFF, # to cm
                #     "width": (raw_message["vehicle_size"] >> 14) & 0xFF # to cm
                # }
            }

            # If the first line contains Rx then this is a message from a remote DSRC
            # unit, if first_line contains Tx it was a message being sent to a remote DSRC
            if first_line.find('Rx') > 0:
                remote_messages.append(message)
            else:
                # compile all of the remote messages since the last local DSRC update plus
                # the most recent DSRC update
                data = {
                    "message": message,
                    "remote_messages": remote_messages # state of nearby vehicles
                }
                # Send the messages to the dispatcher
                self.callback(data)

                if self.log:
                    # sends JSON data to dsrc log file
                    logging.getLogger('dsrc').info(json.dumps(data))

                # once remote_messages have been saved with most recent local DSRC update,
                # clear the list and start anew
                remote_messages = []
