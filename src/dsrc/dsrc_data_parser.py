from threading import Thread
import time, logging, json
import socket
from xml.etree import ElementTree

class DsrcDataParser(Thread):
    """ Poll DSRC API for new data, when new data is found pass along.

    This parser handles making calls to the DSRC to get updated information about
    the vehicle state. If the information retrieved is different from the previously
    obtained information we send the data along to the event dispatcher.
    """
    def __init__(self, callback=None, log=True):
        """ Initialize the data parser. """
        Thread.__init__(self)
        self.callback = callback
        self.log = log
        self.logger = logging.getLogger('dsrc')

        # Setup the UDP socket that we are listening on
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("10.1.1.2", 5005))
        
    def run(self):
        """ Start reading data from the DSRC API. """
       
        while True:
            
            data, addr = self.sock.recvfrom(1024)
            
            # print "recieved message:", data
            first_line, xml = data.split('\n', 1)
            
            root = ElementTree.fromstring(xml)
            message_bytes = root[1].text.split() # Split the bytes on whitespace

            raw_message = {
                "message_id": int(message_bytes[0], 16), # 1 byte message id
                "tmp_id": int(''.join(message_bytes[1:5]), 16), # 4 byte tmp id
                "current_second": int(''.join(message_bytes[5:7]), 16), # 2 byte current seconds

                "lat": int(''.join(message_bytes[7:11]), 16), # 4 byte lat
                "long": int(''.join(message_bytes[11:15]), 16), # 4 byte long
                "elevation": int(''.join(message_bytes[15:17]), 16), # 2 byte elevation 
                "accuracy": int(''.join(message_bytes[17:21]), 16), # 4 byte accuracy
                
                "speed": int(''.join(message_bytes[21:23]), 16), # 2 byte speed
                "heading": int(''.join(message_bytes[23:25]), 16), # 2 byte heading
                "wheel_angle": int(''.join(message_bytes[25:26]), 16), # 1 byte wheel angle

                "accel_long": int(''.join(message_bytes[26:28]), 16), # 2 byte accel long
                "accel_lat_set": int(''.join(message_bytes[28:30]), 16), # 2 byte lateral accel
                "accel_vert": int(''.join(message_bytes[30:31]), 16), # 1 byte accel vert
                "accel_yaw": int(''.join(message_bytes[31:33]), 16), # 2 byte accel yaw 

                "brakes": int(''.join(message_bytes[33:35]), 16), # 2 byte brake status

                "vehicle_size": int(''.join(message_bytes[35:38]), 16) # 3 byte vehicle size 
            }
			
			message = {
				"message_id": raw_message["message_id"], # to number 
				"tmp_id": raw_message["tmp_id"], # unknown
				"current_second": raw_message["current_second"], # to s
				
				"lat": float(raw_message["lat"]) / 1e7, # to minutes.seconds
				"long": float(raw_message["long"]) / 1e7, # to minutes.seconds
				"elevation": float(raw_message["elevation"]) / 1e1, # to m
				"accuracy": raw_message["accuracy"], # unknown 
				
				"speed": float(raw_message["speed"]) / 75e3, # to m/s
				"heading": float(raw_message["heading"]) / 8e1, # unknown
				"wheel_angle": float(raw_message["wheel_angle"]) * 0.3,
				
				"accel_long": float(raw_message["message_id"]) / 14e3, # to m/s^2
				"accel_lat_set": float(raw_message["accel_lat_set"]) / 3e3, # to m/s^2
				"accel_vert": float(raw_message["accel_vert"]) / 1e3, # to G's
				"accel_yaw": float(raw_message["accel_yaw"]) / 175e2, # to degrees/s
				
				"brakes": {
					"aux_brakes": raw_message["brakes"] & 0x03,
					"brake_boost": (raw_message["brakes"] >> 2) & 0x03,
					"scs": (raw_message["brakes"] >> 4) & 0x03,
					"abs": (raw_message["brakes"] >> 6) & 0x03,
					"traction": (raw_message["brakes"] >> 8) & 0x03,
					"spare_bit": (raw_message["brakes"] >> 10) & 0x01,
					"wheel_brakes_unavailable": (raw_message["brakes"] >> 11) & 0x01,
					"wheel_brakes": (raw_message["brakes"] >> 12) & 0x0F
				},

				"vehicle_size": {
					"length": raw_message["vehicle_size"] & 0x3FFF, # to cm
					"width": (raw_message["vehicle_size"] >> 14) & 0xFF # to cm
				}
			}
            
            # If the first line contains Rx then this is a message from a remote DSRC
            # unit, if first_line contains Tx it was a message being sent to a remote DSRC
            is_remote_message = first_line.find('Rx') > 0


            data = {
                "is_remote_message": is_remote_message,
                #"message_bytes": message_bytes,
                "message": message
            }
            
            if self.log:
                # sends JSON data to dsrc log file
                self.logger.debug(str(
                    json.dumps(data,
                    separators=(',',':')))
                )

            self.callback(self, [data])
