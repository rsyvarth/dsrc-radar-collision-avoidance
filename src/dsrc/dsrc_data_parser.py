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

            message = {
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

                "accel_long": int(''.join(message_bytes[26:29]), 16), # 2 byte accel long
                "accel_lat_set": int(''.join(message_bytes[29:31]), 16), # 2 byte lateral accel
                "accel_vert": int(''.join(message_bytes[31:32]), 16), # 1 byte accel vert
                "accel_yaw": int(''.join(message_bytes[32:34]), 16), # 2 byte accel yaw 

                "breaks": int(''.join(message_bytes[34:36]), 16), # 2 byte break status

                "vehicle_size": int(''.join(message_bytes[36:39]), 16) # 3 byte vehicle size 
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
