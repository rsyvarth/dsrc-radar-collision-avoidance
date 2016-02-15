from threading import Thread
import time, sys, logging, json, canlib, random


class RadarDataParser(Thread):
    """ Listens for new Radar messages over CAN and parses for the dispatcher.

    This parser reads messages from the CAN Bus using the Kvaser USB Python SKD
    and formats Radar information into a python object. Then we send the data
    along to the event dispatcher.
    """
    def __init__(self, callback=None, log=True):
        """ Initialize the data parser, connect to the can bus. """
        Thread.__init__(self)
        self.callback = callback
        self.log = log
        self.logger = logging.getLogger('radar')
        self.data = {}

    def run(self):
        """ Start reading data from the CAN Bus and sending full objects to the dispatcher. """
        # NOTE currently this just generates random numbers and sends them to the
        # dispatcher at random intervals
        msgToFunc = {
            1248: self.status_one,
            1249: self.status_two,
            1250: self.status_three,
            1251: self.status_four,
            1280: self.track_msg,
            1281: self.track_msg,
            1282: self.track_msg,
            1283: self.track_msg,
            1284: self.track_msg,
            1285: self.track_msg,
            1286: self.track_msg,
            1287: self.track_msg,
            1288: self.track_msg,
            1289: self.track_msg,
            1290: self.track_msg,
            1291: self.track_msg,
            1292: self.track_msg,
            1293: self.track_msg,
            1294: self.track_msg,
            1295: self.track_msg,
            1296: self.track_msg,
            1297: self.track_msg,
            1298: self.track_msg,
            1299: self.track_msg,
            1300: self.track_msg,
            1301: self.track_msg,
            1302: self.track_msg,
            1303: self.track_msg,
            1304: self.track_msg,
            1305: self.track_msg,
            1306: self.track_msg,
            1307: self.track_msg,
            1308: self.track_msg,
            1309: self.track_msg,
            1310: self.track_msg,
            1311: self.track_msg,
            1312: self.track_msg,
            1313: self.track_msg,
            1314: self.track_msg,
            1315: self.track_msg,
            1316: self.track_msg,
            1317: self.track_msg,
            1318: self.track_msg,
            1319: self.track_msg,
            1320: self.track_msg,
            1321: self.track_msg,
            1322: self.track_msg,
            1323: self.track_msg,
            1324: self.track_msg,
            1325: self.track_msg,
            1326: self.track_msg,
            1327: self.track_msg,
            1328: self.track_msg,
            1329: self.track_msg,
            1330: self.track_msg,
            1331: self.track_msg,
            1332: self.track_msg,
            1333: self.track_msg,
            1334: self.track_msg,
            1335: self.track_msg,
            1336: self.track_msg,
            1337: self.track_msg,
            1338: self.track_msg,
            1339: self.track_msg,
            1340: self.track_msg,
            1341: self.track_msg,
            1342: self.track_msg,
            1343: self.track_msg,
            1344: self.we_dont_know_msg,
            1488: self.validation_msg_one,
            1489: self.validation_msg_two,
            1508: self.additional_status_one,
            1509: self.additional_status_two,
            1510: self.additional_status_three,
            1511: self.additional_status_four,
            1512: self.additional_status_five,
        }
        cl = canlib.canlib()
        channels = cl.getNumberOfChannels()
        ch = 0; #Hard-coded, might need to change!
        if ch >= channels:
            print("Invalid channel number")
            sys.exit()

        try:
            ch1 = cl.openChannel(ch, canlib.canOPEN_ACCEPT_VIRTUAL)
            print("Using channel: %s, EAN: %s" % (ch1.getChannelData_Name(),
            ch1.getChannelData_EAN()))

            ch1.setBusOutputControl(canlib.canDRIVER_NORMAL)
            ch1.setBusParams(canlib.canBITRATE_500K)
            ch1.busOn()
        except (canlib.canError) as ex:
            print(ex)
        message = [0,0,0,0,0,0,191,0]
        ch1.write(1265,message,8)
        while True:
            try:
                msgId, msg, dlc, flg, time = ch1.read()
                print("%9d  %9d  0x%02x  %d  %s" % (msgId, time, flg, dlc, msg))
                print(msg, ''.join('{:02x}'.format(x) for x in msg))
                if msgId in msgToFunc:
                    #This message is valid, so we need to parse it
                    if msgId >= 1280 and msgId <= 1343:
                        msgToFunc[msgId](msgId, msg)
                    else:
                        print(msgId)
                        msgToFunc[msgId](msg)
            except (canlib.canNoMsg) as ex:
                None
            except (canlib.canError) as ex:
                print(ex)

        # time.sleep(random.random()*2)
        # while True:
        # data = [random.randint(0,100)]
        # if self.log:
        # # goes the the Radar log file
        # self.logger.debug(data)
        # self.callback(self, data)
        # time.sleep(random.random()*2)
        #     data = [random.randint(0,100)]
        #     if self.log:
        #         # sends JSON data to radar log file
        #         self.logger.debug(str(
        #             json.dumps(data,
        #             separators=(',',':')))
        #         )

    #message ID x4E1 or 1249
    def status_two(self, msg):
#        Byte 0 bits 0-1 - Rolling Count
#        Byte 0 bits 2-7 - Maximum tracks (number of objects of interest)
#        Byte 1 bit 7 - Overheat Error
#        Byte 1 bit 6 - Range Perf Error
#        Byte 1 bit 5 - Internal Error
#        Byte 1 bit 4 - XCVR Operational
#        0 - Not radiating, 1 - radiating
#        Byte 1 bit 3 - Raw Data Mode
#        Byte 1 bits 0-2, Byte 2 - Steering Angle Ack
#        Byte 3 - Temperature
#        Byte 4 bits 2-7 - Speed Comp Factor
#        Byte 4 bits 0-1 - Grouping Mode
#        Byte 5 - Yaw Rate Bias
#        Bytes 6-7 - SW Version DSP
        self.data["rolling_count"] = msg[0] & 0x03
        self.data["radiating"] = (msg[1] & 0x10) >> 4
        self.data["steering_angle"] = msg[1] & 0x07
        self.data["yaw_rate_bias"] = msg[5]
        pass

    #message ID 500-53F or 1280-1343
    def track_msg(self, msgId, msg):
        #Going to start with track width
        track_width = msg[4] & 0x3C
        track_width = track_width >> 2
        print(track_width)
        #Next we need to get the range
        #NOTE: This spans 2 bytes, so not sure how this works

    def place_holder_msg(self, msg):
        pass

    def we_dont_know_msg(self, msg):
        pass

    def validation_msg_one(self, msg):
        pass

    def validation_msg_two(self, msg):
        pass

    def additional_status_one(self, msg):
        pass

    def additional_status_two(self, msg):
        pass

    def additional_status_three(self, msg):
        pass

    def additional_status_four(self, msg):
        pass

    def additional_status_five(self, msg):
        pass

    def status_one(self, msg):
        pass

    def status_two(self, msg):
        pass

    def status_three(self, msg):
        pass

    def status_four(self, msg):
        pass
