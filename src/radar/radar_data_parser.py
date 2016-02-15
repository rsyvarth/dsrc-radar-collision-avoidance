from threading import Thread
import time
import sys
import logging
import sys

class RadarDataParser(Thread):
    """ Listens for new Radar messages over CAN and parses for the dispatcher.

    This parser reads messages from the CAN Bus using the Kvaser USB Python SKD
    and formats Radar information into a python object. Then we send the data
    along to the event dispatcher.
    """
    def __init__(self, callback = None, log = True):
        """ Initialize the data parser, connect to the can bus. """
        Thread.__init__(self)
        self.callback = callback
        self.log = log
        self.logger = logging.getLogger('radar')

    def run(self):
        """ Start reading data from the CAN Bus and sending full objects to the dispatcher. """
        # NOTE currently this just generates random numbers and sends them to the
        # dispatcher at random intervals
        import canlib
        msgToFunc = {
            1248: "First Message",
            1249: "Second Message",
            1250: "Third Message",
            1251: "Fourth Message",
            1280: "Tracking Message",
            1281: "Tracking Message",
            1282: "Tracking Message",
            1283: "Tracking Message",
            1284: "Tracking Message",
            1285: "Tracking Message",
            1286: "Tracking Message",
            1287: "Tracking Message",
            1288: "Tracking Message",
            1289: "Tracking Message",
            1290: "Tracking Message",
            1291: "Tracking Message",
            1292: "Tracking Message",
            1293: "Tracking Message",
            1294: "Tracking Message",
            1295: "Tracking Message",
            1296: "Tracking Message",
            1297: "Tracking Message",
            1298: "Tracking Message",
            1299: "Tracking Message",
            1300: "Tracking Message",
            1301: "Tracking Message",
            1302: "Tracking Message",
            1303: "Tracking Message",
            1304: "Tracking Message",
            1305: "Tracking Message",
            1306: "Tracking Message",
            1307: "Tracking Message",
            1308: "Tracking Message",
            1309: "Tracking Message",
            1310: "Tracking Message",
            1311: "Tracking Message",
            1312: "Tracking Message",
            1313: "Tracking Message",
            1314: "Tracking Message",
            1315: "Tracking Message",
            1316: "Tracking Message",
            1317: "Tracking Message",
            1318: "Tracking Message",
            1319: "Tracking Message",
            1320: "Tracking Message",
            1321: "Tracking Message",
            1322: "Tracking Message",
            1323: "Tracking Message",
            1324: "Tracking Message",
            1325: "Tracking Message",
            1326: "Tracking Message",
            1327: "Tracking Message",
            1328: "Tracking Message",
            1329: "Tracking Message",
            1330: "Tracking Message",
            1331: "Tracking Message",
            1332: "Tracking Message",
            1333: "Tracking Message",
            1334: "Tracking Message",
            1335: "Tracking Message",
            1336: "Tracking Message",
            1337: "Tracking Message",
            1338: "Tracking Message",
            1339: "Tracking Message",
            1340: "Tracking Message",
            1341: "Tracking Message",
            1342: "Tracking Message",
            1343: "Tracking Message",
            1344: "Weird Message",
            1488: "Validation Message 1",
            1489: "Validation Message 2",
            1508: "Another Message",
            1509: "Another Message 2",
            1510: "Another Message 3",
            1511: "Another Message 4",
            1512: "Another Message 5",
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
                ch1.write(msgId, msg, flg)
            except (canlib.canNoMsg) as ex:
                None
            except (canlib.canError) as ex:
                print(ex)
        '''
        import random
        time.sleep(random.random()*2)
        while True:
        data = [random.randint(0,100)]
        if self.log:
        # goes the the Radar log file
        self.logger.debug(data)
        self.callback(self, data)
        time.sleep(random.random()*2)
        '''

    def status_two(self, msg):
        pass

    def track_msg(self, msgId, msg):
        pass
