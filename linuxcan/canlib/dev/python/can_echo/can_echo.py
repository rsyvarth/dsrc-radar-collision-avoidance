'''
                      Copyright 1996-2013 by KVASER AB            
                  P.O Box 4076 SE-51104 KINNAHULT, SWEDEN
            E-mail: staff@kvaser.se   WWW: http://www.kvaser.se
 
This software is furnished under a license and may be used and copied
only in accordance with the terms of such license.
'''
 
import sys, os
import signal
import threading
import time
import getopt

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import canlib



'''
Realization of the echo server thread.
'''
def can_echo_server(sh, stop_event):
    print 'server thread'
    while (not stop_event.is_set()):   
        try:
            # wait for the message to be received.
            (can_id, can_msg, can_dlc, can_flag, can_time) = sh.read(100)
            print 'server echo received with id: %d msg: %s flg: 0x%X at %d' % (can_id, can_msg, can_flag, can_time)
            # echo the received message.
            sh.write(can_id + 100, can_msg)
        except (canlib.canNoMsg) as ex:
            pass

    print 'server thread will clean up and die.'
    sh.busOff()
    sh.close()

'''
Realization of the echo client thread.
'''
def can_echo_client(ch, stop_event):
    print 'client thread'
    can_id = 0
    while (not stop_event.is_set()):
        try:
            # write echo message to server. 
            ch.write(can_id,[0x55])
            # wait for the message to echo.
            (tmp_id, can_msg, can_dlc, can_flag, can_time) = ch.read(200)
            print 'client echo returned with id: %d msg: %s flg: 0x%X at %d' % (tmp_id, can_msg, can_flag, can_time)
            can_id = can_id +1
            time.sleep(2)
        except (canlib.canNoMsg) as ex:
            print 'no echo message returned'
            pass

    print 'client thread will clean up and die.'
    ch.busOff()
    ch.close()

if __name__ == "__main__":
    cl = canlib.canlib()

    try:
        opts, args = getopt.getopt(sys.argv[1:], "h", ["server_channel=", "client_channel="])
    except getopt.GetoptError:
        print "can_echo.py --server_channel=<ch> --client_channel=<ch>"
        sys.exit(2)
    for opt, arg in opts:
        if opt == "-h":
            print "can_echo.py --server_channel=<ch> --client_channel=<ch>\n"
            print "Connected devices:"
            for ch in range(0, cl.getNumberOfChannels()):
                try:
                    print "%d. %s" % (ch, cl.getChannelData_Name(ch))
                except (canlib.canError) as ex:
                    print ex
            sys.exit(1)
        elif opt in ("--server_channel"):
            server_ch = int(arg)
        elif opt in ("--client_channel"):
            client_ch = int(arg)
    
    if server_ch == None:
        print "can echo server channel must be specified"
        sys.exit(2)
    if client_ch == None:
        print "can echo client channel must be specified"
        sys.exit(2)

    try:
        print "Using device %s as server" % (cl.getChannelData_Name(server_ch))
    except (canlib.canError) as ex:
        print ex
        sys.exit()
        
    try:
        print "Using device %s as client" % (cl.getChannelData_Name(client_ch))
    except (canlib.canError) as ex:
        print ex
        sys.exit()
       
    try:
        sh = cl.openChannel(server_ch, canlib.canOPEN_EXCLUSIVE)
        sh.setBusParams(canlib.canBITRATE_1M)
        sh.setBusOutputControl(canlib.canDRIVER_NORMAL)
        sh.busOn()
    except (canlib.canError) as ex:
        print ex
        sys.exit()   
    
    try:
        ch = cl.openChannel(client_ch, canlib.canOPEN_EXCLUSIVE)
        ch.setBusParams(canlib.canBITRATE_1M)
        ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
        ch.busOn()
        
    except (canlib.canError) as ex:
        print ex
        sys.exit()   
    
    # Create and start the echo server thread
    t1_stop = threading.Event()
    t1 = threading.Thread(target=can_echo_server, args=(sh,t1_stop))
    t1.start()
    
    # Create and start the echo client thread
    t2_stop = threading.Event()
    t2 = threading.Thread(target=can_echo_client, args=(ch,t2_stop))
    t2.start()
    
    while(True):
        try:
            time.sleep(1)
        except (KeyboardInterrupt, SystemExit):
            t1_stop.set()
            t2_stop.set()
            t1.join()
            t2.join()
            print "Threads are dead, exiting..."
            break
