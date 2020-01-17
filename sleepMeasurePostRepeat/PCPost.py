#!/usr/bin/env python

#This script reads out the serial output from sleepMeasurePostRepeat and sends it to the broker.
#NOTE: enable the QUIET and VERBOSE flags in sleepMeasurePostRepeat
#NOTE: this scripts runs once and then terminates.
#NOTE: SLEEPTIME should be lower than the interval used to read the serial output
#NOTE: add this script to the PC crontab, for example to run it every 20 seconds:
#      * * * * * timeout 20 python3 PCPost.py
#      * * * * * sleep 20; timeout 20 python3 PCPost.py
#      * * * * * sleep 40; timeout 20 python3 PCPost.py

#******************************************
#import stuff
import argparse, serial, sys, datetime

#******************************************
def PCPost(args):

    #------------------------------------------
    #setup serial read
    ser = serial.Serial(
        port = args.port,
        baudrate = args.baudrate,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 60)
    ser.flushInput()

    #------------------------------------------
    #variables
    payload = None
    topic = None
    hostname = None
    user = None
    word = None

    #------------------------------------------
    #loop until all values are available
    while True:
        try:

            #------------------------------------------
            #read data line
            data = ser.readline().decode('utf-8', errors='ignore').strip()

            #------------------------------------------
            #read only
            if args.readonly:
                print(data)

            #------------------------------------------
            #extract and post values
            else:

                #------------------------------------------
                #extract values
                if 'message:' in str(data):
                    payload = str(data).split('message: ')[1].strip()
                    if args.debug: print("payload: %s"%payload)
                if 'topic:' in str(data):
                    topic = str(data).split('topic: ')[1].strip()
                    if args.debug: print("topic: %s"%topic)
                if 'server:' in str(data):
                    hostname = str(data).split('server: ')[1].strip()
                    if args.debug: print("hostname: %s"%hostname)
                if 'user:' in str(data):
                    user = str(data).split('user: ')[1].strip()
                    if args.debug: print("user: %s"%user)
                if 'auth:' in str(data):
                    word = str(data).split('auth: ')[1].strip()
                    if args.debug: print("word: %s"%word)

                #------------------------------------------
                #check that all values are available
                if payload is not None and topic is not None and hostname is not None and user is not None and word is not None :

                    #------------------------------------------
                    #debug
                    if args.debug:
                        print("data:")
                        print("\tpayload: %s"%payload)
                        print("\ttopic: %s"%topic)
                        print("\thostname: %s"%hostname)
                        print("\tuser: %s"%user)
                        print("\tword: %s"%word)

                    #------------------------------------------
                    #post data
                    if not args.quiet:

                        import paho.mqtt.publish as publish
                        print("posting data...")
                        
                        try:
                            out = publish.single(
                                topic = topic,
                                payload = payload,
                                hostname = hostname,
                                port = 1883,
                                auth = {"username":user,"password":word},
                                tls = None,
                                transport = "TCP")
                            if out is not None:
                                print(out)
                            else:
                                print("done")
                        except:
                            print('ERROR posting data ('+datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+")")

                    #------------------------------------------
                    #exit loop
                    break

        #------------------------------------------
        except KeyboardInterrupt:
            print()
            sys.exit(0)

#******************************************
if __name__ == '__main__':

    #------------------------------------------
    #parse input arguments
    parser = argparse.ArgumentParser(description='%prog [options]')
    parser.add_argument('-p', '--port', dest='port', default='/dev/cu.SLAB_USBtoUART', help='serial port, usually "/dev/cu.SLAB_USBtoUART" or "/dev/ttyUSB0"')
    parser.add_argument('-b', '--baudrate', dest='baudrate', default=115200, help='baud rate')
    parser.add_argument('-r', '--readonly', dest='readonly', action='store_true', default=False, help='read only')
    parser.add_argument('-q', '--quiet', dest='quiet', action='store_true', default=False, help='read but do not post')
    parser.add_argument('-d', '--debug', dest='debug', action='store_true', default=False, help='debug mode')
    args = parser.parse_args()
    
    #------------------------------------------
    #PCPost
    PCPost(args)
    print
