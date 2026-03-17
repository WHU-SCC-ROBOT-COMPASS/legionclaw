import argparse
import time
import sys

import fastdds_imu
import fastdds_obu_cmd_msg
import fastdds_faults
import fastdds_gnss
import fastdds_ins

#Argument parsing
#############################################

parser = argparse.ArgumentParser(
    description='Run a simple FastDDS publisher or subscriber.')
parser.add_argument('endpoint', action='store',
                    choices=['publisher', 'subscriber'],
                    help='the kind of endpoint to create')
parser.add_argument('--domain', action='store', default=0, type=int,
                    help='domain identifier')
parser.add_argument('--topic', action='store', default='topic',
                    help='topic name')

args = parser.parse_args()


# Loop for the publisher
if (args.endpoint == 'publisher'):

    # Create a publisher on the topic
    try:
        imu_pub = fastdds_imu.ImuDataWriter(0, "rt/drivers/ins/Imu")
        ins_pub = fastdds_ins.InsDataWriter(0, "rt/drivers/ins/Ins")
        gnss_pub = fastdds_gnss.GnssDataWriter(0, "rt/drivers/ins/Gnss")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/drivers/ins/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    imu = fastdds_imu.Imu()
    ins = fastdds_ins.Ins()
    gnss = fastdds_gnss.Gnss()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            imu_pub.write_sample(imu)
            ins_pub.write_sample(ins)
            gnss_pub.write_sample(gnss)
            faults_pub.write_sample(faults)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        obu_cmd_msg_sub = fastdds_obu_cmd_msg.ObuCmdMsgDataReader(0, "rt/vui_client/ObuCmdMsg")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (obu_cmd_msg_sub.wait_for_sample(2)):
                # Read the received message
                if (obu_cmd_msg_sub.take_sample(obu_cmd_msg)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
