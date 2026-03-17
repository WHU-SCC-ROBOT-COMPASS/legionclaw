import argparse
import time
import sys

import fastdds_lane_list
import fastdds_faults

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
        lane_list_output_pub = fastdds_lane_list.LaneListDataWriter(0, "rt/perception/camera/camera_lane_post_process/CLPPLaneList")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/perception/camera/camera_lane_post_process/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    lane_list_output = fastdds_lane_list.LaneList()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            lane_list_output_pub.write_sample(lane_list_output)
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
        lane_list_input_sub = fastdds_lane_list.LaneListDataReader(0, "rt/perception/camera/camera_lane_detect/CLDLaneList")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    lane_list_input = fastdds_lane_list.LaneList()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (lane_list_input_sub.wait_for_sample(2)):
                # Read the received message
                if (lane_list_input_sub.take_sample(lane_list_input)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
