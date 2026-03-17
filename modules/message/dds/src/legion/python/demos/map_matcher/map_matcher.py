import argparse
import time
import sys

import fastdds_lane_list
import fastdds_obstacle_list
import fastdds_faults
import fastdds_location

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
        location_output_pub = fastdds_location.LocationDataWriter(0, "rt/localization/map_matcher/Location")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/localization/map_matcher/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    location_output = fastdds_location.Location()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            location_output_pub.write_sample(location_output)
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
        obstacle_list_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/fusion/motion_manager/MMObstacleList")
        lane_list_sub = fastdds_lane_list.LaneListDataReader(0, "rt/perception/fusion/motion_manager/MMLaneList")
        location_input_sub = fastdds_location.LocationDataReader(0, "rt/localization/global_fusion/Location")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    obstacle_list = fastdds_obstacle_list.ObstacleList()
    lane_list = fastdds_lane_list.LaneList()
    location_input = fastdds_location.Location()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (obstacle_list_sub.wait_for_sample(2)):
                # Read the received message
                if (obstacle_list_sub.take_sample(obstacle_list)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (lane_list_sub.wait_for_sample(2)):
                # Read the received message
                if (lane_list_sub.take_sample(lane_list)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (location_input_sub.wait_for_sample(2)):
                # Read the received message
                if (location_input_sub.take_sample(location_input)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
