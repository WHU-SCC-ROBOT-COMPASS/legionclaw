import argparse
import time
import sys

import fastdds_obstacle_list
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
        obstacle_list_output_pub = fastdds_obstacle_list.ObstacleListDataWriter(0, "rt/perception/camera/camera_objects_fusion/COFObstacleList")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/perception/camera/camera_objects_fusion/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    obstacle_list_output = fastdds_obstacle_list.ObstacleList()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            obstacle_list_output_pub.write_sample(obstacle_list_output)
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
        c2odobstacle_list_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/camera/camera_2d_object_detect/C2ODObstacleList")
        c3odobstacle_list_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/camera/camera_3d_object_detect/C3ODObstacleList")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    c2odobstacle_list = fastdds_obstacle_list.ObstacleList()
    c3odobstacle_list = fastdds_obstacle_list.ObstacleList()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (c2odobstacle_list_sub.wait_for_sample(2)):
                # Read the received message
                if (c2odobstacle_list_sub.take_sample(c2odobstacle_list)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (c3odobstacle_list_sub.wait_for_sample(2)):
                # Read the received message
                if (c3odobstacle_list_sub.take_sample(c3odobstacle_list)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
