import argparse
import time
import sys

import fastdds_image
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
        c3odobstacle_list1_pub = fastdds_obstacle_list.ObstacleListDataWriter(0, "rt/perception/camera/camera_3d_object_detect/C3ODObstacleList1")
        c3odobstacle_list3_pub = fastdds_obstacle_list.ObstacleListDataWriter(0, "rt/perception/camera/camera_3d_object_detect/C3ODObstacleList3")
        c3odobstacle_list4_pub = fastdds_obstacle_list.ObstacleListDataWriter(0, "rt/perception/camera/camera_3d_object_detect/C3ODObstacleList4")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/perception/camera/camera_3d_object_detect/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    c3odobstacle_list1 = fastdds_obstacle_list.ObstacleList()
    c3odobstacle_list3 = fastdds_obstacle_list.ObstacleList()
    c3odobstacle_list4 = fastdds_obstacle_list.ObstacleList()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            c3odobstacle_list1_pub.write_sample(c3odobstacle_list1)
            c3odobstacle_list3_pub.write_sample(c3odobstacle_list3)
            c3odobstacle_list4_pub.write_sample(c3odobstacle_list4)
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
        image_sub = fastdds_image.ImageDataReader(0, "rt/drivers/camera/Image")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    image = fastdds_image.Image()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (image_sub.wait_for_sample(2)):
                # Read the received message
                if (image_sub.take_sample(image)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
