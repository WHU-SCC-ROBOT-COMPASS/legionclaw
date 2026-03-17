import argparse
import time
import sys

import fastdds_point_cloud
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
        point_cloud_output_pub = fastdds_point_cloud.PointCloudDataWriter(0, "rt/perception/lidar/multi_lidar_concate/PointCloud")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/perception/lidar/multi_lidar_concate/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    point_cloud_output = fastdds_point_cloud.PointCloud()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            point_cloud_output_pub.write_sample(point_cloud_output)
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
        point_cloud_input_sub = fastdds_point_cloud.PointCloudDataReader(0, "rt/drivers/lidar/PointCloud")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    point_cloud_input = fastdds_point_cloud.PointCloud()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (point_cloud_input_sub.wait_for_sample(2)):
                # Read the received message
                if (point_cloud_input_sub.take_sample(point_cloud_input)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
