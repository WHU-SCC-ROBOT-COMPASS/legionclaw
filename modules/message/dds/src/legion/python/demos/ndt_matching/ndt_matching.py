import argparse
import time
import sys

import fastdds_odometry
import fastdds_location
import fastdds_point_cloud
import fastdds_faults
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
        odometry_output_pub = fastdds_odometry.OdometryDataWriter(0, "rt/localization/ndt_matching/NMOdometry")
        location_pub = fastdds_location.LocationDataWriter(0, "rt/localization/ndt_matching/Location")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/localization/ndt_matching/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    odometry_output = fastdds_odometry.Odometry()
    location = fastdds_location.Location()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            odometry_output_pub.write_sample(odometry_output)
            location_pub.write_sample(location)
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
        odometry_input_sub = fastdds_odometry.OdometryDataReader(0, "rt/localization/ndt_odom_predictor/NOPOdometry")
        point_cloud_sub = fastdds_point_cloud.PointCloudDataReader(0, "rt/drivers/lidar/PointCloud")
        ins_sub = fastdds_ins.InsDataReader(0, "rt/drivers/ins/Ins")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    odometry_input = fastdds_odometry.Odometry()
    point_cloud = fastdds_point_cloud.PointCloud()
    ins = fastdds_ins.Ins()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (odometry_input_sub.wait_for_sample(2)):
                # Read the received message
                if (odometry_input_sub.take_sample(odometry_input)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (point_cloud_sub.wait_for_sample(2)):
                # Read the received message
                if (point_cloud_sub.take_sample(point_cloud)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (ins_sub.wait_for_sample(2)):
                # Read the received message
                if (ins_sub.take_sample(ins)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
