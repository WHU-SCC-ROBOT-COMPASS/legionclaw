import argparse
import time
import sys

import fastdds_odometry
import fastdds_location
import fastdds_imu
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
        location_pub = fastdds_location.LocationDataWriter(0, "rt/localization/global_fusion/Location")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/localization/global_fusion/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    location = fastdds_location.Location()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
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
        imu_sub = fastdds_imu.ImuDataReader(0, "rt/drivers/ins/Imu")
        ins_sub = fastdds_ins.InsDataReader(0, "rt/drivers/ins/Ins")
        gnss_sub = fastdds_gnss.GnssDataReader(0, "rt/drivers/ins/Gnss")
        vio_odometry_sub = fastdds_odometry.OdometryDataReader(0, "rt/localization/visual_inertial_odometry/VIOOdometry")
        wo_odometry_sub = fastdds_odometry.OdometryDataReader(0, "rt/localization/wheel_odometry/WOOdometry")
        vwgl_odometry_sub = fastdds_odometry.OdometryDataReader(0, "rt/localization/visual_wheel_gps_localization/VWGLOdometry")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    imu = fastdds_imu.Imu()
    ins = fastdds_ins.Ins()
    gnss = fastdds_gnss.Gnss()
    vio_odometry = fastdds_odometry.Odometry()
    wo_odometry = fastdds_odometry.Odometry()
    vwgl_odometry = fastdds_odometry.Odometry()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (imu_sub.wait_for_sample(2)):
                # Read the received message
                if (imu_sub.take_sample(imu)):
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
            # Wait for a message for 2 seconds
            if (gnss_sub.wait_for_sample(2)):
                # Read the received message
                if (gnss_sub.take_sample(gnss)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (vio_odometry_sub.wait_for_sample(2)):
                # Read the received message
                if (vio_odometry_sub.take_sample(vio_odometry)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (wo_odometry_sub.wait_for_sample(2)):
                # Read the received message
                if (wo_odometry_sub.take_sample(wo_odometry)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (vwgl_odometry_sub.wait_for_sample(2)):
                # Read the received message
                if (vwgl_odometry_sub.take_sample(vwgl_odometry)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
