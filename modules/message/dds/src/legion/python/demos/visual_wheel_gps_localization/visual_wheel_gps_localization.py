import argparse
import time
import sys

import fastdds_odometry
import fastdds_image
import fastdds_faults
import fastdds_chassis
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
        odometry_pub = fastdds_odometry.OdometryDataWriter(0, "rt/localization/visual_wheel_gps_localization/VWGLOdometry")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/localization/visual_wheel_gps_localization/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    odometry = fastdds_odometry.Odometry()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            odometry_pub.write_sample(odometry)
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
        chassis_sub = fastdds_chassis.ChassisDataReader(0, "rt/drivers/canbus/Chassis")
        ins_sub = fastdds_ins.InsDataReader(0, "rt/drivers/ins/Ins")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    image = fastdds_image.Image()
    chassis = fastdds_chassis.Chassis()
    ins = fastdds_ins.Ins()

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
            # Wait for a message for 2 seconds
            if (chassis_sub.wait_for_sample(2)):
                # Read the received message
                if (chassis_sub.take_sample(chassis)):
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
