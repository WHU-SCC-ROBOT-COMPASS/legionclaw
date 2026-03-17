import argparse
import time
import sys

import fastdds_image
import fastdds_ins
import fastdds_chassis
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
        location_pub = fastdds_location.LocationDataWriter(0, "rt/localization/orb_slam2/Location")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    location = fastdds_location.Location()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            location_pub.write_sample(location)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        ins_sub = fastdds_ins.InsDataReader(0, "rt/drivers/ins/Ins")
        left_image_sub = fastdds_image.ImageDataReader(0, "rt/drivers/camera/LeftImage")
        right_image_sub = fastdds_image.ImageDataReader(0, "rt/drivers/camera/RightImage")
        chassis_sub = fastdds_chassis.ChassisDataReader(0, "rt/drivers/canbus/Chassis")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    ins = fastdds_ins.Ins()
    left_image = fastdds_image.Image()
    right_image = fastdds_image.Image()
    chassis = fastdds_chassis.Chassis()

    try:
        while True:
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
            if (left_image_sub.wait_for_sample(2)):
                # Read the received message
                if (left_image_sub.take_sample(left_image)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (right_image_sub.wait_for_sample(2)):
                # Read the received message
                if (right_image_sub.take_sample(right_image)):
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


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
