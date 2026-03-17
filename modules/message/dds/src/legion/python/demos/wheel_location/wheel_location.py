import argparse
import time
import sys

import fastdds_wheel_info
import fastdds_parking_info_list
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
        location_pub = fastdds_location.LocationDataWriter(0, "rt/localization/wheel_location/Location")
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
        wheel_info_sub = fastdds_wheel_info.WheelInfoDataReader(0, "rt/drivers/canbus/WheelInfo")
        parking_info_list_sub = fastdds_parking_info_list.ParkingInfoListDataReader(0, "rt/perception/fusion/traffic_sign_fusion/ParkingInfoList")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    wheel_info = fastdds_wheel_info.WheelInfo()
    parking_info_list = fastdds_parking_info_list.ParkingInfoList()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (wheel_info_sub.wait_for_sample(2)):
                # Read the received message
                if (wheel_info_sub.take_sample(wheel_info)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (parking_info_list_sub.wait_for_sample(2)):
                # Read the received message
                if (parking_info_list_sub.take_sample(parking_info_list)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
