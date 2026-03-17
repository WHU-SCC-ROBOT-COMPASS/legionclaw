import argparse
import time
import sys

import fastdds_image
import fastdds_pavementype

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
        pavementype_pub = fastdds_pavementype.PavementypeDataWriter(0, "rt/perception/camera/camera_pavemen_type/Pavementype")
        image_pub = fastdds_image.ImageDataWriter(0, "rt/perception/camera/camera_pavemen_type/Image")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    pavementype = fastdds_pavementype.Pavementype()
    image = fastdds_image.Image()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            pavementype_pub.write_sample(pavementype)
            image_pub.write_sample(image)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        pass

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    pass

    try:
        while True:
            pass


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
