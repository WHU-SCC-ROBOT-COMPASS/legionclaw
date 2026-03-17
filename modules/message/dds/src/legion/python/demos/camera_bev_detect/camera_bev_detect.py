import argparse
import time
import sys

import fastdds_image

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
        camer_bev_single_output_image_pub = fastdds_image.ImageDataWriter(0, "rt/perception/camera/camera_bev_detect/CamerBevSingleOutputImage")
        camera_bev_output_image_pub = fastdds_image.ImageDataWriter(0, "rt/perception/camera/camera_bev_detect/CameraBevOutputImage")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    camer_bev_single_output_image = fastdds_image.Image()
    camera_bev_output_image = fastdds_image.Image()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            camer_bev_single_output_image_pub.write_sample(camer_bev_single_output_image)
            camera_bev_output_image_pub.write_sample(camera_bev_output_image)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        image_input_sub = fastdds_image.ImageDataReader(0, "rt/perception/camera/camera_bev_detect/CameraBevInputImage")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    image_input = fastdds_image.Image()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (image_input_sub.wait_for_sample(2)):
                # Read the received message
                if (image_input_sub.take_sample(image_input)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
