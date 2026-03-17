import argparse
import time
import sys

import fastdds_image
import fastdds_camera_parking_info_list
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
        camera_parking_info_list1_pub = fastdds_camera_parking_info_list.CameraParkingInfoListDataWriter(0, "rt/perception/camera/camera_parking_detect/CameraParkingInfoList1")
        camera_parking_info_list3_pub = fastdds_camera_parking_info_list.CameraParkingInfoListDataWriter(0, "rt/perception/camera/camera_parking_detect/CameraParkingInfoList3")
        camera_parking_info_list4_pub = fastdds_camera_parking_info_list.CameraParkingInfoListDataWriter(0, "rt/perception/camera/camera_parking_detect/CameraParkingInfoList4")
        fisheye_camera_parking_info_list3_pub = fastdds_camera_parking_info_list.CameraParkingInfoListDataWriter(0, "rt/perception/camera/camera_parking_detect/FisheyeCameraParkingInfoList3")
        fisheye_camera_parking_info_list4_pub = fastdds_camera_parking_info_list.CameraParkingInfoListDataWriter(0, "rt/perception/camera/camera_parking_detect/FisheyeCameraParkingInfoList4")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/perception/camera/camera_parking_detect/Faults")
        c3odobstacle_list1_pub = fastdds_obstacle_list.ObstacleListDataWriter(0, "rt/perception/camera/camera_parking_detect/C3ODObstacleList1")
        c3odobstacle_list3_pub = fastdds_obstacle_list.ObstacleListDataWriter(0, "rt/perception/camera/camera_parking_detect/C3ODObstacleList3")
        c3odobstacle_list4_pub = fastdds_obstacle_list.ObstacleListDataWriter(0, "rt/perception/camera/camera_parking_detect/C3ODObstacleList4")
        outputimage1_pub = fastdds_image.ImageDataWriter(0, "rt/perception/camera/camera_parking_detect/Outputimage1")
        output_image2_pub = fastdds_image.ImageDataWriter(0, "rt/perception/camera/camera_parking_detect/OutputImage2")
        output_image3_pub = fastdds_image.ImageDataWriter(0, "rt/perception/camera/camera_parking_detect/OutputImage3")
        output_image4_pub = fastdds_image.ImageDataWriter(0, "rt/perception/camera/camera_parking_detect/OutputImage4")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    camera_parking_info_list1 = fastdds_camera_parking_info_list.CameraParkingInfoList()
    camera_parking_info_list3 = fastdds_camera_parking_info_list.CameraParkingInfoList()
    camera_parking_info_list4 = fastdds_camera_parking_info_list.CameraParkingInfoList()
    fisheye_camera_parking_info_list3 = fastdds_camera_parking_info_list.CameraParkingInfoList()
    fisheye_camera_parking_info_list4 = fastdds_camera_parking_info_list.CameraParkingInfoList()
    faults = fastdds_faults.Faults()
    c3odobstacle_list1 = fastdds_obstacle_list.ObstacleList()
    c3odobstacle_list3 = fastdds_obstacle_list.ObstacleList()
    c3odobstacle_list4 = fastdds_obstacle_list.ObstacleList()
    outputimage1 = fastdds_image.Image()
    output_image2 = fastdds_image.Image()
    output_image3 = fastdds_image.Image()
    output_image4 = fastdds_image.Image()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            camera_parking_info_list1_pub.write_sample(camera_parking_info_list1)
            camera_parking_info_list3_pub.write_sample(camera_parking_info_list3)
            camera_parking_info_list4_pub.write_sample(camera_parking_info_list4)
            fisheye_camera_parking_info_list3_pub.write_sample(fisheye_camera_parking_info_list3)
            fisheye_camera_parking_info_list4_pub.write_sample(fisheye_camera_parking_info_list4)
            faults_pub.write_sample(faults)
            c3odobstacle_list1_pub.write_sample(c3odobstacle_list1)
            c3odobstacle_list3_pub.write_sample(c3odobstacle_list3)
            c3odobstacle_list4_pub.write_sample(c3odobstacle_list4)
            outputimage1_pub.write_sample(outputimage1)
            output_image2_pub.write_sample(output_image2)
            output_image3_pub.write_sample(output_image3)
            output_image4_pub.write_sample(output_image4)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        image_input_sub = fastdds_image.ImageDataReader(0, "rt/drivers/camera/Image")

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
