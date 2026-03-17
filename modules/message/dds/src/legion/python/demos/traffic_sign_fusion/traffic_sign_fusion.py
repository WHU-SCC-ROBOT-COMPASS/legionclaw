import argparse
import time
import sys

import fastdds_traffic_light_msg
import fastdds_parking_info_list
import fastdds_location
import fastdds_lane_list
import fastdds_drivable_region
import fastdds_camera_parking_info_list
import fastdds_faults
import fastdds_obstacle_list

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
        traffic_light_msg_pub = fastdds_traffic_light_msg.TrafficLightMsgDataWriter(0, "rt/perception/fusion/traffic_sign_fusion/TrafficLightMsg")
        lane_list_pub = fastdds_lane_list.LaneListDataWriter(0, "rt/perception/fusion/traffic_sign_fusion/MFLaneList")
        parking_info_list_pub = fastdds_parking_info_list.ParkingInfoListDataWriter(0, "rt/perception/fusion/traffic_sign_fusion/ParkingInfoList")
        wl_parking_info_list_pub = fastdds_parking_info_list.ParkingInfoListDataWriter(0, "rt/perception/fusion/traffic_sign_fusion/WLParkingInfoList")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/perception/fusion/traffic_sign_fusion/Faults")
        drivable_region_pub = fastdds_drivable_region.DrivableRegionDataWriter(0, "rt/perception/fusion/traffic_sign_fusion/DrivableRegion")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    traffic_light_msg = fastdds_traffic_light_msg.TrafficLightMsg()
    lane_list = fastdds_lane_list.LaneList()
    parking_info_list = fastdds_parking_info_list.ParkingInfoList()
    wl_parking_info_list = fastdds_parking_info_list.ParkingInfoList()
    faults = fastdds_faults.Faults()
    drivable_region = fastdds_drivable_region.DrivableRegion()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            traffic_light_msg_pub.write_sample(traffic_light_msg)
            lane_list_pub.write_sample(lane_list)
            parking_info_list_pub.write_sample(parking_info_list)
            wl_parking_info_list_pub.write_sample(wl_parking_info_list)
            faults_pub.write_sample(faults)
            drivable_region_pub.write_sample(drivable_region)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        location_sub = fastdds_location.LocationDataReader(0, "rt/localization/global_fusion/Location")
        obstacle_list_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/fusion/motion_manager/MMObstacleList")
        fisheye_camera_parking_info_list3_sub = fastdds_camera_parking_info_list.CameraParkingInfoListDataReader(0, "rt/perception/camera/camera_parking_detect/FisheyeCameraParkingInfoList3")
        fisheye_camera_parking_info_list4_sub = fastdds_camera_parking_info_list.CameraParkingInfoListDataReader(0, "rt/perception/camera/camera_parking_detect/FisheyeCameraParkingInfoList4")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    location = fastdds_location.Location()
    obstacle_list = fastdds_obstacle_list.ObstacleList()
    fisheye_camera_parking_info_list3 = fastdds_camera_parking_info_list.CameraParkingInfoList()
    fisheye_camera_parking_info_list4 = fastdds_camera_parking_info_list.CameraParkingInfoList()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (location_sub.wait_for_sample(2)):
                # Read the received message
                if (location_sub.take_sample(location)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (obstacle_list_sub.wait_for_sample(2)):
                # Read the received message
                if (obstacle_list_sub.take_sample(obstacle_list)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (fisheye_camera_parking_info_list3_sub.wait_for_sample(2)):
                # Read the received message
                if (fisheye_camera_parking_info_list3_sub.take_sample(fisheye_camera_parking_info_list3)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (fisheye_camera_parking_info_list4_sub.wait_for_sample(2)):
                # Read the received message
                if (fisheye_camera_parking_info_list4_sub.take_sample(fisheye_camera_parking_info_list4)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
