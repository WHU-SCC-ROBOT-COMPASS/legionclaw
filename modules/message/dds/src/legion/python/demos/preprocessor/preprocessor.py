import argparse
import time
import sys

import fastdds_radar_obstacle_list_msg
import fastdds_location
import fastdds_faults
import fastdds_obstacle_list
import fastdds_ultrasonic

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
        obstacle_list_output_pub = fastdds_obstacle_list.ObstacleListDataWriter(0, "rt/perception/fusion/preprocessor/PObstacleList")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/perception/fusion/preprocessor/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    obstacle_list_output = fastdds_obstacle_list.ObstacleList()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            obstacle_list_output_pub.write_sample(obstacle_list_output)
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
        location_sub = fastdds_location.LocationDataReader(0, "rt/localization/global_fusion/Location")
        c3odobstacle_list4_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/camera/camera_3d_object_detect/C3ODObstacleList4")
        c3odobstacle_list3_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/camera/camera_3d_object_detect/C3ODObstacleList3")
        c3odobstacle_list2_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/camera/camera_3d_object_detect/C3ODObstacleList2")
        c3odobstacle_list1_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/camera/camera_3d_object_detect/C3ODObstacleList1")
        ultrasonic_sub = fastdds_ultrasonic.UltrasonicDataReader(0, "rt/drivers/uss/Ultrasonic")
        radar_obstacle_list_msg_sub = fastdds_radar_obstacle_list_msg.RadarObstacleListMsgDataReader(0, "rt/drivers/radar/RadarObstacleListMsg")
        radar_obstacle_list_msg_f_r_sub = fastdds_radar_obstacle_list_msg.RadarObstacleListMsgDataReader(0, "rt/drivers/radar/RadarObstacleListMsg_F_r")
        radar_obstacle_list_msg_b_r_sub = fastdds_radar_obstacle_list_msg.RadarObstacleListMsgDataReader(0, "rt/drivers/radar/RadarObstacleListMsg_B_r")
        radar_obstacle_list_msg_b_l_sub = fastdds_radar_obstacle_list_msg.RadarObstacleListMsgDataReader(0, "rt/drivers/radar/RadarObstacleListMsg_B_l")
        radar_obstacle_list_msg_f_l_sub = fastdds_radar_obstacle_list_msg.RadarObstacleListMsgDataReader(0, "rt/drivers/radar/RadarObstacleListMsg_F_l")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    location = fastdds_location.Location()
    c3odobstacle_list4 = fastdds_obstacle_list.ObstacleList()
    c3odobstacle_list3 = fastdds_obstacle_list.ObstacleList()
    c3odobstacle_list2 = fastdds_obstacle_list.ObstacleList()
    c3odobstacle_list1 = fastdds_obstacle_list.ObstacleList()
    ultrasonic = fastdds_ultrasonic.Ultrasonic()
    radar_obstacle_list_msg = fastdds_radar_obstacle_list_msg.RadarObstacleListMsg()
    radar_obstacle_list_msg_f_r = fastdds_radar_obstacle_list_msg.RadarObstacleListMsg()
    radar_obstacle_list_msg_b_r = fastdds_radar_obstacle_list_msg.RadarObstacleListMsg()
    radar_obstacle_list_msg_b_l = fastdds_radar_obstacle_list_msg.RadarObstacleListMsg()
    radar_obstacle_list_msg_f_l = fastdds_radar_obstacle_list_msg.RadarObstacleListMsg()

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
            if (c3odobstacle_list4_sub.wait_for_sample(2)):
                # Read the received message
                if (c3odobstacle_list4_sub.take_sample(c3odobstacle_list4)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (c3odobstacle_list3_sub.wait_for_sample(2)):
                # Read the received message
                if (c3odobstacle_list3_sub.take_sample(c3odobstacle_list3)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (c3odobstacle_list2_sub.wait_for_sample(2)):
                # Read the received message
                if (c3odobstacle_list2_sub.take_sample(c3odobstacle_list2)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (c3odobstacle_list1_sub.wait_for_sample(2)):
                # Read the received message
                if (c3odobstacle_list1_sub.take_sample(c3odobstacle_list1)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (ultrasonic_sub.wait_for_sample(2)):
                # Read the received message
                if (ultrasonic_sub.take_sample(ultrasonic)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (radar_obstacle_list_msg_sub.wait_for_sample(2)):
                # Read the received message
                if (radar_obstacle_list_msg_sub.take_sample(radar_obstacle_list_msg)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (radar_obstacle_list_msg_f_r_sub.wait_for_sample(2)):
                # Read the received message
                if (radar_obstacle_list_msg_f_r_sub.take_sample(radar_obstacle_list_msg_f_r)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (radar_obstacle_list_msg_b_r_sub.wait_for_sample(2)):
                # Read the received message
                if (radar_obstacle_list_msg_b_r_sub.take_sample(radar_obstacle_list_msg_b_r)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (radar_obstacle_list_msg_b_l_sub.wait_for_sample(2)):
                # Read the received message
                if (radar_obstacle_list_msg_b_l_sub.take_sample(radar_obstacle_list_msg_b_l)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (radar_obstacle_list_msg_f_l_sub.wait_for_sample(2)):
                # Read the received message
                if (radar_obstacle_list_msg_f_l_sub.take_sample(radar_obstacle_list_msg_f_l)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
