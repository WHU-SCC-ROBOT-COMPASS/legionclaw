import argparse
import time
import sys

import fastdds_radar_obstacle_list_msg

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
        radar_obstacle_list_msg_pub = fastdds_radar_obstacle_list_msg.RadarObstacleListMsgDataWriter(0, "rt/drivers/radar/RadarObstacleListMsg")
        radar_obstacle_list_msg_f_r_pub = fastdds_radar_obstacle_list_msg.RadarObstacleListMsgDataWriter(0, "rt/drivers/radar/RadarObstacleListMsg_F_r")
        radar_obstacle_list_msg_b_r_pub = fastdds_radar_obstacle_list_msg.RadarObstacleListMsgDataWriter(0, "rt/drivers/radar/RadarObstacleListMsg_B_r")
        radar_obstacle_list_msg_b_l_pub = fastdds_radar_obstacle_list_msg.RadarObstacleListMsgDataWriter(0, "rt/drivers/radar/RadarObstacleListMsg_B_l")
        radar_obstacle_list_msg_f_l_pub = fastdds_radar_obstacle_list_msg.RadarObstacleListMsgDataWriter(0, "rt/drivers/radar/RadarObstacleListMsg_F_l")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    radar_obstacle_list_msg = fastdds_radar_obstacle_list_msg.RadarObstacleListMsg()
    radar_obstacle_list_msg_f_r = fastdds_radar_obstacle_list_msg.RadarObstacleListMsg()
    radar_obstacle_list_msg_b_r = fastdds_radar_obstacle_list_msg.RadarObstacleListMsg()
    radar_obstacle_list_msg_b_l = fastdds_radar_obstacle_list_msg.RadarObstacleListMsg()
    radar_obstacle_list_msg_f_l = fastdds_radar_obstacle_list_msg.RadarObstacleListMsg()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            radar_obstacle_list_msg_pub.write_sample(radar_obstacle_list_msg)
            radar_obstacle_list_msg_f_r_pub.write_sample(radar_obstacle_list_msg_f_r)
            radar_obstacle_list_msg_b_r_pub.write_sample(radar_obstacle_list_msg_b_r)
            radar_obstacle_list_msg_b_l_pub.write_sample(radar_obstacle_list_msg_b_l)
            radar_obstacle_list_msg_f_l_pub.write_sample(radar_obstacle_list_msg_f_l)
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
