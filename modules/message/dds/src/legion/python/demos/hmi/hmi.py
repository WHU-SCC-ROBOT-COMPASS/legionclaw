import argparse
import time
import sys

import fastdds_command_respond
import fastdds_parking_info_list
import fastdds_parking_info
import fastdds_module_status
import fastdds_comm_command
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
        comm_command_pub = fastdds_comm_command.CommCommandDataWriter(0, "rt/hmi/CommCommand")
        parking_info_pub = fastdds_parking_info.ParkingInfoDataWriter(0, "rt/hmi/ParkingInfo")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    comm_command = fastdds_comm_command.CommCommand()
    parking_info = fastdds_parking_info.ParkingInfo()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            comm_command_pub.write_sample(comm_command)
            parking_info_pub.write_sample(parking_info)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        command_respond_sub = fastdds_command_respond.CommandRespondDataReader(0, "rt/arbitration/CommandRespond")
        obstacle_list_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/fusion/motion_manager/MMObstacleList")
        parking_info_list_sub = fastdds_parking_info_list.ParkingInfoListDataReader(0, "rt/perception/fusion/traffic_sign_fusion/ParkingInfoList")
        module_status_sub = fastdds_module_status.ModuleStatusDataReader(0, "rt/avp/apa_state_machine/ModuleStatus")
        module_status_sub = fastdds_module_status.ModuleStatusDataReader(0, "rt/avp/parking_in/ModuleStatus")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    command_respond = fastdds_command_respond.CommandRespond()
    obstacle_list = fastdds_obstacle_list.ObstacleList()
    parking_info_list = fastdds_parking_info_list.ParkingInfoList()
    module_status = fastdds_module_status.ModuleStatus()
    module_status = fastdds_module_status.ModuleStatus()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (command_respond_sub.wait_for_sample(2)):
                # Read the received message
                if (command_respond_sub.take_sample(command_respond)):
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
            if (parking_info_list_sub.wait_for_sample(2)):
                # Read the received message
                if (parking_info_list_sub.take_sample(parking_info_list)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (module_status_sub.wait_for_sample(2)):
                # Read the received message
                if (module_status_sub.take_sample(module_status)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (module_status_sub.wait_for_sample(2)):
                # Read the received message
                if (module_status_sub.take_sample(module_status)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
