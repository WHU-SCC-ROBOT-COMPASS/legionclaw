import argparse
import time
import sys

import fastdds_command_respond
import fastdds_parking_info_list
import fastdds_parking_info
import fastdds_module_status
import fastdds_comm_command

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
        command_respond_pub = fastdds_command_respond.CommandRespondDataWriter(0, "rt/avp/parking_search/CommandRespond")
        module_status_pub = fastdds_module_status.ModuleStatusDataWriter(0, "rt/avp/parking_search/ModuleStatus")
        parking_info_output_pub = fastdds_parking_info.ParkingInfoDataWriter(0, "rt/avp/parking_search/ParkingInfo")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    command_respond = fastdds_command_respond.CommandRespond()
    module_status = fastdds_module_status.ModuleStatus()
    parking_info_output = fastdds_parking_info.ParkingInfo()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            command_respond_pub.write_sample(command_respond)
            module_status_pub.write_sample(module_status)
            parking_info_output_pub.write_sample(parking_info_output)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        comm_command_sub = fastdds_comm_command.CommCommandDataReader(0, "rt/avp/apa_state_machine/PSCommCommand")
        parking_info_input_sub = fastdds_parking_info.ParkingInfoDataReader(0, "rt/hmi/ParkingInfo")
        parking_info_list_sub = fastdds_parking_info_list.ParkingInfoListDataReader(0, "rt/perception/fusion/traffic_sign_fusion/ParkingInfoList")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    comm_command = fastdds_comm_command.CommCommand()
    parking_info_input = fastdds_parking_info.ParkingInfo()
    parking_info_list = fastdds_parking_info_list.ParkingInfoList()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (comm_command_sub.wait_for_sample(2)):
                # Read the received message
                if (comm_command_sub.take_sample(comm_command)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (parking_info_input_sub.wait_for_sample(2)):
                # Read the received message
                if (parking_info_input_sub.take_sample(parking_info_input)):
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
