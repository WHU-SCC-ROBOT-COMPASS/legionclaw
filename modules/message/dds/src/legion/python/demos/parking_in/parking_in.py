import argparse
import time
import sys

import fastdds_command_respond
import fastdds_parking_info
import fastdds_obu_cmd_msg
import fastdds_module_status
import fastdds_adc_trajectory
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
        command_respond_output_pub = fastdds_command_respond.CommandRespondDataWriter(0, "rt/avp/parking_in/CommandRespond")
        obu_cmd_msg_pub = fastdds_obu_cmd_msg.ObuCmdMsgDataWriter(0, "rt/avp/parking_in/ObuCmdMsg")
        module_status_pub = fastdds_module_status.ModuleStatusDataWriter(0, "rt/avp/parking_in/ModuleStatus")
        parking_info_output_pub = fastdds_parking_info.ParkingInfoDataWriter(0, "rt/avp/parking_in/ParkingInfo")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    command_respond_output = fastdds_command_respond.CommandRespond()
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()
    module_status = fastdds_module_status.ModuleStatus()
    parking_info_output = fastdds_parking_info.ParkingInfo()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            command_respond_output_pub.write_sample(command_respond_output)
            obu_cmd_msg_pub.write_sample(obu_cmd_msg)
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
        comm_command_sub = fastdds_comm_command.CommCommandDataReader(0, "rt/avp/apa_state_machine/PICommCommand")
        command_respond_input_sub = fastdds_command_respond.CommandRespondDataReader(0, "rt/planning/CommandRespond")
        parking_info_input_sub = fastdds_parking_info.ParkingInfoDataReader(0, "rt/avp/parking_search/ParkingInfo")
        adc_trajectory_sub = fastdds_adc_trajectory.ADCTrajectoryDataReader(0, "rt/planning/ADCTrajectory")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    comm_command = fastdds_comm_command.CommCommand()
    command_respond_input = fastdds_command_respond.CommandRespond()
    parking_info_input = fastdds_parking_info.ParkingInfo()
    adc_trajectory = fastdds_adc_trajectory.ADCTrajectory()

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
            if (command_respond_input_sub.wait_for_sample(2)):
                # Read the received message
                if (command_respond_input_sub.take_sample(command_respond_input)):
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
            if (adc_trajectory_sub.wait_for_sample(2)):
                # Read the received message
                if (adc_trajectory_sub.take_sample(adc_trajectory)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
