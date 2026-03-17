import argparse
import time
import sys

import fastdds_control_command
import fastdds_wheel_info
import fastdds_obu_cmd_msg
import fastdds_faults
import fastdds_chassis

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
        wheel_info_pub = fastdds_wheel_info.WheelInfoDataWriter(0, "rt/drivers/canbus/WheelInfo")
        chassis_pub = fastdds_chassis.ChassisDataWriter(0, "rt/drivers/canbus/Chassis")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/drivers/canbus/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    wheel_info = fastdds_wheel_info.WheelInfo()
    chassis = fastdds_chassis.Chassis()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            wheel_info_pub.write_sample(wheel_info)
            chassis_pub.write_sample(chassis)
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
        obu_cmd_msg_sub = fastdds_obu_cmd_msg.ObuCmdMsgDataReader(0, "rt/vui_client/ObuCmdMsg")
        control_command_sub = fastdds_control_command.ControlCommandDataReader(0, "rt/safety/safeguard/SafeControlCommand")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()
    control_command = fastdds_control_command.ControlCommand()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (obu_cmd_msg_sub.wait_for_sample(2)):
                # Read the received message
                if (obu_cmd_msg_sub.take_sample(obu_cmd_msg)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (control_command_sub.wait_for_sample(2)):
                # Read the received message
                if (control_command_sub.take_sample(control_command)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
