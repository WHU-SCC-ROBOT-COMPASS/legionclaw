import argparse
import time
import sys

import fastdds_module_status
import fastdds_command_respond
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
        comm_command_output_pub = fastdds_comm_command.CommCommandDataWriter(0, "rt/arbitration/CommCommand")
        command_respond_output_pub = fastdds_command_respond.CommandRespondDataWriter(0, "rt/arbitration/CommandRespond")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    comm_command_output = fastdds_comm_command.CommCommand()
    command_respond_output = fastdds_command_respond.CommandRespond()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            comm_command_output_pub.write_sample(comm_command_output)
            command_respond_output_pub.write_sample(command_respond_output)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        comm_command_input_sub = fastdds_comm_command.CommCommandDataReader(0, "rt/hmi/CommCommand")
        command_respond_input_sub = fastdds_command_respond.CommandRespondDataReader(0, "rt/avp/apa_state_machine/CommandRespond")
        module_status_sub = fastdds_module_status.ModuleStatusDataReader(0, "rt/avp/apa_state_machine/ModuleStatus")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    comm_command_input = fastdds_comm_command.CommCommand()
    command_respond_input = fastdds_command_respond.CommandRespond()
    module_status = fastdds_module_status.ModuleStatus()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (comm_command_input_sub.wait_for_sample(2)):
                # Read the received message
                if (comm_command_input_sub.take_sample(comm_command_input)):
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
