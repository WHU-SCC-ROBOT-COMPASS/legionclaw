import argparse
import time
import sys

import fastdds_module_status
import fastdds_command_respond
import fastdds_faults
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
        command_respond_output_pub = fastdds_command_respond.CommandRespondDataWriter(0, "rt/avp/apa_state_machine/CommandRespond")
        ps_comm_command_pub = fastdds_comm_command.CommCommandDataWriter(0, "rt/avp/apa_state_machine/PSCommCommand")
        pi_comm_command_pub = fastdds_comm_command.CommCommandDataWriter(0, "rt/avp/apa_state_machine/PICommCommand")
        module_status_output_pub = fastdds_module_status.ModuleStatusDataWriter(0, "rt/avp/apa_state_machine/ModuleStatus")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    command_respond_output = fastdds_command_respond.CommandRespond()
    ps_comm_command = fastdds_comm_command.CommCommand()
    pi_comm_command = fastdds_comm_command.CommCommand()
    module_status_output = fastdds_module_status.ModuleStatus()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            command_respond_output_pub.write_sample(command_respond_output)
            ps_comm_command_pub.write_sample(ps_comm_command)
            pi_comm_command_pub.write_sample(pi_comm_command)
            module_status_output_pub.write_sample(module_status_output)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        comm_command_input_sub = fastdds_comm_command.CommCommandDataReader(0, "rt/arbitration/CommCommand")
        pi_command_respond_sub = fastdds_command_respond.CommandRespondDataReader(0, "rt/avp/parking_in/PICommandRespond")
        ps_command_respond_sub = fastdds_command_respond.CommandRespondDataReader(0, "rt/avp/parking_search/PSCommandRespond")
        pi_module_status_sub = fastdds_module_status.ModuleStatusDataReader(0, "rt/avp/parking_in/PIModuleStatus")
        ps_module_status_sub = fastdds_module_status.ModuleStatusDataReader(0, "rt/avp/parking_search/PSModuleStatus")
        faults_sub = fastdds_faults.FaultsDataReader(0, "rt/safety/safeguard/Faults")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    comm_command_input = fastdds_comm_command.CommCommand()
    pi_command_respond = fastdds_command_respond.CommandRespond()
    ps_command_respond = fastdds_command_respond.CommandRespond()
    pi_module_status = fastdds_module_status.ModuleStatus()
    ps_module_status = fastdds_module_status.ModuleStatus()
    faults = fastdds_faults.Faults()

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
            if (pi_command_respond_sub.wait_for_sample(2)):
                # Read the received message
                if (pi_command_respond_sub.take_sample(pi_command_respond)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (ps_command_respond_sub.wait_for_sample(2)):
                # Read the received message
                if (ps_command_respond_sub.take_sample(ps_command_respond)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (pi_module_status_sub.wait_for_sample(2)):
                # Read the received message
                if (pi_module_status_sub.take_sample(pi_module_status)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (ps_module_status_sub.wait_for_sample(2)):
                # Read the received message
                if (ps_module_status_sub.take_sample(ps_module_status)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (faults_sub.wait_for_sample(2)):
                # Read the received message
                if (faults_sub.take_sample(faults)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
