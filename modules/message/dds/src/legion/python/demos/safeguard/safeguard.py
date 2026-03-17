import argparse
import time
import sys

import fastdds_control_command
import fastdds_security_decision
import fastdds_sotif_monitor_result
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
        control_command_output_pub = fastdds_control_command.ControlCommandDataWriter(0, "rt/safety/safeguard/SafeControlCommand")
        faults_output_pub = fastdds_faults.FaultsDataWriter(0, "rt/safety/safeguard/Faults")
        security_decision_pub = fastdds_security_decision.SecurityDecisionDataWriter(0, "rt/safety/safeguard/SecurityDecision")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    control_command_output = fastdds_control_command.ControlCommand()
    faults_output = fastdds_faults.Faults()
    security_decision = fastdds_security_decision.SecurityDecision()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            control_command_output_pub.write_sample(control_command_output)
            faults_output_pub.write_sample(faults_output)
            security_decision_pub.write_sample(security_decision)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        control_command_input_sub = fastdds_control_command.ControlCommandDataReader(0, "rt/control/ControlCommand")
        faults_input_sub = fastdds_faults.FaultsDataReader(0, "rtFaults")
        chassis_sub = fastdds_chassis.ChassisDataReader(0, "rt/drivers/canbus/Chassis")
        sotif_monitor_result_sub = fastdds_sotif_monitor_result.SotifMonitorResultDataReader(0, "rt/safety/sotif_monitor/SotifMonitorResult")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    control_command_input = fastdds_control_command.ControlCommand()
    faults_input = fastdds_faults.Faults()
    chassis = fastdds_chassis.Chassis()
    sotif_monitor_result = fastdds_sotif_monitor_result.SotifMonitorResult()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (control_command_input_sub.wait_for_sample(2)):
                # Read the received message
                if (control_command_input_sub.take_sample(control_command_input)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (faults_input_sub.wait_for_sample(2)):
                # Read the received message
                if (faults_input_sub.take_sample(faults_input)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (chassis_sub.wait_for_sample(2)):
                # Read the received message
                if (chassis_sub.take_sample(chassis)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (sotif_monitor_result_sub.wait_for_sample(2)):
                # Read the received message
                if (sotif_monitor_result_sub.take_sample(sotif_monitor_result)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
