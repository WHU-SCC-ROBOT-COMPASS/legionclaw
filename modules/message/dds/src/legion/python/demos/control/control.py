import argparse
import time
import sys

import fastdds_planning_cmd
import fastdds_control_command
import fastdds_location
import fastdds_events
import fastdds_obu_cmd_msg
import fastdds_control_analysis
import fastdds_adc_trajectory
import fastdds_chassis
import fastdds_faults

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
        control_command_pub = fastdds_control_command.ControlCommandDataWriter(0, "rt/control/ControlCommand")
        control_analysis_pub = fastdds_control_analysis.ControlAnalysisDataWriter(0, "rt/control/ControlAnalysis")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/control/Faults")
        events_pub = fastdds_events.EventsDataWriter(0, "rt/control/Events")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    control_command = fastdds_control_command.ControlCommand()
    control_analysis = fastdds_control_analysis.ControlAnalysis()
    faults = fastdds_faults.Faults()
    events = fastdds_events.Events()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            control_command_pub.write_sample(control_command)
            control_analysis_pub.write_sample(control_analysis)
            faults_pub.write_sample(faults)
            events_pub.write_sample(events)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        adc_trajectory_sub = fastdds_adc_trajectory.ADCTrajectoryDataReader(0, "rt/planning/ADCTrajectory")
        chassis_sub = fastdds_chassis.ChassisDataReader(0, "rt/drivers/canbus/Chassis")
        location_sub = fastdds_location.LocationDataReader(0, "rt/localization/global_fusion/Location")
        obu_cmd_msg_sub = fastdds_obu_cmd_msg.ObuCmdMsgDataReader(0, "rt/vui_client/ObuCmdMsg")
        planning_cmd_sub = fastdds_planning_cmd.PlanningCmdDataReader(0, "rt/planning/PlanningCmd")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    adc_trajectory = fastdds_adc_trajectory.ADCTrajectory()
    chassis = fastdds_chassis.Chassis()
    location = fastdds_location.Location()
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()
    planning_cmd = fastdds_planning_cmd.PlanningCmd()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (adc_trajectory_sub.wait_for_sample(2)):
                # Read the received message
                if (adc_trajectory_sub.take_sample(adc_trajectory)):
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
            if (location_sub.wait_for_sample(2)):
                # Read the received message
                if (location_sub.take_sample(location)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
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
            if (planning_cmd_sub.wait_for_sample(2)):
                # Read the received message
                if (planning_cmd_sub.take_sample(planning_cmd)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
