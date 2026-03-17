import argparse
import time
import sys

import fastdds_parking_info_list
import fastdds_hmi_obstacle_list
import fastdds_location
import fastdds_parking_state_display
import fastdds_diagnostic_array
import fastdds_hmi_parking_info
import fastdds_hmi_obu_cmd_msg
import fastdds_events
import fastdds_parking_info
import fastdds_obu_cmd_msg
import fastdds_adc_trajectory
import fastdds_chassis
import fastdds_hmi_diagnostic_array
import fastdds_hmi_parking_info_list
import fastdds_obstacle_list
import fastdds_hmi_vehicle_msg
import fastdds_hmi_trajectory
import fastdds_hmi_parking_state_display

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
        hmi_diagnostic_array_pub = fastdds_hmi_diagnostic_array.HMIDiagnosticArrayDataWriter(0, "rt/hmi_bridge/HMIDiagnosticArray")
        hmi_trajectory_pub = fastdds_hmi_trajectory.HMITrajectoryDataWriter(0, "rt/hmi_bridge/HMIADCTrajectory")
        hmi_parking_state_display_pub = fastdds_hmi_parking_state_display.HMIParkingStateDisplayDataWriter(0, "rt/hmi_bridge/HMIParkingStateDisplay")
        hmi_obstacle_list_pub = fastdds_hmi_obstacle_list.HMIObstacleListDataWriter(0, "rt/hmi_bridge/HMIObstacleList")
        hmi_parking_info_list_pub = fastdds_hmi_parking_info_list.HMIParkingInfoListDataWriter(0, "rt/hmi_bridge/HMIParkingInfoList")
        hmi_vehicle_msg_pub = fastdds_hmi_vehicle_msg.HMIVehicleMsgDataWriter(0, "rt/hmi_bridge/HMIVehicleMsg")
        parking_info_pub = fastdds_parking_info.ParkingInfoDataWriter(0, "rt/hmi_bridge/ParkingInfo")
        obu_cmd_msg_pub = fastdds_obu_cmd_msg.ObuCmdMsgDataWriter(0, "rt/hmi_bridge/ObuCmdMsg")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    hmi_diagnostic_array = fastdds_hmi_diagnostic_array.HMIDiagnosticArray()
    hmi_trajectory = fastdds_hmi_trajectory.HMITrajectory()
    hmi_parking_state_display = fastdds_hmi_parking_state_display.HMIParkingStateDisplay()
    hmi_obstacle_list = fastdds_hmi_obstacle_list.HMIObstacleList()
    hmi_parking_info_list = fastdds_hmi_parking_info_list.HMIParkingInfoList()
    hmi_vehicle_msg = fastdds_hmi_vehicle_msg.HMIVehicleMsg()
    parking_info = fastdds_parking_info.ParkingInfo()
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            hmi_diagnostic_array_pub.write_sample(hmi_diagnostic_array)
            hmi_trajectory_pub.write_sample(hmi_trajectory)
            hmi_parking_state_display_pub.write_sample(hmi_parking_state_display)
            hmi_obstacle_list_pub.write_sample(hmi_obstacle_list)
            hmi_parking_info_list_pub.write_sample(hmi_parking_info_list)
            hmi_vehicle_msg_pub.write_sample(hmi_vehicle_msg)
            parking_info_pub.write_sample(parking_info)
            obu_cmd_msg_pub.write_sample(obu_cmd_msg)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        hmi_parking_info_sub = fastdds_hmi_parking_info.HMIParkingInfoDataReader(0, "rt/hmi/HMIParkingInfo")
        hmi_obu_cmd_msg_sub = fastdds_hmi_obu_cmd_msg.HMIObuCmdMsgDataReader(0, "rt/hmi/HMIObuCmdMsg")
        diagnostic_array_sub = fastdds_diagnostic_array.DiagnosticArrayDataReader(0, "rt/hmi_bridge/DiagnosticArray")
        adc_trajectory_sub = fastdds_adc_trajectory.ADCTrajectoryDataReader(0, "rt/hmi_bridge/ADCTrajectory")
        parking_state_display_sub = fastdds_parking_state_display.ParkingStateDisplayDataReader(0, "rt/hmi_bridge/ParkingStateDisplay")
        events_sub = fastdds_events.EventsDataReader(0, "rt/hmi_bridge/Events")
        obstacle_list_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/hmi_bridge/ObstacleList")
        parking_info_list_sub = fastdds_parking_info_list.ParkingInfoListDataReader(0, "rt/hmi_bridge/ParkingInfoList")
        location_sub = fastdds_location.LocationDataReader(0, "rt/hmi_bridge/Location")
        chassis_sub = fastdds_chassis.ChassisDataReader(0, "rt/hmi_bridge/Chassis")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    hmi_parking_info = fastdds_hmi_parking_info.HMIParkingInfo()
    hmi_obu_cmd_msg = fastdds_hmi_obu_cmd_msg.HMIObuCmdMsg()
    diagnostic_array = fastdds_diagnostic_array.DiagnosticArray()
    adc_trajectory = fastdds_adc_trajectory.ADCTrajectory()
    parking_state_display = fastdds_parking_state_display.ParkingStateDisplay()
    events = fastdds_events.Events()
    obstacle_list = fastdds_obstacle_list.ObstacleList()
    parking_info_list = fastdds_parking_info_list.ParkingInfoList()
    location = fastdds_location.Location()
    chassis = fastdds_chassis.Chassis()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (hmi_parking_info_sub.wait_for_sample(2)):
                # Read the received message
                if (hmi_parking_info_sub.take_sample(hmi_parking_info)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (hmi_obu_cmd_msg_sub.wait_for_sample(2)):
                # Read the received message
                if (hmi_obu_cmd_msg_sub.take_sample(hmi_obu_cmd_msg)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (diagnostic_array_sub.wait_for_sample(2)):
                # Read the received message
                if (diagnostic_array_sub.take_sample(diagnostic_array)):
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
            # Wait for a message for 2 seconds
            if (parking_state_display_sub.wait_for_sample(2)):
                # Read the received message
                if (parking_state_display_sub.take_sample(parking_state_display)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (events_sub.wait_for_sample(2)):
                # Read the received message
                if (events_sub.take_sample(events)):
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
            if (location_sub.wait_for_sample(2)):
                # Read the received message
                if (location_sub.take_sample(location)):
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


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
