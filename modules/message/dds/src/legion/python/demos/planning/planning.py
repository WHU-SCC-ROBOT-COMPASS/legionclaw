import argparse
import time
import sys

import fastdds_stop_info
import fastdds_planning_cmd
import fastdds_parking_out_info
import fastdds_parking_state_display
import fastdds_faults
import fastdds_traffic_events
import fastdds_traffic_light_msg
import fastdds_planning_analysis
import fastdds_events
import fastdds_adc_trajectory
import fastdds_prediction_obstacles
import fastdds_parking_info
import fastdds_lane_list
import fastdds_drivable_region
import fastdds_chassis
import fastdds_routing_response
import fastdds_guide_info
import fastdds_location
import fastdds_obu_cmd_msg
import fastdds_sotif_monitor_result
import fastdds_trajectory_array

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
        adc_trajectory_pub = fastdds_adc_trajectory.ADCTrajectoryDataWriter(0, "rt/planning/ADCTrajectory")
        planning_cmd_pub = fastdds_planning_cmd.PlanningCmdDataWriter(0, "rt/planning/PlanningCmd")
        planning_analysis_pub = fastdds_planning_analysis.PlanningAnalysisDataWriter(0, "rt/planning/PlanningAnalysis")
        parking_state_display_pub = fastdds_parking_state_display.ParkingStateDisplayDataWriter(0, "rt/planning/ParkingStateDisplay")
        trajectory_array_pub = fastdds_trajectory_array.TrajectoryArrayDataWriter(0, "rt/planning/TrajectoryArray")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/planning/Faults")
        events_pub = fastdds_events.EventsDataWriter(0, "rt/planning/Events")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    adc_trajectory = fastdds_adc_trajectory.ADCTrajectory()
    planning_cmd = fastdds_planning_cmd.PlanningCmd()
    planning_analysis = fastdds_planning_analysis.PlanningAnalysis()
    parking_state_display = fastdds_parking_state_display.ParkingStateDisplay()
    trajectory_array = fastdds_trajectory_array.TrajectoryArray()
    faults = fastdds_faults.Faults()
    events = fastdds_events.Events()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            adc_trajectory_pub.write_sample(adc_trajectory)
            planning_cmd_pub.write_sample(planning_cmd)
            planning_analysis_pub.write_sample(planning_analysis)
            parking_state_display_pub.write_sample(parking_state_display)
            trajectory_array_pub.write_sample(trajectory_array)
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
        routing_response_sub = fastdds_routing_response.RoutingResponseDataReader(0, "rt/routing/RoutingResponse")
        local_routing_response_sub = fastdds_routing_response.RoutingResponseDataReader(0, "rt/local_map/LocalRoutingResponse")
        parking_info_sub = fastdds_parking_info.ParkingInfoDataReader(0, "rt/routing/ParkingInfo")
        stop_info_sub = fastdds_stop_info.StopInfoDataReader(0, "rt/routing/StopInfo")
        traffic_light_msg_sub = fastdds_traffic_light_msg.TrafficLightMsgDataReader(0, "rt/perception/fusion/traffic_sign_fusion/TrafficLightMsg")
        location_sub = fastdds_location.LocationDataReader(0, "rt/localization/global_fusion/Location")
        prediction_obstacles_sub = fastdds_prediction_obstacles.PredictionObstaclesDataReader(0, "rt/prediction/PredictionObstacles")
        lane_list_sub = fastdds_lane_list.LaneListDataReader(0, "rtLaneList")
        chassis_sub = fastdds_chassis.ChassisDataReader(0, "rt/drivers/canbus/Chassis")
        sotif_monitor_result_sub = fastdds_sotif_monitor_result.SotifMonitorResultDataReader(0, "rt/safety/sotif_monitor/SotifMonitorResult")
        obu_cmd_msg_sub = fastdds_obu_cmd_msg.ObuCmdMsgDataReader(0, "rt/vui_client/ObuCmdMsg")
        drivable_region_sub = fastdds_drivable_region.DrivableRegionDataReader(0, "rt/perception/fusion/traffic_sign_fusion/DrivableRegion")
        parking_out_info_sub = fastdds_parking_out_info.ParkingOutInfoDataReader(0, "rt/avp/parking_out/ParkingOutInfo")
        guide_info_sub = fastdds_guide_info.GuideInfoDataReader(0, "rt/routing/GuideInfo")
        traffic_events_sub = fastdds_traffic_events.TrafficEventsDataReader(0, "rt/routing/TrafficEvents")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    routing_response = fastdds_routing_response.RoutingResponse()
    local_routing_response = fastdds_routing_response.RoutingResponse()
    parking_info = fastdds_parking_info.ParkingInfo()
    stop_info = fastdds_stop_info.StopInfo()
    traffic_light_msg = fastdds_traffic_light_msg.TrafficLightMsg()
    location = fastdds_location.Location()
    prediction_obstacles = fastdds_prediction_obstacles.PredictionObstacles()
    lane_list = fastdds_lane_list.LaneList()
    chassis = fastdds_chassis.Chassis()
    sotif_monitor_result = fastdds_sotif_monitor_result.SotifMonitorResult()
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()
    drivable_region = fastdds_drivable_region.DrivableRegion()
    parking_out_info = fastdds_parking_out_info.ParkingOutInfo()
    guide_info = fastdds_guide_info.GuideInfo()
    traffic_events = fastdds_traffic_events.TrafficEvents()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (routing_response_sub.wait_for_sample(2)):
                # Read the received message
                if (routing_response_sub.take_sample(routing_response)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (local_routing_response_sub.wait_for_sample(2)):
                # Read the received message
                if (local_routing_response_sub.take_sample(local_routing_response)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (parking_info_sub.wait_for_sample(2)):
                # Read the received message
                if (parking_info_sub.take_sample(parking_info)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (stop_info_sub.wait_for_sample(2)):
                # Read the received message
                if (stop_info_sub.take_sample(stop_info)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (traffic_light_msg_sub.wait_for_sample(2)):
                # Read the received message
                if (traffic_light_msg_sub.take_sample(traffic_light_msg)):
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
            if (prediction_obstacles_sub.wait_for_sample(2)):
                # Read the received message
                if (prediction_obstacles_sub.take_sample(prediction_obstacles)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (lane_list_sub.wait_for_sample(2)):
                # Read the received message
                if (lane_list_sub.take_sample(lane_list)):
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
            if (drivable_region_sub.wait_for_sample(2)):
                # Read the received message
                if (drivable_region_sub.take_sample(drivable_region)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (parking_out_info_sub.wait_for_sample(2)):
                # Read the received message
                if (parking_out_info_sub.take_sample(parking_out_info)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (guide_info_sub.wait_for_sample(2)):
                # Read the received message
                if (guide_info_sub.take_sample(guide_info)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (traffic_events_sub.wait_for_sample(2)):
                # Read the received message
                if (traffic_events_sub.take_sample(traffic_events)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
