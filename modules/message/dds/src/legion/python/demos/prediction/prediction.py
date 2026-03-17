import argparse
import time
import sys

import fastdds_odometry
import fastdds_location
import fastdds_prediction_obstacles
import fastdds_lane_list
import fastdds_obu_cmd_msg
import fastdds_local_map
import fastdds_adc_trajectory
import fastdds_faults
import fastdds_obstacle_list
import fastdds_routing_response
import fastdds_traffic_events

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
        prediction_obstacles_pub = fastdds_prediction_obstacles.PredictionObstaclesDataWriter(0, "rt/prediction/PredictionObstacles")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/prediction/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    prediction_obstacles = fastdds_prediction_obstacles.PredictionObstacles()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            prediction_obstacles_pub.write_sample(prediction_obstacles)
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
        location_sub = fastdds_location.LocationDataReader(0, "rt/localization/global_fusion/Location")
        adc_trajectory_sub = fastdds_adc_trajectory.ADCTrajectoryDataReader(0, "rt/planning/ADCTrajectory")
        obstacle_list_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/fusion/motion_manager/MMObstacleList")
        odometry_sub = fastdds_odometry.OdometryDataReader(0, "rt/localization/local_map_location/Map2LocalTF")
        traffic_events_sub = fastdds_traffic_events.TrafficEventsDataReader(0, "rt/routing/TrafficEventsLocal")
        routing_response_sub = fastdds_routing_response.RoutingResponseDataReader(0, "rt/local_map/LocalRoutingResponse")
        local_map_sub = fastdds_local_map.LocalMapDataReader(0, "rt/prediction/LocalMap")
        lane_list_sub = fastdds_lane_list.LaneListDataReader(0, "rtMFLaneList")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()
    location = fastdds_location.Location()
    adc_trajectory = fastdds_adc_trajectory.ADCTrajectory()
    obstacle_list = fastdds_obstacle_list.ObstacleList()
    odometry = fastdds_odometry.Odometry()
    traffic_events = fastdds_traffic_events.TrafficEvents()
    routing_response = fastdds_routing_response.RoutingResponse()
    local_map = fastdds_local_map.LocalMap()
    lane_list = fastdds_lane_list.LaneList()

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
            if (location_sub.wait_for_sample(2)):
                # Read the received message
                if (location_sub.take_sample(location)):
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
            if (obstacle_list_sub.wait_for_sample(2)):
                # Read the received message
                if (obstacle_list_sub.take_sample(obstacle_list)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (odometry_sub.wait_for_sample(2)):
                # Read the received message
                if (odometry_sub.take_sample(odometry)):
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
            if (local_map_sub.wait_for_sample(2)):
                # Read the received message
                if (local_map_sub.take_sample(local_map)):
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


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
