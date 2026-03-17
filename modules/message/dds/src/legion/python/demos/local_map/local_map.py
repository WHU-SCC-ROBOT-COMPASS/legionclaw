import argparse
import time
import sys

import fastdds_odometry
import fastdds_location
import fastdds_free_space
import fastdds_events
import fastdds_lane_list
import fastdds_obu_cmd_msg
import fastdds_faults
import fastdds_chassis
import fastdds_road_mark_list
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
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/local_map/Faults")
        events_pub = fastdds_events.EventsDataWriter(0, "rt/local_map/Events")
        lane_list_output_pub = fastdds_lane_list.LaneListDataWriter(0, "rt/local_map/LaneList")
        road_mark_list_pub = fastdds_road_mark_list.RoadMarkListDataWriter(0, "rt/local_map/RoadMarkList")
        free_space_pub = fastdds_free_space.FreeSpaceDataWriter(0, "rt/local_map/FreeSpace")
        routing_response_output_pub = fastdds_routing_response.RoutingResponseDataWriter(0, "rt/local_map/LocalRoutingResponse")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    faults = fastdds_faults.Faults()
    events = fastdds_events.Events()
    lane_list_output = fastdds_lane_list.LaneList()
    road_mark_list = fastdds_road_mark_list.RoadMarkList()
    free_space = fastdds_free_space.FreeSpace()
    routing_response_output = fastdds_routing_response.RoutingResponse()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            faults_pub.write_sample(faults)
            events_pub.write_sample(events)
            lane_list_output_pub.write_sample(lane_list_output)
            road_mark_list_pub.write_sample(road_mark_list)
            free_space_pub.write_sample(free_space)
            routing_response_output_pub.write_sample(routing_response_output)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        routing_response_input_sub = fastdds_routing_response.RoutingResponseDataReader(0, "rt/routing/RoutingResponse")
        location_sub = fastdds_location.LocationDataReader(0, "rt/localization/global_fusion/Location")
        traffic_events_sub = fastdds_traffic_events.TrafficEventsDataReader(0, "rt/routing/TrafficEvents")
        chassis_sub = fastdds_chassis.ChassisDataReader(0, "rt/drivers/canbus/Chassis")
        lane_list_input_sub = fastdds_lane_list.LaneListDataReader(0, "rt/perception/fusion/traffic_sign_fusion/MFLaneList")
        obstacle_list_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/fusion/motion_manager/MMObstacleList")
        odometry_sub = fastdds_odometry.OdometryDataReader(0, "rt/localization/local_map_location/Map2LocalTF")
        obu_cmd_msg_sub = fastdds_obu_cmd_msg.ObuCmdMsgDataReader(0, "rt/vui_client/ObuCmdMsg")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    routing_response_input = fastdds_routing_response.RoutingResponse()
    location = fastdds_location.Location()
    traffic_events = fastdds_traffic_events.TrafficEvents()
    chassis = fastdds_chassis.Chassis()
    lane_list_input = fastdds_lane_list.LaneList()
    obstacle_list = fastdds_obstacle_list.ObstacleList()
    odometry = fastdds_odometry.Odometry()
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()

    try:
        while True:
            # Wait for a message for 2 seconds
            if (routing_response_input_sub.wait_for_sample(2)):
                # Read the received message
                if (routing_response_input_sub.take_sample(routing_response_input)):
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
            if (traffic_events_sub.wait_for_sample(2)):
                # Read the received message
                if (traffic_events_sub.take_sample(traffic_events)):
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
            if (lane_list_input_sub.wait_for_sample(2)):
                # Read the received message
                if (lane_list_input_sub.take_sample(lane_list_input)):
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
            if (obu_cmd_msg_sub.wait_for_sample(2)):
                # Read the received message
                if (obu_cmd_msg_sub.take_sample(obu_cmd_msg)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
