import argparse
import time
import sys

import fastdds_stop_info
import fastdds_traffic_light_msg
import fastdds_odometry
import fastdds_global_route_msg
import fastdds_location
import fastdds_polygon_2d
import fastdds_routing_request
import fastdds_guide_info
import fastdds_parking_info
import fastdds_obu_cmd_msg
import fastdds_faults
import fastdds_chassis
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
        routing_response_pub = fastdds_routing_response.RoutingResponseDataWriter(0, "rt/routing/RoutingResponse")
        parking_info_pub = fastdds_parking_info.ParkingInfoDataWriter(0, "rt/routing/ParkingInfo")
        stop_info_pub = fastdds_stop_info.StopInfoDataWriter(0, "rt/routing/StopInfo")
        global_route_msg_pub = fastdds_global_route_msg.GlobalRouteMsgDataWriter(0, "rt/routing/GlobalRouteMsg")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/routing/Faults")
        polygon_2d_pub = fastdds_polygon_2d.Polygon2DDataWriter(0, "rt/routing/MapBoundary")
        traffic_events_output_pub = fastdds_traffic_events.TrafficEventsDataWriter(0, "rt/routing/TrafficEvents")
        guide_info_pub = fastdds_guide_info.GuideInfoDataWriter(0, "rt/routing/GuideInfo")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    routing_response = fastdds_routing_response.RoutingResponse()
    parking_info = fastdds_parking_info.ParkingInfo()
    stop_info = fastdds_stop_info.StopInfo()
    global_route_msg = fastdds_global_route_msg.GlobalRouteMsg()
    faults = fastdds_faults.Faults()
    polygon_2d = fastdds_polygon_2d.Polygon2D()
    traffic_events_output = fastdds_traffic_events.TrafficEvents()
    guide_info = fastdds_guide_info.GuideInfo()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            routing_response_pub.write_sample(routing_response)
            parking_info_pub.write_sample(parking_info)
            stop_info_pub.write_sample(stop_info)
            global_route_msg_pub.write_sample(global_route_msg)
            faults_pub.write_sample(faults)
            polygon_2d_pub.write_sample(polygon_2d)
            traffic_events_output_pub.write_sample(traffic_events_output)
            guide_info_pub.write_sample(guide_info)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        traffic_light_msg_sub = fastdds_traffic_light_msg.TrafficLightMsgDataReader(0, "rt/perception/fusion/traffic_sign_fusion/TrafficLightMsg")
        location_sub = fastdds_location.LocationDataReader(0, "rt/localization/global_fusion/Location")
        chassis_sub = fastdds_chassis.ChassisDataReader(0, "rt/drivers/canbus/Chassis")
        obu_cmd_msg_sub = fastdds_obu_cmd_msg.ObuCmdMsgDataReader(0, "rt/vui_client/ObuCmdMsg")
        traffic_events_input_sub = fastdds_traffic_events.TrafficEventsDataReader(0, "rt/vui_client/SpeedLimit")
        odometry_sub = fastdds_odometry.OdometryDataReader(0, "rt/localization/local_map_location/Map2LocalTF")
        routing_request_sub = fastdds_routing_request.RoutingRequestDataReader(0, "rt/vui_client/RoutingRequest")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    traffic_light_msg = fastdds_traffic_light_msg.TrafficLightMsg()
    location = fastdds_location.Location()
    chassis = fastdds_chassis.Chassis()
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()
    traffic_events_input = fastdds_traffic_events.TrafficEvents()
    odometry = fastdds_odometry.Odometry()
    routing_request = fastdds_routing_request.RoutingRequest()

    try:
        while True:
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
            if (chassis_sub.wait_for_sample(2)):
                # Read the received message
                if (chassis_sub.take_sample(chassis)):
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
            if (traffic_events_input_sub.wait_for_sample(2)):
                # Read the received message
                if (traffic_events_input_sub.take_sample(traffic_events_input)):
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
            if (routing_request_sub.wait_for_sample(2)):
                # Read the received message
                if (routing_request_sub.take_sample(routing_request)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
