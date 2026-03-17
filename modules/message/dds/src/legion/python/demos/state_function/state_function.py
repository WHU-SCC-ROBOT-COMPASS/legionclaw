import argparse
import time
import sys

import fastdds_stop_info
import fastdds_location
import fastdds_security_decision
import fastdds_routing_request
import fastdds_events
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
        events_output_pub = fastdds_events.EventsDataWriter(0, "rt/state_function/Events")
        stop_info_output_pub = fastdds_stop_info.StopInfoDataWriter(0, "rt/state_function/StopInfo")
        routing_request_pub = fastdds_routing_request.RoutingRequestDataWriter(0, "rt/state_function/RoutingRequest")
        obu_cmd_msg_pub = fastdds_obu_cmd_msg.ObuCmdMsgDataWriter(0, "rt/state_function/ObuCmdMsg")
        traffic_events_pub = fastdds_traffic_events.TrafficEventsDataWriter(0, "rt/state_function/SpeedLimit")
        parking_info_pub = fastdds_parking_info.ParkingInfoDataWriter(0, "rt/state_function/ParkingInfo")
        faults_output_pub = fastdds_faults.FaultsDataWriter(0, "rt/state_function/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    events_output = fastdds_events.Events()
    stop_info_output = fastdds_stop_info.StopInfo()
    routing_request = fastdds_routing_request.RoutingRequest()
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()
    traffic_events = fastdds_traffic_events.TrafficEvents()
    parking_info = fastdds_parking_info.ParkingInfo()
    faults_output = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            events_output_pub.write_sample(events_output)
            stop_info_output_pub.write_sample(stop_info_output)
            routing_request_pub.write_sample(routing_request)
            obu_cmd_msg_pub.write_sample(obu_cmd_msg)
            traffic_events_pub.write_sample(traffic_events)
            parking_info_pub.write_sample(parking_info)
            faults_output_pub.write_sample(faults_output)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        chassis_sub = fastdds_chassis.ChassisDataReader(0, "rt/drivers/canbus/Chassis")
        stop_info_input_sub = fastdds_stop_info.StopInfoDataReader(0, "rt/routing/StopInfo")
        events_input_sub = fastdds_events.EventsDataReader(0, "rt/planning/Events")
        container_sub = fastdds_events.EventsDataReader(0, "rt/state_function/container")
        location_sub = fastdds_location.LocationDataReader(0, "rt/localization/global_fusion/Location")
        routing_response_sub = fastdds_routing_response.RoutingResponseDataReader(0, "rt/routing/RoutingResponse")
        faults_input_sub = fastdds_faults.FaultsDataReader(0, "rt/safety/safeguard/Faults")
        security_decision_sub = fastdds_security_decision.SecurityDecisionDataReader(0, "rt/safety/safeguard/SecurityDecision")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    chassis = fastdds_chassis.Chassis()
    stop_info_input = fastdds_stop_info.StopInfo()
    events_input = fastdds_events.Events()
    container = fastdds_events.Events()
    location = fastdds_location.Location()
    routing_response = fastdds_routing_response.RoutingResponse()
    faults_input = fastdds_faults.Faults()
    security_decision = fastdds_security_decision.SecurityDecision()

    try:
        while True:
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
            if (stop_info_input_sub.wait_for_sample(2)):
                # Read the received message
                if (stop_info_input_sub.take_sample(stop_info_input)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (events_input_sub.wait_for_sample(2)):
                # Read the received message
                if (events_input_sub.take_sample(events_input)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')
            # Wait for a message for 2 seconds
            if (container_sub.wait_for_sample(2)):
                # Read the received message
                if (container_sub.take_sample(container)):
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
            if (routing_response_sub.wait_for_sample(2)):
                # Read the received message
                if (routing_response_sub.take_sample(routing_response)):
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
            if (security_decision_sub.wait_for_sample(2)):
                # Read the received message
                if (security_decision_sub.take_sample(security_decision)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
