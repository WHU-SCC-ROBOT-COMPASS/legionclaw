import argparse
import time
import sys

import fastdds_routing_request
import fastdds_traffic_events
import fastdds_obu_cmd_msg

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
        routing_request_pub = fastdds_routing_request.RoutingRequestDataWriter(0, "rt/vui_client/RoutingRequest")
        obu_cmd_msg_pub = fastdds_obu_cmd_msg.ObuCmdMsgDataWriter(0, "rt/vui_client/ObuCmdMsg")
        traffic_events_pub = fastdds_traffic_events.TrafficEventsDataWriter(0, "rt/vui_client/SpeedLimit")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    routing_request = fastdds_routing_request.RoutingRequest()
    obu_cmd_msg = fastdds_obu_cmd_msg.ObuCmdMsg()
    traffic_events = fastdds_traffic_events.TrafficEvents()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            routing_request_pub.write_sample(routing_request)
            obu_cmd_msg_pub.write_sample(obu_cmd_msg)
            traffic_events_pub.write_sample(traffic_events)
            print("success")
            #pub1.write_sample(msg1)
            time.sleep(1)
    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')

# Loop for the subscriber
else:

    # Create a publisher on the topic
    try:
        pass

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    pass

    try:
        while True:
            pass


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
