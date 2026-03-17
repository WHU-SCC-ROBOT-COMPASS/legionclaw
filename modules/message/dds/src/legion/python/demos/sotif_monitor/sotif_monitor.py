import argparse
import time
import sys

import fastdds_prediction_obstacles
import fastdds_location
import fastdds_sotif_monitor_result
import fastdds_chassis
import fastdds_faults
import fastdds_obstacle_list
import fastdds_routing_response

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
        sotif_monitor_result_pub = fastdds_sotif_monitor_result.SotifMonitorResultDataWriter(0, "rt/safety/sotif_monitor/SotifMonitorResult")
        faults_pub = fastdds_faults.FaultsDataWriter(0, "rt/safety/sotif_monitor/Faults")
    except Exception as e:
        print('Error creating publisher: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to send
    sotif_monitor_result = fastdds_sotif_monitor_result.SotifMonitorResult()
    faults = fastdds_faults.Faults()
   

    try:
        counter = 0
        while True:
            # Set the index in the message
            counter = counter + 1
            # publish
            sotif_monitor_result_pub.write_sample(sotif_monitor_result)
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
        location_sub = fastdds_location.LocationDataReader(0, "rt/localization/global_fusion/Location")
        chassis_sub = fastdds_chassis.ChassisDataReader(0, "rt/drivers/canbus/Chassis")
        prediction_obstacles_sub = fastdds_prediction_obstacles.PredictionObstaclesDataReader(0, "rt/prediction/PredictionObstacles")
        obstacle_list_sub = fastdds_obstacle_list.ObstacleListDataReader(0, "rt/perception/fusion/motion_manager/ObstacleList")
        routing_response_sub = fastdds_routing_response.RoutingResponseDataReader(0, "rt/routing/RoutingResponse")

    except Exception as e:
        print('Error creating subscriber: {error}'.format(
            error=e.__class__))
        sys.exit(-1)

    # Create the message to receive the data
    location = fastdds_location.Location()
    chassis = fastdds_chassis.Chassis()
    prediction_obstacles = fastdds_prediction_obstacles.PredictionObstacles()
    obstacle_list = fastdds_obstacle_list.ObstacleList()
    routing_response = fastdds_routing_response.RoutingResponse()

    try:
        while True:
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
            if (prediction_obstacles_sub.wait_for_sample(2)):
                # Read the received message
                if (prediction_obstacles_sub.take_sample(prediction_obstacles)):
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
            if (routing_response_sub.wait_for_sample(2)):
                # Read the received message
                if (routing_response_sub.take_sample(routing_response)):
                    print("success")
                       
                else:
                    print('Bad sample')
            else:
                print('No messages received in the last loop')


    except KeyboardInterrupt:
        print ('Shutdown requested...exiting')
