cmake_minimum_required(VERSION 3.5)
project(ros2_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
#   message_generation
#   std_msgs
# )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NaviInfoMsg.msg"
  "msg/LaneInfoExtend.msg"
  "msg/LocalMap.msg"
  "msg/Header.msg"
  "msg/Time.msg"
  "msg/Status.msg"
  "msg/PointENU.msg"
  "msg/PointBasic.msg"
  "msg/PointLLH.msg"
  "msg/Point2D.msg"
  "msg/Point3D.msg"
  "msg/Quaternion.msg"
  "msg/Polygon3D.msg"
  "msg/Polygon2D.msg"
  "msg/ImageKeyPoint.msg"
  "msg/SLPoint.msg"
  "msg/SLBoundary.msg"
  "msg/FrenetFramePoint.msg"
  "msg/SpeedPoint.msg"
  "msg/PathPoint.msg"
  "msg/Path.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  "msg/VehicleMotionPoint.msg"
  "msg/VehicleMotion.msg"
  "msg/GaussianInfo.msg"
  "msg/VehicleSignal.msg"
  "msg/PadMessage.msg"
  "msg/Stories.msg"
  "msg/Fault.msg"
  "msg/Faults.msg"
  "msg/Event.msg"
  "msg/Events.msg"
  "msg/EStop.msg"
  "msg/Matrix3D.msg"
  "msg/Pose.msg"
  "msg/Uncertainty.msg"
  "msg/Chassis.msg"
  "msg/ControlCommand.msg"
  "msg/WheelInfo.msg"
  "msg/ControlAnalysis.msg"
  "msg/VehicleParam.msg"
  "msg/VehicleConfig.msg"
  "msg/PredictionObstacles.msg"
  "msg/Location.msg"
  "msg/Odometry.msg"
  "msg/ObstacleList.msg"
  "msg/LaneList.msg"
  "msg/TrafficLightMsg.msg"
  "msg/Ultrasonic.msg"
  "msg/RadarObstacleListMsg.msg"
  "msg/PointCloud.msg"
  "msg/RadarObstacle.msg"
  "msg/UltrasonicObstacle.msg"
  "msg/PointXYZIRT.msg"
  "msg/CompressedImage.msg"
  "msg/Image.msg"
  "msg/Gnss.msg"
  "msg/Imu.msg"
  "msg/Ins.msg"
  "msg/ObuCmdMsg.msg"
  "msg/RoutingRequest.msg"
  "msg/ObuCmd.msg"
  "msg/ParkingInfo.msg"
  "msg/KeyPoint.msg"
  "msg/SensorCalibrator.msg"
  "msg/TrafficLight.msg"
  "msg/StopPoint.msg"
  "msg/LanePoint.msg"
  "msg/LaneInfo.msg"
  "msg/ObstacleIntent.msg"
  "msg/RoutingResponse.msg"
  "msg/StopInfo.msg"
  "msg/ADCTrajectory.msg"
  "msg/BBox2D.msg"
  "msg/Obstacle.msg"
  "msg/RSSInfo.msg"
  "msg/ObstaclePriority.msg"
  "msg/PlanningCmd.msg"
  "msg/PredictionObstacle.msg"
  "msg/TrajectoryInPrediction.msg"
  "msg/TrajectoryPointInPrediction.msg"
  "msg/PredictionTrajectoryPoint.msg"
  "msg/PerceptionObstacle.msg"
  "msg/ObstacleFeature.msg"
  "msg/LaneLine.msg"
  "msg/LaneLineCubicCurve.msg"
  "msg/EndPoints.msg"
  "msg/FreeSpace.msg"
  "msg/HolisticPathPrediction.msg"
  "msg/RoadMark.msg"
  "msg/TrafficLightDebug.msg"
  "msg/TrafficLightBox.msg"
  "msg/ImageRect.msg"
  "msg/RadarState.msg"
  "msg/RadarStateError.msg"
  "msg/RadarStateMode.msg"
  "msg/SotifMonitorResult.msg"
  "msg/Region.msg"
  "msg/Grid.msg"
  "msg/CameraParkingInfo.msg"
  "msg/CameraParkingInfoList.msg"
  "msg/SecurityDecision.msg"
  "msg/WarningCommand.msg"
  "msg/TrajectoryLimitCommand.msg"
  "msg/ParkingInfoList.msg"
  "msg/CommCommand.msg"
  "msg/CommandRespond.msg"
  "msg/ModuleStatus.msg"
  "msg/Command.msg"
  "msg/Message.msg"
  "msg/PlanningAnalysis.msg"
  "msg/TimeConsume.msg"
  "msg/PlanningParkingDebug.msg"
  "msg/TrajectoryArray.msg"
  "msg/VehicleState.msg"
  "msg/DrivableRegion.msg"
  "msg/ObstacleInteractiveTag.msg"
  "msg/Twist.msg"
  "msg/ParkingStateDisplay.msg"
  "msg/ParkingRoi.msg"
  "msg/Point2dList.msg"
  "msg/ParkingOutInfo.msg"
  "msg/JunctionInfo.msg"
  "msg/TrafficEvents.msg"
  "msg/LimitSpeedInfo.msg"
  "msg/Log.msg"
  "msg/Pavementype.msg"
  "msg/WLConstraintInfoList.msg"
  "msg/WLConstraintInfo.msg"
  "msg/UssObstacleList.msg"
  "msg/UssObstacle.msg"
  "msg/UssParkingInfoList.msg"
  "msg/UssParkingInfo.msg"
  "msg/CameraParkingStopper.msg"
  "msg/ParkingStopper.msg"
  "msg/DiagnosticArray.msg"
  "msg/DiagnosticStatus.msg"
  "msg/KeyValues.msg"
  "msg/AlarmMessage.msg"
  "msg/LaneletInfo.msg"
  "msg/GlobalRouteMsg.msg"
  "msg/RouteFusionInfo.msg"
  "msg/RoadMarkList.msg"
  "msg/HMIObuCmd.msg"
  "msg/HMIObuCmdMsg.msg"
  "msg/HMIParkingStateDisplay.msg"
  "msg/HMIParkingInfo.msg"
  "msg/HMIParkingInfoList.msg"
  "msg/HMIVehicleMsg.msg"
  "msg/HMIObstacle.msg"
  "msg/HMIObstacleList.msg"
  "msg/HMITrajectoryPoint.msg"
  "msg/HMITrajectory.msg"
  "msg/HMIDiagnosticStatus.msg"
  "msg/HMIDiagnosticArray.msg"
  "msg/GuideInfo.msg"
  "msg/GuideRoad.msg"
  "msg/CurvatureInfo.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)
# install(FILES ros22ros1.yaml
#  DESTINATION share/${PROJECT_NAME}
# )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
