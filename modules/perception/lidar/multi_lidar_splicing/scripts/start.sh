source /opt/ros/humble/setup.bash
source ~/project/legionclaw/modules/message/ros2/install/setup.bash 

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="/home/BEACON/project/driver/lidar_launch/config/fastdds_hybrid.xml"
echo "[DDS] 使用 ros_cyclonedds.xml"

cd bin
./multi_lidar_splicing