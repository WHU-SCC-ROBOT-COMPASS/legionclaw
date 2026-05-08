source /opt/ros/humble/setup.bash
source ~/project/legionclaw/modules/message/ros2/install/setup.bash 

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="/home/BEACON/project/driver/lidar_launch/config/fastdds_hybrid.xml"
echo "[DDS] 使用 ros_cyclonedds.xml"

export LD_LIBRARY_PATH="~/project/legionclaw/modules/perception/lidar/lidar_cluster_detect/bin:~/project/legionclaw/modules/perception/lidar/lidar_ground_segmentation/bin:~/project/legionclaw/modules/perception/lidar/multi_lidar_splicing/bin:~/project/legionclaw/modules/perception/fusion/motion_manager/bin:~/project/legionclaw/third_party/arm64/lib/opencv4:~/project/legionclaw/third_party/arm64/lib:/usr/local/legionclaw/third_party/arm64/lib/opencv4:/usr/local/legionclaw/third_party/arm64/lib:${LD_LIBRARY_PATH:-}"

cd bin
./lidar_cluster_detect