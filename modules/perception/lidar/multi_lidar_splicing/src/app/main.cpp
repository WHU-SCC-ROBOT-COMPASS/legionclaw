#include "multi_lidar_splicing.h"
#include <jsoncpp/json/json.h>

static bool loadConfig(const std::string &filename,
                std::string &lidar_front,
                std::string &lidar_mid,
                std::string &lidar_left,
                std::string &lidar_right,
                std::string &lidar_back,
                std::string &frame_id,
                std::string &publish_topic)
{
    Json::Reader reader;
    Json::Value root;

    std::ifstream is(filename, std::ios::binary);
    if (!is.is_open())
    {
        std::cout << "Error opening file:" << filename << std::endl;
        return false;
    }

    if (reader.parse(is, root))
    {
        if (root["calibration_params_path"].isNull() || root["calibration_params_path"].type() != Json::objectValue ||
            root["calibration_params_path"]["lidar_front"].isNull() || root["calibration_params_path"]["lidar_front"].type() != Json::stringValue ||
            root["calibration_params_path"]["lidar_mid"].isNull()  || root["calibration_params_path"]["lidar_mid"].type()  != Json::stringValue ||
            root["calibration_params_path"]["lidar_left"].isNull()  || root["calibration_params_path"]["lidar_left"].type()  != Json::stringValue ||
            root["calibration_params_path"]["lidar_right"].isNull() || root["calibration_params_path"]["lidar_right"].type() != Json::stringValue ||
            root["calibration_params_path"]["lidar_back"].isNull()  || root["calibration_params_path"]["lidar_back"].type()  != Json::stringValue)
        {
            std::cout << "Error calibration_params_path type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["frame_id"].isNull() || root["frame_id"].type() != Json::stringValue)
        {
            std::cout << "Error frame_id type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["publish_topic"].isNull() || root["publish_topic"].type() != Json::stringValue)
        {
            std::cout << "Error publish_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        lidar_front   = root["calibration_params_path"]["lidar_front"].asString();
        lidar_mid    = root["calibration_params_path"]["lidar_mid"].asString();
        lidar_left    = root["calibration_params_path"]["lidar_left"].asString();
        lidar_right   = root["calibration_params_path"]["lidar_right"].asString();
        lidar_back    = root["calibration_params_path"]["lidar_back"].asString();
        frame_id      = root["frame_id"].asString();
        publish_topic = root["publish_topic"].asString();
    }

    is.close();
    return true;
}

int main(int argc, char *argv[])
{
    std::string file_path =
      "./conf/config.json";
    std::string lidar_front, lidar_mid, lidar_left, lidar_right,  lidar_back, frame_id, publish_topic;
    if (!loadConfig(file_path, lidar_front, lidar_mid, lidar_left, lidar_right, lidar_back, frame_id, publish_topic))
        return 1;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiLidarSplicing>(
        lidar_front, lidar_mid, lidar_left, lidar_right, lidar_back, frame_id, publish_topic);
    node->run();
    rclcpp::shutdown();
    return 0;
}
