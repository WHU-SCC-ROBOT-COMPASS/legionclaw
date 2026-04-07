#include "multi_lidar_splicing.h"
#include <jsoncpp/json/json.h>
#include <algorithm>

static bool loadConfig(const std::string &filename,
                std::vector<LidarConfig> &lidars,
                std::string &frame_id,
                std::string &publish_topic,
                FilterRegion &filter_region)
{
    Json::Reader reader;
    Json::Value root;

    std::ifstream is(filename, std::ios::binary);
    if (!is.is_open())
    {
        std::cout << "Error opening file:" << filename << std::endl;
        return false;
    }

    if (!reader.parse(is, root))
    {
        std::cout << "Error parsing JSON:" << filename << std::endl;
        is.close();
        return false;
    }

    // Required fields
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

    frame_id = root["frame_id"].asString();
    publish_topic = root["publish_topic"].asString();

    // Optional lidar configurations
    static const char* lidar_names[] = {"lidar_front", "lidar_mid", "lidar_left", "lidar_right", "lidar_back"};

    for (const auto& name : lidar_names) {
        if (!root["calibration_params_path"].isNull() &&
            !root["calibration_params_path"][name].isNull() &&
            root["calibration_params_path"][name].type() == Json::stringValue)
        {
            LidarConfig cfg;
            cfg.name = name;
            cfg.path = root["calibration_params_path"][name].asString();
            cfg.enabled = true;
            lidars.push_back(cfg);
        }
    }

    if (lidars.empty())
    {
        std::cout << "Error: no lidar configured, at least one lidar is required" << std::endl;
        is.close();
        return false;
    }

    // Filter region configuration
    if (!root["filter_region"].isNull())
    {
        const Json::Value &fr = root["filter_region"];
        filter_region.enable = fr["enable"].asBool();
        filter_region.x_min = static_cast<float>(fr["x_min"].asDouble());
        filter_region.x_max = static_cast<float>(fr["x_max"].asDouble());
        filter_region.y_min = static_cast<float>(fr["y_min"].asDouble());
        filter_region.y_max = static_cast<float>(fr["y_max"].asDouble());
        filter_region.z_min = static_cast<float>(fr["z_min"].asDouble());
        filter_region.z_max = static_cast<float>(fr["z_max"].asDouble());
    }

    is.close();
    return true;
}

int main(int argc, char *argv[])
{
    std::string file_path = "./conf/config.json";
    std::vector<LidarConfig> lidar_configs;
    std::string frame_id, publish_topic;
    FilterRegion filter_region;

    if (!loadConfig(file_path, lidar_configs, frame_id, publish_topic, filter_region))
        return 1;

    std::cout << "[MultiLidarSplicing] Configured lidars: ";
    for (size_t i = 0; i < lidar_configs.size(); ++i) {
        std::cout << lidar_configs[i].name;
        if (i < lidar_configs.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    if (filter_region.enable) {
        std::cout << "[MultiLidarSplicing] Filter region: x[" << filter_region.x_min
                  << ", " << filter_region.x_max << "], y[" << filter_region.y_min
                  << ", " << filter_region.y_max << "], z[" << filter_region.z_min
                  << ", " << filter_region.z_max << "]" << std::endl;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiLidarSplicing>(lidar_configs, frame_id, publish_topic, filter_region);
    node->run();
    rclcpp::shutdown();
    return 0;
}