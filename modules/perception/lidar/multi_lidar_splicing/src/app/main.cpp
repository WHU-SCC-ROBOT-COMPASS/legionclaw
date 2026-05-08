#include "multi_lidar_splicing.h"
#include <jsoncpp/json/json.h>
#include <algorithm>

static bool loadConfig(const std::string &filename,
                std::vector<LidarConfig> &lidars,
                std::string &frame_id,
                std::string &publish_topic,
                FilterRegion &filter_region,
                FilterRegion &filter_region_outside,
                SyncConfig &sync_config)
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

    // Filter region configuration (inside filter)
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

    // Filter region outside configuration
    if (!root["filter_region_outside"].isNull())
    {
        const Json::Value &fro = root["filter_region_outside"];
        filter_region_outside.enable = fro["enable"].asBool();
        filter_region_outside.x_min = static_cast<float>(fro["x_min"].asDouble());
        filter_region_outside.x_max = static_cast<float>(fro["x_max"].asDouble());
        filter_region_outside.y_min = static_cast<float>(fro["y_min"].asDouble());
        filter_region_outside.y_max = static_cast<float>(fro["y_max"].asDouble());
        filter_region_outside.z_min = static_cast<float>(fro["z_min"].asDouble());
        filter_region_outside.z_max = static_cast<float>(fro["z_max"].asDouble());
    }

    // Sync configuration
    if (!root["sync_config"].isNull())
    {
        const Json::Value &sc = root["sync_config"];
        sync_config.enable_approximate_time_sync = sc["enable_approximate_time_sync"].asBool();
        sync_config.sync_time_tolerance = sc["sync_time_tolerance"].asDouble();
        sync_config.min_lidars_for_fusion = sc["min_lidars_for_fusion"].asInt();
        sync_config.message_timeout = sc["message_timeout"].asDouble();
        sync_config.strict_sync = sc["strict_sync"].asBool();
    }
    else
    {
        sync_config.enable_approximate_time_sync = true;
        sync_config.sync_time_tolerance = 0.1;
        sync_config.min_lidars_for_fusion = 1;
        sync_config.message_timeout = 1.0;
        sync_config.strict_sync = false;
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
    FilterRegion filter_region_outside;
    SyncConfig sync_config;

    if (!loadConfig(file_path, lidar_configs, frame_id, publish_topic, filter_region, filter_region_outside, sync_config))
        return 1;

    std::cout << "[MultiLidarSplicing] Configured lidars: ";
    for (size_t i = 0; i < lidar_configs.size(); ++i) {
        std::cout << lidar_configs[i].name;
        if (i < lidar_configs.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    if (filter_region.enable) {
        std::cout << "[MultiLidarSplicing] Filter region (inside): x[" << filter_region.x_min
                  << ", " << filter_region.x_max << "], y[" << filter_region.y_min
                  << ", " << filter_region.y_max << "], z[" << filter_region.z_min
                  << ", " << filter_region.z_max << "]" << std::endl;
    }

    if (filter_region_outside.enable) {
        std::cout << "[MultiLidarSplicing] Filter region (outside): x[" << filter_region_outside.x_min
                  << ", " << filter_region_outside.x_max << "], y[" << filter_region_outside.y_min
                  << ", " << filter_region_outside.y_max << "], z[" << filter_region_outside.z_min
                  << ", " << filter_region_outside.z_max << "]" << std::endl;
    }

    std::cout << "[MultiLidarSplicing] Sync config: approximate_time=" 
              << (sync_config.enable_approximate_time_sync ? "true" : "false")
              << ", tolerance=" << sync_config.sync_time_tolerance << "s"
              << ", min_lidars=" << sync_config.min_lidars_for_fusion
              << ", timeout=" << sync_config.message_timeout << "s"
              << ", strict=" << (sync_config.strict_sync ? "true" : "false") << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiLidarSplicing>(lidar_configs, frame_id, publish_topic, filter_region, filter_region_outside, sync_config);
    node->run();
    rclcpp::shutdown();
    return 0;
}