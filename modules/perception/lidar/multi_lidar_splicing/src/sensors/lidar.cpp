#include "lidar.h"

Lidar::Lidar(std::string intrinsic_and_extrinsic_json_path)
{
    Json::Reader reader;
    Json::Value root;
    std::vector<float> rotation, translation;

    std::ifstream is(intrinsic_and_extrinsic_json_path, std::ios::binary);
    if (!is.is_open())
    {
        std::cout << "Error opening file:" << intrinsic_and_extrinsic_json_path << std::endl;
    }
    else
    {
        if (reader.parse(is, root))
        {
            if (!root["channel"].isNull() && root["channel"].type() == Json::stringValue)
            {
                this->channel = root["channel"].asString();
            }
            else
            {
                std::cout << "Error channel type:" << intrinsic_and_extrinsic_json_path << std::endl;
            }
        }

        // rotation: 支持 [9] 或 [3][3]
        if (root["rotation"].isNull() || root["rotation"].type() != Json::arrayValue)
        {
            std::cout << "Error rotation type:" << intrinsic_and_extrinsic_json_path << std::endl;
        }
        if (root["rotation"].size() == 3)
        {
            for (unsigned int i = 0; i < root["rotation"].size(); i++)
            {
                if (root["rotation"][i].isNull() || root["rotation"][i].type() != Json::arrayValue)
                {
                    std::cout << "Error rotation type:" << intrinsic_and_extrinsic_json_path << ":" << i << std::endl;
                }
                if (root["rotation"][i].size() != 3)
                {
                    std::cout << "Error rotation size:" << intrinsic_and_extrinsic_json_path << ":" << i << std::endl;
                }

                for (unsigned int j = 0; j < root["rotation"][i].size(); j++)
                {
                    float data = root["rotation"][i][j].asFloat();
                    rotation.push_back(data);
                }
            }
        }
        else if (root["rotation"].size() == 9)
        {
            for (unsigned int i = 0; i < root["rotation"].size(); i++)
            {
                float data = root["rotation"][i].asFloat();
                rotation.push_back(data);
            }
        }
        else
        {
            std::cout << "Error rotation size:" << intrinsic_and_extrinsic_json_path << std::endl;
        }

        // translation: 支持 [3] 或 {x,y,z}
        if (!root["translation"].isNull() && root["translation"].type() == Json::arrayValue)
        {
            for (unsigned int i = 0; i < root["translation"].size(); i++)
            {
                float data = root["translation"][i].asFloat();
                translation.push_back(data);
            }
        }
        else if (!root["translation"].isNull() && root["translation"].type() == Json::objectValue)
        {
            float x = root["translation"]["x"].asFloat();
            float y = root["translation"]["y"].asFloat();
            float z = root["translation"]["z"].asFloat();
            translation = {x,y,z};
        }
        else
        {
            std::cout << "Error translation type:" << intrinsic_and_extrinsic_json_path << std::endl;
        }
        is.close();
    }

    this->extrinsic << rotation[0], rotation[1], rotation[2], translation[0],
                        rotation[3], rotation[4], rotation[5], translation[1],
                        rotation[6], rotation[7], rotation[8], translation[2],
                        0.0,        0.0,        0.0,        1.0;
}

Lidar::~Lidar() = default;

void Lidar::setFrameId(std::string frame_id) { this->frame_id = frame_id; }
std::string Lidar::getChannel() const { return this->channel; }
std::string Lidar::getFrameId() const { return this->frame_id; }
Eigen::Matrix4f Lidar::getExtrinsic() const { return this->extrinsic; }
