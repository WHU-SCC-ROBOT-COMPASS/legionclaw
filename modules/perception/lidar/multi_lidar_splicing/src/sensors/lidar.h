#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>

class Lidar
{
private:
    std::string channel;
    std::string frame_id;
    Eigen::Matrix4f extrinsic;

public:
    explicit Lidar(std::string intrinsic_and_extrinsic_json_path);
    ~Lidar();

    void setFrameId(std::string frame_id);

    std::string getChannel() const;
    std::string getFrameId() const;
    Eigen::Matrix4f getExtrinsic() const;
};
