#pragma once

#include "sensors/BaseLidarSensor.hpp"

#include <memory>
#include <string>

namespace lidar
{

class LidarFactory
{
public:
    static std::unique_ptr<BaseLidarSensor> createSensor(const std::string& type, const std::string& sourcePath);
};

} // namespace lidar
