#include "sensors/LidarFactory.hpp"

#include "sensors/VelodyneLidar.hpp"

#include <algorithm>
#include <ranges>
#include <cctype>

namespace lidar
{

std::unique_ptr<BaseLidarSensor> LidarFactory::createSensor(const std::string& type, const std::string& sourcePath)
{
    std::string lowerType = type;
    std::ranges::transform(lowerType.begin(), lowerType.end(), lowerType.begin(), ::tolower);

    if (sourcePath.empty())
    {
        return nullptr;
    }

    if (lowerType == "velodyne" || lowerType == "velodyne_hdl")
    {
        return std::make_unique<VelodyneLidar>("Velodyne HDL-32E", sourcePath);
    }

    if (lowerType == "velodyne_vlp")
    {
        auto sensor = std::make_unique<VelodyneLidar>("Velodyne VLP-16", sourcePath);
        sensor->configure(30.0F, 120.0F);
        return sensor;
    }

    return nullptr;
}

} // namespace lidar
