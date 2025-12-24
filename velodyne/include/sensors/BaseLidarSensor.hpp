#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lidar
{

struct LidarPoint
{
    float x;
    float y;
    float z;
    float intensity;
};

class BaseLidarSensor
{
public:
    using PointCloud = std::vector<LidarPoint>;

    virtual ~BaseLidarSensor() = default;

    /// Identifier for the sensor (Velodyne, SolidState, etc.)
    virtual const std::string& identifier() const noexcept = 0;

    /// Configure the sensor before a run.
    virtual void configure(float vertical_fov_deg, float max_range_m) = 0;

    /// The sensor pushes the next frame into the provided buffer.
    virtual bool readNextScan(PointCloud& destination, uint64_t& timestamp_us) = 0;
};

} // namespace lidar
