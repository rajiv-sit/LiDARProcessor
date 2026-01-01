#pragma once

#include "sensors/BaseLidarSensor.hpp"

namespace visualization
{
class IVisualizer
{
public:
    virtual ~IVisualizer() = default;

    virtual bool initialize() = 0;
    virtual void updatePoints(const lidar::BaseLidarSensor::PointCloud& points) = 0;
    virtual void render() = 0;
    virtual bool windowShouldClose() const = 0;
    virtual float frameSpeedScale() const = 0;
};

} // namespace visualization
