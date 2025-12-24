#pragma once

#include "sensors/BaseLidarSensor.hpp"
#include "visualization/Visualizer.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

namespace lidar
{

class LidarEngine
{
public:
    explicit LidarEngine(std::unique_ptr<BaseLidarSensor> sensor);

    bool initialize();
    void run();

private:
    void captureFrame();

    static constexpr std::chrono::milliseconds kTargetFrameDuration{33};

    std::unique_ptr<BaseLidarSensor> m_sensor;
    visualization::Visualizer m_visualizer;
    std::array<BaseLidarSensor::PointCloud, 2> m_pointBuffers;
    size_t m_readIndex;
    uint64_t m_latestTimestamp;
};

} // namespace lidar
