#pragma once

#include "sensors/BaseLidarSensor.hpp"
#include "visualization/IVisualizer.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

namespace lidar
{

class LidarEngine
{
public:
    explicit LidarEngine(std::unique_ptr<BaseLidarSensor> sensor,
                         std::unique_ptr<visualization::IVisualizer> visualizer = nullptr);

    bool initialize();
    void run();

    uint64_t latestTimestamp() const { return m_latestTimestamp; }

private:
    friend struct LidarEngineTestHelper;
    void captureFrame();

    static constexpr std::chrono::milliseconds kTargetFrameDuration{33};

    std::unique_ptr<BaseLidarSensor> m_sensor;
    std::unique_ptr<visualization::IVisualizer> m_visualizer;
    std::array<BaseLidarSensor::PointCloud, 2> m_pointBuffers;
    size_t m_readIndex;
    uint64_t m_latestTimestamp;
};

} // namespace lidar
