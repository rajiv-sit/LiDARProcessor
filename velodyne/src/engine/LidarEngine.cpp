#include "engine/LidarEngine.hpp"

#include <iostream>
#include <chrono>
#include <thread>

namespace lidar
{

LidarEngine::LidarEngine(std::unique_ptr<BaseLidarSensor> sensor)
    : m_sensor(std::move(sensor))
    , m_readIndex(0U)
    , m_latestTimestamp(0U)
{
}

bool LidarEngine::initialize()
{
    if (!m_sensor)
    {
        std::cerr << "No sensor configured for the LiDAR engine" << '\n';
        return false;
    }

    m_sensor->configure(30.0F, 120.0F);
    std::cout << "Preparing sensor " << m_sensor->identifier() << '\n';
    return m_visualizer.initialize();
}

void LidarEngine::run()
{
    if (!initialize())
    {
        return;
    }

    while (!m_visualizer.windowShouldClose())
    {
        const auto frameStart = std::chrono::steady_clock::now();

        captureFrame();
        m_visualizer.updatePoints(m_pointBuffers[m_readIndex]);
        m_visualizer.render();

        m_readIndex = (m_readIndex + 1U) % m_pointBuffers.size();

        const auto scaledTarget = kTargetFrameDuration / m_visualizer.frameSpeedScale();
        const auto frameDuration = std::chrono::steady_clock::now() - frameStart;
        if (frameDuration < scaledTarget)
        {
            std::this_thread::sleep_for(scaledTarget - frameDuration);
        }
    }
}

void LidarEngine::captureFrame()
{
    uint64_t timestamp = 0U;
    BaseLidarSensor::PointCloud& buffer = m_pointBuffers[m_readIndex];
    buffer.clear();

    if (!m_sensor->readNextScan(buffer, timestamp))
    {
        std::cerr << "Sensor returned no data" << '\n';
        return;
    }

    m_latestTimestamp = timestamp;
}

} // namespace lidar
