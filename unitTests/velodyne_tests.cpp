#include <cstddef>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "engine/LidarEngine.hpp"
#include "sensors/BaseLidarSensor.hpp"
#include "sensors/LidarFactory.hpp"
#include "sensors/VelodyneLidar.hpp"
#include "visualization/IVisualizer.hpp"

#include "LidarScan.hpp"

namespace lidar
{
struct LidarEngineTestHelper
{
    static void captureFrame(LidarEngine& engine)
    {
        engine.captureFrame();
    }
};

struct VelodyneLidarTestHelper
{
    static void overrideScan(VelodyneLidar& lidar, const VDYNE::LiDARScan_t& scan)
    {
        lidar.m_scan = scan;
    }

    static void configureForTest(VelodyneLidar& lidar,
                                 const VDYNE::LiDARConfiguration_t& config,
                                 float metersPerTick,
                                 float microsecondsPerLaserFiring,
                                 float spinRate)
    {
        lidar.m_config = config;
        lidar.m_metersPerTick = metersPerTick;
        lidar.m_microsecondsPerLaserFiring = microsecondsPerLaserFiring;
        lidar.m_spinRate = spinRate;
    }

    static void setInitialized(VelodyneLidar& lidar, bool initialized)
    {
        lidar.m_initialized = initialized;
        lidar.m_pendingScan = initialized;
    }

    static void setMaxRange(VelodyneLidar& lidar, float value)
    {
        lidar.m_maxRangeMeters = value;
    }

    static float maxRange(const VelodyneLidar& lidar)
    {
        return lidar.m_maxRangeMeters;
    }

    static void setVerticalAngle(VelodyneLidar& lidar, std::size_t index, float angle)
    {
        if (index < lidar.m_verticalAnglesRad.size())
        {
            lidar.m_verticalAnglesRad[index] = angle;
        }
    }

    static void populateGeometry(VelodyneLidar& lidar, BaseLidarSensor::PointCloud& destination)
    {
        lidar.populateGeometry(destination);
    }
};
} // namespace lidar

namespace
{
class FakeVisualizer : public visualization::IVisualizer
{
public:
    bool initialize() override
    {
        ++initializeCalls;
        return initializeResult;
    }

    void updatePoints(const lidar::BaseLidarSensor::PointCloud&) override
    {
        ++updateCount;
    }

    void render() override
    {
        ++renderCount;
    }

    bool windowShouldClose() const override
    {
        return windowShouldCloseResult;
    }

    float frameSpeedScale() const override
    {
        return frameSpeedScaleResult;
    }

    bool initializeResult = true;
    bool windowShouldCloseResult = true;
    float frameSpeedScaleResult = 1.0F;
    int initializeCalls = 0;
    int updateCount = 0;
    int renderCount = 0;
};

class FakeSensor : public lidar::BaseLidarSensor
{
public:
    explicit FakeSensor(std::string identifier = "fake")
        : m_identifier(std::move(identifier))
    {
    }

    const std::string& identifier() const noexcept override
    {
        return m_identifier;
    }

    void configure(float vertical_fov_deg, float max_range_m) override
    {
        configureCount++;
        lastVerticalFov = vertical_fov_deg;
        lastMaxRange = max_range_m;
    }

    bool readNextScan(PointCloud& destination, uint64_t& timestamp_us) override
    {
        ++readCount;
        if (!readNextScanResult)
        {
            return false;
        }

        destination.clear();
        destination.push_back({0.0F, 0.0F, 0.0F, 1.0F});
        timestamp_us = timestampValue;
        return true;
    }

    std::string m_identifier;
    int configureCount = 0;
    int readCount = 0;
    bool readNextScanResult = true;
    uint64_t timestampValue = 0;
    float lastVerticalFov = 0.0F;
    float lastMaxRange = 0.0F;
};
} // namespace

TEST(LidarEngineTest, InitializeWithoutSensorFailsFast)
{
    auto visualizer = std::make_unique<FakeVisualizer>();
    lidar::LidarEngine engine(nullptr, std::move(visualizer));

    EXPECT_FALSE(engine.initialize());
}

TEST(LidarEngineTest, InitializePropagatesConfiguratorWithVisualizerFailure)
{
    auto sensor = std::make_unique<FakeSensor>();
    auto visualizer = std::make_unique<FakeVisualizer>();
    visualizer->initializeResult = false;

    lidar::LidarEngine engine(std::move(sensor), std::move(visualizer));
    EXPECT_FALSE(engine.initialize());
}

TEST(LidarEngineTest, CaptureFrameUpdatesLatestTimestamp)
{
    auto sensor = std::make_unique<FakeSensor>();
    sensor->timestampValue = 1234ULL;
    auto visualizer = std::make_unique<FakeVisualizer>();

    lidar::LidarEngine engine(std::move(sensor), std::move(visualizer));
    ASSERT_TRUE(engine.initialize());
    lidar::LidarEngineTestHelper::captureFrame(engine);

    EXPECT_EQ(engine.latestTimestamp(), 1234ULL);
}

TEST(LidarFactoryTest, CreateSensorRespectsEmptySource)
{
    EXPECT_EQ(lidar::LidarFactory::createSensor("velodyne", ""), nullptr);
}

TEST(LidarFactoryTest, CreateSensorHandlesCaseInsensitiveType)
{
    auto sensor = lidar::LidarFactory::createSensor("VELoDyne", "dummy.pcap");
    ASSERT_NE(sensor, nullptr);
    EXPECT_EQ(sensor->identifier(), "Velodyne HDL-32E");
}

TEST(VelodyneLidarTest, ConfigureClampsMaximumRange)
{
    lidar::VelodyneLidar lidar("lidar", "");
    lidar.configure(10.0F, 0.0F);

    EXPECT_NEAR(lidar::VelodyneLidarTestHelper::maxRange(lidar), 0.01F, 1e-6F);
}

TEST(VelodyneLidarTest, PopulateGeometryProducesCoordinates)
{
    lidar::VelodyneLidar lidar("lidar", "");
    VDYNE::LiDARConfiguration_t config{1, 1, 1};
    lidar::VelodyneLidarTestHelper::configureForTest(lidar, config, 0.01F, 0.0F, 0.0F);
    lidar::VelodyneLidarTestHelper::setInitialized(lidar, true);
    lidar::VelodyneLidarTestHelper::setMaxRange(lidar, 10.0F);
    lidar::VelodyneLidarTestHelper::setVerticalAngle(lidar, 0, 0.0F);

    VDYNE::LiDARScan_t scan{};
    scan.lidarHardware = VDYNE::LiDARHardware_t::HDL32;
    scan.firings[0].azimuth = 0;
    scan.firings[0].v_laser[0].range = 100;
    scan.firings[0].v_laser[0].refl = 128;
    lidar::VelodyneLidarTestHelper::overrideScan(lidar, scan);

    lidar::BaseLidarSensor::PointCloud points;
    lidar::VelodyneLidarTestHelper::populateGeometry(lidar, points);

    ASSERT_EQ(points.size(), 1U);
    EXPECT_NEAR(points[0].x, 1.0F, 1e-3F);
    EXPECT_NEAR(points[0].y, 0.0F, 1e-3F);
    EXPECT_NEAR(points[0].z, 0.0F, 1e-3F);
}
