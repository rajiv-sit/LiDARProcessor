#include <algorithm>

#include <gtest/gtest.h>

#include "mapping/LidarVirtualSensorMapping.hpp"
#include "sensors/BaseLidarSensor.hpp"

namespace
{
lidar::LidarPoint make_point(float x, float y, float z)
{
    return {x, y, z, 1.0F};
}
} // namespace

TEST(LidarVirtualSensorMappingTest, NonGroundPointsPopulateHull)
{
    mapping::LidarVirtualSensorMapping mapper;
    lidar::BaseLidarSensor::PointCloud points{make_point(1.0F, 0.0F, 0.5F)};

    mapper.updatePoints(points);

    EXPECT_FALSE(mapper.nonGroundHull().empty());

    const auto snapshots = mapper.snapshots();
    const auto validSnapshots =
        std::count_if(snapshots.begin(), snapshots.end(), [](const auto& snapshot) { return snapshot.valid; });
    EXPECT_GT(validSnapshots, 0);
}

TEST(LidarVirtualSensorMappingTest, GroundPointsAppearInGroundHull)
{
    mapping::LidarVirtualSensorMapping mapper;
    lidar::BaseLidarSensor::PointCloud points{make_point(1.0F, 0.0F, -2.0F)};

    mapper.updatePoints(points);

    EXPECT_FALSE(mapper.groundHull().empty());
}

TEST(LidarVirtualSensorMappingTest, PointsInsideContourAreIgnored)
{
    mapping::LidarVirtualSensorMapping mapper;
    mapper.setVehicleContour({{-1.0F, -1.0F}, {-1.0F, 1.0F}, {1.0F, 1.0F}, {1.0F, -1.0F}});
    lidar::BaseLidarSensor::PointCloud points{make_point(0.0F, 0.0F, 0.0F)};

    mapper.updatePoints(points);

    EXPECT_TRUE(mapper.nonGroundHull().empty());
}
