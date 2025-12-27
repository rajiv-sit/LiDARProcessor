#pragma once

#include "sensors/BaseLidarSensor.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <array>
#include <cstddef>
#include <limits>
#include <vector>

namespace mapping
{
class LidarVirtualSensorMapping
{
public:
    static constexpr std::size_t kNumAngularSensors = 72U;
    static constexpr std::size_t kVirtualSensorCount = kNumAngularSensors;

    explicit LidarVirtualSensorMapping(float floorHeight = -1.8F);

    void setFloorHeight(float floorHeight);
    void updatePoints(const lidar::BaseLidarSensor::PointCloud& points);
    void setVehicleContour(const std::vector<glm::vec2>& contour);

    const std::vector<glm::vec2>& hull() const noexcept;

    struct SensorSnapshot
    {
        bool valid = false;
        bool isAngular = false;
        glm::vec2 reference = glm::vec2(0.0F);
        float lowerAngle = 0.0F;
        float upperAngle = 0.0F;
        bool wrapAround = false;
        float orthMinX = 0.0F;
        float orthMaxX = 0.0F;
        float orthSideSign = 0.0F;
        float orthMinY = 0.0F;
        float orthMaxY = 0.0F;
        glm::vec2 position = glm::vec2(0.0F);
        float distanceSquared = std::numeric_limits<float>::max();
    };

    std::array<SensorSnapshot, kVirtualSensorCount> snapshots() const;

private:
    struct SensorDefinition
    {
        bool isAngular = false;
        glm::vec2 reference = glm::vec2(0.0F);
        float lowerAngle = 0.0F;
        float upperAngle = 0.0F;
        bool wrapAround = false;
        float orthMinX = 0.0F;
        float orthMaxX = 0.0F;
        float orthSideSign = 0.0F;
        float orthMinY = 0.0F;
        float orthMaxY = 0.0F;
    };

    struct SensorSample
    {
        bool valid = false;
        float distanceSquared = std::numeric_limits<float>::max();
        glm::vec2 position = glm::vec2(0.0F);
    };

    void rebuild();
    void resetSamples();
    float normalizeAngle(float angle);
    bool sensorContains(const SensorDefinition& sensor, const glm::vec2& point) const;

    std::array<SensorDefinition, kVirtualSensorCount> m_sensorDefinitions{};
    std::array<SensorSample, kVirtualSensorCount> m_sensorSamples{};
    std::vector<glm::vec2> m_hull;
    glm::vec2 m_vehicleCenter = glm::vec2(0.0F);
    float m_vehicleRadius = 0.0F;
    float m_floorHeight;
};

} // namespace mapping
