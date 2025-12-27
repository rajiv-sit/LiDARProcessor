#include "mapping/LidarVirtualSensorMapping.hpp"

#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <cmath>

namespace mapping
{
namespace
{
constexpr float kSensorTolerance = 1e-5F;
}

LidarVirtualSensorMapping::LidarVirtualSensorMapping(float floorHeight)
    : m_floorHeight(floorHeight)
{
    rebuild();
}

void LidarVirtualSensorMapping::setFloorHeight(float floorHeight)
{
    if (std::fabs(floorHeight - m_floorHeight) < kSensorTolerance)
    {
        return;
    }
    m_floorHeight = floorHeight;
}

void LidarVirtualSensorMapping::updatePoints(
    const lidar::BaseLidarSensor::PointCloud& points)
{
    resetSamples();

    for (const auto& point : points)
    {
        if (point.z < m_floorHeight)
        {
            continue;
        }

        const glm::vec2 position(point.x, point.y);
        const float distanceSquared = glm::dot(position, position);

        for (std::size_t sensorIndex = 0; sensorIndex < kVirtualSensorCount; ++sensorIndex)
        {
            const auto& sensor = m_sensorDefinitions[sensorIndex];
            if (!sensorContains(sensor, position))
            {
                continue;
            }

            auto& sample = m_sensorSamples[sensorIndex];
            if (distanceSquared < sample.distanceSquared)
            {
                sample.distanceSquared = distanceSquared;
                sample.position = position;
                sample.valid = true;
            }
        }
    }

    m_hull.clear();
    for (const auto& sample : m_sensorSamples)
    {
        if (sample.valid)
        {
            m_hull.push_back(sample.position);
        }
    }
}

void LidarVirtualSensorMapping::setVehicleContour(const std::vector<glm::vec2>& contour)
{
    if (contour.empty())
    {
        return;
    }

    glm::vec2 center(0.0F);
    for (const auto& point : contour)
    {
        center += point;
    }
    center /= static_cast<float>(contour.size());

    float maxDistanceSquared = 0.0F;
    for (const auto& point : contour)
    {
        const glm::vec2 offset = point - center;
        maxDistanceSquared = std::max(maxDistanceSquared, glm::dot(offset, offset));
    }
    const float radius = std::sqrt(maxDistanceSquared);

    const glm::vec2 centerDelta = center - m_vehicleCenter;
    const bool centerChanged = glm::dot(centerDelta, centerDelta) > (kSensorTolerance * kSensorTolerance);
    const bool radiusChanged = std::fabs(radius - m_vehicleRadius) > kSensorTolerance;
    if (!centerChanged && !radiusChanged)
    {
        return;
    }

    m_vehicleCenter = center;
    m_vehicleRadius = radius;
    rebuild();
}

const std::vector<glm::vec2>& LidarVirtualSensorMapping::hull() const noexcept
{
    return m_hull;
}

std::array<LidarVirtualSensorMapping::SensorSnapshot, LidarVirtualSensorMapping::kVirtualSensorCount>
LidarVirtualSensorMapping::snapshots() const
{
    std::array<SensorSnapshot, kVirtualSensorCount> output{};
    for (std::size_t i = 0; i < kVirtualSensorCount; ++i)
    {
        const auto& definition = m_sensorDefinitions[i];
        const auto& sample = m_sensorSamples[i];
        output[i] = SensorSnapshot{
            sample.valid,
            definition.isAngular,
            definition.reference,
            definition.lowerAngle,
            definition.upperAngle,
            definition.wrapAround,
            definition.orthMinX,
            definition.orthMaxX,
            definition.orthSideSign,
            definition.orthMinY,
            definition.orthMaxY,
            sample.position,
            sample.distanceSquared};
    }
    return output;
}

void LidarVirtualSensorMapping::rebuild()
{
    std::fill(m_sensorDefinitions.begin(), m_sensorDefinitions.end(), SensorDefinition{});
    std::fill(m_sensorSamples.begin(), m_sensorSamples.end(), SensorSample{});
    m_hull.clear();

    const float delta = glm::two_pi<float>() / static_cast<float>(kNumAngularSensors);
    float theta = 0.0F;

    for (std::size_t index = 0; index < kNumAngularSensors; ++index)
    {
        const float startAngle = normalizeAngle(theta);
        theta += delta;
        const float endAngle = normalizeAngle(theta);

        SensorDefinition definition{};
        definition.isAngular = true;
        definition.reference = m_vehicleCenter;
        definition.lowerAngle = startAngle;
        definition.upperAngle = endAngle;
        definition.wrapAround = endAngle < startAngle;
        m_sensorDefinitions[index] = definition;
    }
}

void LidarVirtualSensorMapping::resetSamples()
{
    for (auto& sample : m_sensorSamples)
    {
        sample.valid = false;
        sample.distanceSquared = std::numeric_limits<float>::max();
        sample.position = glm::vec2(0.0F);
    }
}

bool LidarVirtualSensorMapping::sensorContains(const SensorDefinition& sensor, const glm::vec2& point) const
{
    if (sensor.isAngular)
    {
        const glm::vec2 relative = point - sensor.reference;
        const float radiusSquared = glm::dot(relative, relative);
        if (radiusSquared < kSensorTolerance)
        {
            return true;
        }

        float angle = std::atan2(relative.y, relative.x);
        if (angle < 0.0F)
        {
            angle += glm::two_pi<float>();
        }

        if (sensor.wrapAround)
        {
            return angle >= sensor.lowerAngle || angle <= sensor.upperAngle;
        }
        return angle >= sensor.lowerAngle && angle <= sensor.upperAngle;
    }

    if (sensor.orthSideSign > 0.0F && point.y < 0.0F)
    {
        return false;
    }
    if (sensor.orthSideSign < 0.0F && point.y > 0.0F)
    {
        return false;
    }

    const float minY = std::min(sensor.orthMinY, sensor.orthMaxY);
    const float maxY = std::max(sensor.orthMinY, sensor.orthMaxY);
    if (point.y < minY || point.y > maxY)
    {
        return false;
    }

    return point.x >= sensor.orthMinX && point.x <= sensor.orthMaxX;
}

float LidarVirtualSensorMapping::normalizeAngle(float angle)
{
    constexpr float twoPi = glm::two_pi<float>();
    float normalized = std::fmod(angle, twoPi);
    if (normalized < 0.0F)
    {
        normalized += twoPi;
    }
    return normalized;
}

} // namespace mapping
