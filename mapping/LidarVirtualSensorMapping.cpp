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

void LidarVirtualSensorMapping::setSensorOffset(const glm::vec2& offset)
{
    const glm::vec2 delta = offset - m_sensorOffset;
    if (glm::length(delta) < kSensorTolerance)
    {
        return;
    }
    m_sensorOffset = offset;
}

void LidarVirtualSensorMapping::updatePoints(
    const lidar::BaseLidarSensor::PointCloud& points)
{
    resetSamples();

    for (const auto& point : points)
    {
        const glm::vec2 rawPosition(point.x, point.y);
        const glm::vec2 position = rawPosition - m_sensorOffset;
        if (isInsideVehicleContour(position))
        {
            continue;
        }
        const bool groundPoint = point.z < m_floorHeight;
        const float distanceSquared = glm::dot(position, position);

        auto& samples = groundPoint ? m_sensorSamplesGround : m_sensorSamples;
        for (std::size_t sensorIndex = 0; sensorIndex < kVirtualSensorCount; ++sensorIndex)
        {
            const auto& sensor = m_sensorDefinitions[sensorIndex];
            if (!sensorContains(sensor, position))
            {
                continue;
            }

            auto& sample = samples[sensorIndex];
            if (distanceSquared < sample.distanceSquared)
            {
                sample.distanceSquared = distanceSquared;
                sample.position = position;
                sample.valid = true;
            }
        }
    }

    m_hullNonGround.clear();
    for (const auto& sample : m_sensorSamples)
    {
        if (sample.valid)
        {
            m_hullNonGround.push_back(sample.position);
        }
    }

    m_hullGround.clear();
    for (const auto& sample : m_sensorSamplesGround)
    {
        if (sample.valid)
        {
            m_hullGround.push_back(sample.position);
        }
    }
}

void LidarVirtualSensorMapping::setVehicleContour(const std::vector<glm::vec2>& contour)
{
    if (contour.empty())
    {
        return;
    }
    m_vehicleContour = contour;

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
    return m_hullNonGround;
}

const std::vector<glm::vec2>& LidarVirtualSensorMapping::groundHull() const noexcept
{
    return m_hullGround;
}

const std::vector<glm::vec2>& LidarVirtualSensorMapping::nonGroundHull() const noexcept
{
    return m_hullNonGround;
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
    std::fill(m_sensorSamplesGround.begin(), m_sensorSamplesGround.end(), SensorSample{});
    m_hullNonGround.clear();
    m_hullGround.clear();

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
    for (auto& sample : m_sensorSamplesGround)
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

bool LidarVirtualSensorMapping::isInsideVehicleContour(const glm::vec2& point) const
{
    if (m_vehicleContour.size() < 3)
    {
        return false;
    }

    bool inside = false;
    size_t count = m_vehicleContour.size();
    size_t j = count - 1;
    for (size_t i = 0; i < count; ++i)
    {
        const auto& a = m_vehicleContour[i];
        const auto& b = m_vehicleContour[j];
        const bool intersects = ((a.y > point.y) != (b.y > point.y)) &&
            (point.x <
             (b.x - a.x) * (point.y - a.y) / ((b.y - a.y) != 0.0F ? (b.y - a.y) : std::numeric_limits<float>::epsilon()) + a.x);
        if (intersects)
        {
            inside = !inside;
        }
        j = i;
    }
    return inside;
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
