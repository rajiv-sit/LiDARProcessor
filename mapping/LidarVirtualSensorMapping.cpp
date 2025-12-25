#include "mapping/LidarVirtualSensorMapping.hpp"

#include <glm/gtc/constants.hpp>

#include <cmath>

namespace mapping
{
namespace
{
constexpr float kSensorTolerance = 1e-5F;
}

LidarVirtualSensorMapping::LidarVirtualSensorMapping(float forwardOffset, float rearOffset, float floorHeight)
    : m_forwardOffset(forwardOffset)
    , m_rearOffset(rearOffset)
    , m_floorHeight(floorHeight)
{
    rebuild();
}

void LidarVirtualSensorMapping::setOffsets(float forwardOffset, float rearOffset)
{
    if (std::fabs(forwardOffset - m_forwardOffset) < kSensorTolerance &&
        std::fabs(rearOffset - m_rearOffset) < kSensorTolerance)
    {
        return;
    }

    m_forwardOffset = forwardOffset;
    m_rearOffset = rearOffset;
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

    const glm::vec2 forwardPoint(m_forwardOffset, 0.0F);
    const glm::vec2 rearPoint(m_rearOffset, 0.0F);
    const float orthogonalWidth =
        (m_forwardOffset - m_rearOffset) / static_cast<float>(kNumOrthogonalSensors);
    const float delta = glm::pi<float>() / static_cast<float>(kNumAngularSensors);
    float theta = glm::pi<float>();
    std::size_t index = 0;

    auto addAngular = [&](const glm::vec2& origin, std::size_t count) {
        for (std::size_t i = 0; i < count && index < kVirtualSensorCount; ++i)
        {
            const float startAngle = normalizeAngle(theta);
            theta += delta;
            const float endAngle = normalizeAngle(theta);

            SensorDefinition definition{};
            definition.isAngular = true;
            definition.reference = origin;
            definition.lowerAngle = startAngle;
            definition.upperAngle = endAngle;
            definition.wrapAround = endAngle < startAngle;
            definition.orthSideSign = 0.0F;
            definition.orthMinX = 0.0F;
            definition.orthMaxX = 0.0F;
            m_sensorDefinitions[index++] = definition;
        }
    };

    auto addOrthogonal = [&](const glm::vec2& origin, float step) {
        const float sideSign = step >= 0.0F ? 1.0F : -1.0F;
        for (std::size_t i = 0; i < kNumOrthogonalSensors && index < kVirtualSensorCount; ++i)
        {
            const glm::vec2 start = origin + glm::vec2(step * static_cast<float>(i), 0.0F);
            const glm::vec2 end =
                origin + glm::vec2(step * (static_cast<float>(i) + 1.0F), 0.0F);

            SensorDefinition definition{};
            definition.isAngular = false;
            definition.reference = origin;
            definition.orthMinX = std::min(start.x, end.x);
            definition.orthMaxX = std::max(start.x, end.x);
            definition.orthSideSign = sideSign;
            definition.lowerAngle = 0.0F;
            definition.upperAngle = 0.0F;
            definition.wrapAround = false;
            m_sensorDefinitions[index++] = definition;
        }
    };

    addAngular(rearPoint, kNumAngularSensors / 2U);
    addOrthogonal(rearPoint, orthogonalWidth);
    addAngular(forwardPoint, kNumAngularSensors);
    addOrthogonal(forwardPoint, -orthogonalWidth);
    addAngular(rearPoint, kNumAngularSensors / 2U);
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
