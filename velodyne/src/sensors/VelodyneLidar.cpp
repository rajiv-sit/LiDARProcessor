#include "sensors/VelodyneLidar.hpp"

#include <cmath>
#include <iostream>
#include <algorithm>

namespace lidar
{

const std::array<float, VDYNE::maxkHDLNumBeams> VelodyneLidar::HDL32_VERTICAL_ANGLES_RAD = {
    -0.535293F, -0.162839F, -0.511905F, -0.139626F, -0.488692F, -0.116239F, -0.465305F, -0.093026F,
    -0.442092F, -0.069813F, -0.418879F, -0.046600F, -0.395666F, -0.023213F, -0.372279F, 0.0F,
    -0.349066F, 0.023213F, -0.325853F, 0.046600F, -0.302466F, 0.069813F, -0.279253F, 0.093026F,
    -0.256040F, 0.116413F, -0.232652F, 0.139626F, -0.209440F, 0.162839F, -0.186227F, 0.186227F};

const std::array<float, VDYNE::maxkHDLNumBeams> VelodyneLidar::VLP16_VERTICAL_ANGLES_RAD = {
    -0.261799F, 0.0174533F, -0.226893F, 0.0523599F, -0.191986F, 0.0872665F, -0.15708F, 0.122173F,
    -0.122173F, 0.15708F, -0.0872665F, 0.191986F, -0.0523599F, 0.226893F, -0.0174533F, 0.261799F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

namespace
{
constexpr float kRadiansPerTick = 1.745329251994329e-04F;
constexpr float kTwoPi = 6.28318530717958647692F;
}

VelodyneLidar::VelodyneLidar(std::string identifier, std::string pcapPath)
    : m_identifier(std::move(identifier))
    , m_pcapPath(std::move(pcapPath))
{
    std::copy(HDL32_VERTICAL_ANGLES_RAD.begin(), HDL32_VERTICAL_ANGLES_RAD.end(), m_verticalAnglesRad.begin());
    m_config = VDYNE::HDL32_Hardware;
}

VelodyneLidar::~VelodyneLidar()
{
    finalizeSensor();
}

const std::string& VelodyneLidar::identifier() const noexcept
{
    return m_identifier;
}

void VelodyneLidar::configure(float vertical_fov_deg, float max_range_m)
{
    m_verticalFovDeg = vertical_fov_deg;
    m_maxRangeMeters = std::max(0.01F, max_range_m);
    if (!m_initialized)
    {
        initializeSensor();
    }
}

bool VelodyneLidar::readNextScan(PointCloud& destination, uint64_t& timestamp_us)
{
    if (!m_initialized || !m_pendingScan)
    {
        return false;
    }

    destination.clear();
    destination.reserve(m_config.blocksPerScan * m_config.firingSequencesPerBlock * m_config.numBeams);

    populateGeometry(destination);
    timestamp_us = m_scan.timestamp_us;

    const int rc = GetNextLidarScan(&m_scan);
    if (rc != GLSE_SUCCESS)
    {
        m_pendingScan = false;
        finalizeSensor();
        return true;
    }

    m_pendingScan = true;
    return true;
}

void VelodyneLidar::initializeSensor()
{
    if (m_initialized || m_pcapPath.empty())
    {
        return;
    }

    const int rc = GetFirstLidarScan(m_pcapPath.c_str(), &m_scan);
    if (rc != GLSE_SUCCESS)
    {
        std::cerr << "VelodyneLidar: Failed to open PCAP " << m_pcapPath << " (" << rc << ")" << std::endl;
        return;
    }

    m_initialized = true;
    m_pendingScan = true;

    switch (m_scan.lidarHardware)
    {
        case VDYNE::LiDARHardware_t::HDL32:
            m_config = VDYNE::HDL32_Hardware;
            m_metersPerTick = 0.002F;
            m_microsecondsPerLaserFiring = 1.152F;
            m_spinRate = 600.0F * (1.0F / 60.0F * kTwoPi / 1e6F);
            std::copy(HDL32_VERTICAL_ANGLES_RAD.begin(), HDL32_VERTICAL_ANGLES_RAD.end(), m_verticalAnglesRad.begin());
            break;
        case VDYNE::LiDARHardware_t::VLP16:
            m_config = VDYNE::VLP16_Hardware;
            m_metersPerTick = 0.002F;
            m_microsecondsPerLaserFiring = 2.304F;
            m_spinRate = 600.0F * (1.0F / 60.0F * kTwoPi / 1e6F);
            std::copy(VLP16_VERTICAL_ANGLES_RAD.begin(), VLP16_VERTICAL_ANGLES_RAD.end(), m_verticalAnglesRad.begin());
            break;
        default:
            std::cerr << "VelodyneLidar: Unsupported hardware - defaulting to HDL32 config" << std::endl;
            m_config = VDYNE::HDL32_Hardware;
            std::copy(HDL32_VERTICAL_ANGLES_RAD.begin(), HDL32_VERTICAL_ANGLES_RAD.end(), m_verticalAnglesRad.begin());
            break;
    }
}

void VelodyneLidar::finalizeSensor()
{
    if (m_initialized)
    {
        EndLidarEnumeration();
        m_initialized = false;
        m_pendingScan = false;
    }
}

void VelodyneLidar::populateGeometry(PointCloud& destination)
{
    for (size_t block = 0; block < m_config.blocksPerScan; ++block)
    {
        for (size_t firing = 0; firing < m_config.firingSequencesPerBlock; ++firing)
        {
            const size_t firingIdx = block * m_config.firingSequencesPerBlock + firing;
            const auto& currentFiring = m_scan.firings[firingIdx];
            const float baseTheta = static_cast<float>(currentFiring.azimuth) * kRadiansPerTick;

            for (size_t beam = 0; beam < m_config.numBeams; ++beam)
            {
                const uint16_t rawRange = currentFiring.v_laser[beam].range;
                if (rawRange == 0)
                {
                    continue;
                }

                const float rangeMeters = static_cast<float>(rawRange) * m_metersPerTick;
                if (rangeMeters > m_maxRangeMeters)
                {
                    continue;
                }

                const float phi = m_verticalAnglesRad[beam];
                const float theta = baseTheta + m_spinRate * static_cast<float>(beam) * m_microsecondsPerLaserFiring;

                const float cosPhi = std::cos(phi);
                const float x = rangeMeters * cosPhi * std::cos(theta);
                const float y = -rangeMeters * cosPhi * std::sin(theta);
                const float z = rangeMeters * std::sin(phi);

                destination.push_back({x, y, z, static_cast<float>(currentFiring.v_laser[beam].refl) / 255.0F});
            }
        }
    }
}

} // namespace lidar
