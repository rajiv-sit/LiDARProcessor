#pragma once

#include "LidarScan.hpp"
#include "VelodynePCAPReader.hpp"
#include "sensors/BaseLidarSensor.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace lidar
{

class VelodyneLidar : public BaseLidarSensor
{
public:
    explicit VelodyneLidar(std::string identifier, std::string pcapPath);
    ~VelodyneLidar() override;

    const std::string& identifier() const noexcept override;
    void configure(float vertical_fov_deg, float max_range_m) override;
    bool readNextScan(PointCloud& destination, uint64_t& timestamp_us) override;

private:
    void initializeSensor();
    void finalizeSensor();
    void populateGeometry(PointCloud& destination);

    static const std::array<float, VDYNE::maxkHDLNumBeams> HDL32_VERTICAL_ANGLES_RAD;
    static const std::array<float, VDYNE::maxkHDLNumBeams> VLP16_VERTICAL_ANGLES_RAD;

    std::string m_identifier;
    std::string m_pcapPath;
    VDYNE::LiDARScan_t m_scan{};
    VDYNE::LiDARConfiguration_t m_config{};
    std::array<float, VDYNE::maxkHDLNumBeams> m_verticalAnglesRad{};

    float m_verticalFovDeg = 30.0F;
    float m_maxRangeMeters = 120.0F;
    float m_metersPerTick = 0.002F;
    float m_microsecondsPerLaserFiring = 1.152F;
    float m_spinRate = 600.0F * (1.0F / 60.0F * 2.0F * 3.14159265358979323846F / 1e6F);

    bool m_initialized = false;
    bool m_pendingScan = false;
};

} // namespace lidar
