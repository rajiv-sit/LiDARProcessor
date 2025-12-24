#pragma once

#include <stdint.h>
#include <stdlib.h>

#define VELODYNE_PACKET_LEN (1248)
#define ETHERNET_HEADER_LEN (42)

namespace VDYNE
{

const int HDL_NUM_ROT_ANGLES = 36000;
const int HDL_MAX_NUM_LASERS = 64;

enum HDLBlockIdentifier
{
    BLOCK_0_TO_31  = 0xeeff,
    BLOCK_32_TO_63 = 0xddff
};
typedef enum
{
    COMPLETE_SCAN,
    PARTIAL_SCAN
} LidarScanStatus_e;

const size_t kHDLNumBeamsPerBlock = 32 * 12; // or 16 * 24 for VLP16, but they are the same value.
// const size_t kHDLMaxNumPtsPerScan = kHDLMaxBlocksPerScan * kHDLNumBeamsPerBlock;

const size_t maxkHDLFiringSequencesPerBlock = 24;
const size_t maxkHDLNumBeams                = 32;
const size_t maxkHDLMaxBlocksPerScan        = 181;

#pragma pack(push, 1)

typedef enum
{
    VLP16,
    HDL32,
    VLP32C,
    unknown
} LiDARHardware_t;

typedef struct
{
    size_t blocksPerScan;
    size_t firingSequencesPerBlock;
    size_t numBeams;
} LiDARConfiguration_t;

// Calculation of number of blocks per full scan:
// VLP16:
// 1 data block = 1 firing cycle = 55.296us.
// 1 data packet = 24 data blocks = 24 * 55.296 = 1.327ms
// At 600 rpm: 100ms/rev * 1/1.327ms/dataPkt = 75.3 dataPkts/rev

// HDL32:
// 1 Lidar UPD packet = 12 data blocks (each has 32 beams per block)
// Firing period for one block T_block = 46.08 us
// Packets per revolution PPR = T_revolution / (12 * T_block)
// At 600 rpm : 0.1e6 / (12 * 46.08) = 180.8 blocks per revolution

// VLP32C:
// 1 Lidar UPD packet = 12 data blocks (each has 32 beams per block)
// Firing period for one block T_block = 55.296 us
// Packets per revolution PPR = T_revolution / (12 * T_block)
// At 600 rpm : 0.1e6 / (12 * 55.296) = 150.7 blocks per revolution

const LiDARConfiguration_t VLP16_Hardware   = {76, 24, 16};
const LiDARConfiguration_t HDL32_Hardware   = {181, 12, 32};
const LiDARConfiguration_t VLP32C_Hardware  = {151, 12, 32};
const LiDARConfiguration_t unknown_Hardware = {
    0,
    0,
    0,
};

struct data_point_t
{
    uint16_t range;
    uint8_t  refl;
};

struct data_block_t
{
    uint16_t     flag; // 0xEEFF = HDL32 (0 to 31)
    uint16_t     azimuth;
    data_point_t v_laser[maxkHDLNumBeams];
};

struct LiDARScan_t
{
    LiDARHardware_t lidarHardware;
    uint64_t        timestamp_us; // "CAN Time" - (filled in by framework)
    uint64_t
        block_timestamp_us[maxkHDLMaxBlocksPerScan]; // "CAN Time" - (filled in by framework) for each individual block

    /* Decoded Velodyne laser firing data */
    data_block_t firings[maxkHDLFiringSequencesPerBlock * maxkHDLMaxBlocksPerScan];
};

struct velodyne_data_packet_t
{
    uint8_t data[VELODYNE_PACKET_LEN - ETHERNET_HEADER_LEN];
};

#pragma pack(pop)

} // namespace VDYNE
