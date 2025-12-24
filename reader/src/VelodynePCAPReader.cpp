#include "VelodynePCAPReader.hpp"

#include <algorithm>
#include <array>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <mutex>
#include <numeric>
#include <iostream>
#include <shared_mutex>

#include "LidarScan.hpp"

static FILE*                    fpLiDAR                   = NULL;
static PCAPLiDARTimeScalingType gPcapLidarTimeScalingType = PCAPLiDARTimeScalingType::Corrected;
static unsigned int             dataPacketLength          = 1206 + 42;
static unsigned int             gpsPacketLength           = 512 + 42;

#pragma pack(push, 1)
struct pcap_hdr_t
{
    unsigned int   magic_number;  /* magic number */
    unsigned short version_major; /* major version number */
    unsigned short version_minor; /* minor version number */
    int            thiszone;      /* GMT to local correction */
    unsigned int   sigfigs;       /* accuracy of timestamps */
    unsigned int   snaplen;       /* max length of captured packets, in octets */
    unsigned int   network;       /* data link type */
};

struct pcaprec_hdr_t
{
    unsigned int ts_sec;   /* timestamp seconds */
    unsigned int ts_usec;  /* timestamp microseconds */
    unsigned int incl_len; /* number of octets of packet saved in file */
    unsigned int orig_len; /* actual length of packet */
};

struct data_point_t
{
    short range;
    char  refl;
};

const uint16_t kFlagUpper = 0xEEFF;
struct data_block_t
{
    uint16_t     flag; /* EEFF */
    uint16_t     azimuth;
    data_point_t v_laser[32];
};

struct data_packet_t
{
    char         header[42];
    data_block_t block[12];
    unsigned int tstamp;
    short        factory;
};

struct VelodynePositioningPacket
{
    uint8_t  ethHeader[42];
    uint8_t  reserved[14];
    uint16_t Gyro1;
    uint16_t Temp1;
    uint16_t Accel1X;
    uint16_t Accel1Y;
    uint16_t Gyro2;
    uint16_t Temp2;
    uint16_t Accel2X;
    uint16_t Accel2Y;
    uint16_t Gyro3;
    uint16_t Temp3;
    uint16_t Accel3X;
    uint16_t Accel3Y;
    uint8_t  reserved2[160];
    uint32_t GpsTimestampTOTH;
    uint8_t  reserved3[4];
    uint8_t  NMEASentence[72];
    uint8_t  reserved4[234];
};
#pragma pack(pop)

static bool readNextDataPacket(data_packet_t* pkt, uint64_t* timestamp_us)
{
    // Initialize the return value
    bool validDataPacket = false;

    // Read in the information in the PCAP record header
    pcaprec_hdr_t phdr;
    if (fread(&phdr, sizeof(phdr), 1, fpLiDAR) == 1)
    {
        // Process data packet
        if (phdr.orig_len == dataPacketLength)
        {
            // Compute the timestamp for this LiDAR packet, depending on the .pcap file version.
            *timestamp_us = getPCAPVersionDependentLiDARTimestamp(phdr.ts_sec, phdr.ts_usec, gPcapLidarTimeScalingType);

            // Read packet data
            validDataPacket = fread(pkt, sizeof(data_packet_t), 1, fpLiDAR) == 1;
        }
        else // Skip other packets
        {
            // Advance the file pointer to the next PCAP header, ignoring the data in the unknown packet
            validDataPacket =
                fseek(fpLiDAR, phdr.orig_len, SEEK_CUR) == 0 ? readNextDataPacket(pkt, timestamp_us) : false;
        }
    }

    return validDataPacket;
}

int ImplGetNextLidarScan(VDYNE::LiDARScan_t* scan, FILE* fp)
{
    int rc = GLSE_FILEIOERR;

    data_packet_t   pkt;
    size_t          kHDLMaxBlocksPerScan = 181;
    static uint16_t azimuthChange        = 0;
    for (size_t iBlock = 0; iBlock < kHDLMaxBlocksPerScan; iBlock++)
    {
        bool bOK = readNextDataPacket(&pkt, &scan->block_timestamp_us[iBlock]);
        if (bOK)
        {
            if (iBlock == 0)
            {                                   // check LiDAR type
                if ((pkt.factory >> 8) == 0x22) // when the most significant byte of the 2-byte factory information is
                                                // 0x22, the data is coming from a VLP16
                {
                    // information found in the VLP16 User Manual and Programming Guide on page 16
                    scan->lidarHardware  = VDYNE::LiDARHardware_t::VLP16;
                    kHDLMaxBlocksPerScan = VDYNE::VLP16_Hardware.blocksPerScan;
                }
                else if ((pkt.factory >> 8) == 0x21)
                {
                    // when the most significant byte of the 2-byte factory information is 0x21, the data is coming from
                    // an HDL32E
                    // listed as "blank" or "reserved" in the HDL32 manual, the VLP16 manual describes these bytes
                    scan->lidarHardware  = VDYNE::LiDARHardware_t::HDL32;
                    kHDLMaxBlocksPerScan = VDYNE::HDL32_Hardware.blocksPerScan;
                }
                else if ((pkt.factory >> 8) == 0x28)
                {
                    // when the most significant byte of the 2-byte factory information is 0x28, the data is coming from
                    // an VLP32-C
                    scan->lidarHardware  = VDYNE::LiDARHardware_t::VLP32C;
                    kHDLMaxBlocksPerScan = VDYNE::VLP32C_Hardware.blocksPerScan;
                }
                else
                {
                    scan->lidarHardware = VDYNE::LiDARHardware_t::unknown;
                    std::cerr << "VelodynePCAPReader: Unsupported/Unknown Velodyne Lidar Hardware." << std::endl;
                }
            }
            if (scan->lidarHardware == VDYNE::LiDARHardware_t::VLP16)
            {
                for (size_t i = 0; i < VDYNE::VLP16_Hardware.firingSequencesPerBlock; i++)
                {
                    if (i % 2)
                    { // odd-numbered values are stored differently
                        scan->firings[iBlock * VDYNE::VLP16_Hardware.firingSequencesPerBlock + i].flag =
                            scan->firings[iBlock * VDYNE::VLP16_Hardware.firingSequencesPerBlock + i - 1].flag;
                        if (i != (VDYNE::VLP16_Hardware.firingSequencesPerBlock - 1))
                        { // unless this is the last firing sequence, the azimuth is the average of the next and
                          // previous
                            azimuthChange = (pkt.block[i / 2 + 1].azimuth - pkt.block[i / 2].azimuth) / 2;
                        } // if it is the last firing sequence, the previous azimuth change is used
                        scan->firings[iBlock * VDYNE::VLP16_Hardware.firingSequencesPerBlock + i].azimuth =
                            scan->firings[iBlock * VDYNE::VLP16_Hardware.firingSequencesPerBlock + i - 1].azimuth +
                            azimuthChange;
                        memcpy(&scan->firings[iBlock * VDYNE::VLP16_Hardware.firingSequencesPerBlock + i].v_laser,
                               &pkt.block[i / 2].v_laser[16],
                               16 * sizeof(pkt.block[0].v_laser[0]));
                    }
                    else
                    { // even-numbered values are much simpler
                        memcpy(&scan->firings[iBlock * VDYNE::VLP16_Hardware.firingSequencesPerBlock + i],
                               &pkt.block[i / 2],
                               sizeof(pkt.block[0]) -
                                   16 *
                                       sizeof(pkt.block[0]
                                                  .v_laser[0])); // the last 16 laser readings belong to the next firing
                    }
                }
            }
            else if (scan->lidarHardware == VDYNE::LiDARHardware_t::HDL32)
            {
                memcpy(&scan->firings[iBlock * VDYNE::HDL32_Hardware.firingSequencesPerBlock],
                       &pkt.block,
                       sizeof(pkt.block)); // direct memcpy works for HDL32
            }
            else if (scan->lidarHardware == VDYNE::LiDARHardware_t::VLP32C)
            {
                memcpy(&scan->firings[iBlock * VDYNE::VLP32C_Hardware.firingSequencesPerBlock],
                       &pkt.block,
                       sizeof(pkt.block)); // direct memcpy should work for VLP32C
            }
            else
            {
                // Unknown or unimplemented lidar hardware. No operation done.
            }

            // Set the return code
            rc = GLSE_SUCCESS;
        }
    }

    // Set the total scan timestamp
    scan->timestamp_us = scan->block_timestamp_us[kHDLMaxBlocksPerScan - 1];

    return rc;
}

static bool isValidMagicNumber(uint32_t magic)
{
    return ((magic == 0xa1b23c4d) || (magic == 0x4d3cb2a1) || (magic == 0xa1b2c3d4) || (magic == 0xd4c3b2a1));
}

extern "C" int GetFirstLidarScan(const char* filename, VDYNE::LiDARScan_t* scan)
{
    // First, verify the file is valid.
    int rc = GLSE_FILEIOERR;
#if defined(WIN32)
    errno_t e = fopen_s(&fpLiDAR, filename, "rb");
    if (e == 0)
    {
#else
    fpLiDAR = fopen(filename, "rb");
    if (NULL != fpLiDAR)
    {
#endif
        pcap_hdr_t ghdr;
        int        numread = fread(&ghdr, sizeof(ghdr), 1, fpLiDAR);

        if ((numread == 1) && isValidMagicNumber(ghdr.magic_number))
        {
            // Store the stream position prior to determining the LiDAR time scaling type, since that operation
            // may move the stream.
            fpos_t dataStartPos;
            fgetpos(fpLiDAR, &dataStartPos);

            // The .pcap file appears to be valid. Now determine the LiDAR timestamp
            // scaling.
            determineLiDARTimeScalingType(ghdr.version_major, ghdr.version_minor, fpLiDAR, &gPcapLidarTimeScalingType);

            // Reset the file stream to where it was prior to determining the LiDAR time scaling type.
            fsetpos(fpLiDAR, &dataStartPos);

            rc = ImplGetNextLidarScan(scan, fpLiDAR);
        }
        if (numread != 1)
        {
            perror(filename);
        }
        else if (!isValidMagicNumber(ghdr.magic_number))
        {
            std::cerr << "GetFirstLidarScan: unkown PCAP file format: " << filename << std::endl;
            rc = GLSE_FILEFORMAT_ERROR;
        }
    }
#if defined(WIN32)
    else if (e != 2)
    { // 2 = file not found
#else
    else if (errno != 2)
    {
#endif
        perror(filename);
    }
    return rc;
}

extern "C" int GetNextLidarScan(VDYNE::LiDARScan_t* scan)
{
    return ImplGetNextLidarScan(scan, fpLiDAR);
}

extern "C" void EndLidarEnumeration()
{
    if (fpLiDAR != NULL)
    {
        fclose(fpLiDAR);
    }
    // TODO Free any other resources that were allocated here
}

static unsigned long long convertSecondsToMicroSeconds(unsigned int timestamp_s)
{
    return static_cast<unsigned long long>(timestamp_s) * 1000000ULL;
}

extern "C" unsigned long long
    getPCAPVersionDependentLiDARTimestamp(const unsigned int             phdr_ts_sec,
                                          const unsigned int             phdr_ts_usec,
                                          const PCAPLiDARTimeScalingType pcapLidarTimeScalingType)
{
    unsigned long long timestamp_us = 0U;

    switch (pcapLidarTimeScalingType)
    {
        case PCAPLiDARTimeScalingType::Legacy:
        {
            // Incorrect timestamp scaling in .pcap file, so we must fix here.
            unsigned long long secTicks  = static_cast<unsigned long long>(1000U * phdr_ts_sec);
            unsigned long long usecTicks = static_cast<unsigned long long>(1000U * phdr_ts_usec);
            timestamp_us                 = secTicks + usecTicks;
            break;
        }
        case PCAPLiDARTimeScalingType::Corrected:
        {
            // Correct timestamp scaling is used in .pcap file. Note that there is a CAN rollover
            // time in DV tool at approximately 72 minutes (i.e. 2^32 - 1 microseconds). Therefore,
            // we must take our total tick value and obtain the modulus using the rollover constant.
            unsigned long long secTicks  = convertSecondsToMicroSeconds(phdr_ts_sec);
            unsigned long long usecTicks = static_cast<unsigned long long>(phdr_ts_usec);
            timestamp_us                 = (secTicks + usecTicks) % ((1ULL << 32) - 1ULL);
            break;
        }
        default:
            // If this error is thrown, there is a bug in the code that must be resolved.
            std::cerr << "VelodynePCAPReader.cpp::getPCAPVersionDependentTimestamp: Incorrect pcapLidarTimeScalingType "
                         "received."
                      << std::endl;
    }

    return timestamp_us;
}

extern "C" void determineLiDARTimeScalingType(const unsigned short      pcapVersionMajor,
                                              const unsigned short      pcapVersionMinor,
                                              FILE*                     fpLiDAR,
                                              PCAPLiDARTimeScalingType* pcapLidarTimeScalingType)
{
    PCAPLiDARTimeScalingType arbitratedPcapLidarTimeScalingType;

    // If the .pcap file version is greater than 2.4, we know that the corrected LiDAR timestamps are used.
    if (pcapVersionMajor > 2 || pcapVersionMajor == 2 && pcapVersionMinor > 4)
    {
        arbitratedPcapLidarTimeScalingType = PCAPLiDARTimeScalingType::Corrected;
    }
    // If the .pcap file version is less than 2.4, we know that the legacy LiDAR timestamps are used.
    else if (pcapVersionMajor < 2 || pcapVersionMajor == 2 && pcapVersionMinor < 4)
    {
        arbitratedPcapLidarTimeScalingType = PCAPLiDARTimeScalingType::Legacy;
    }
    // If the .pcap file version is 2.4, then we have to infer scaling ourselves. This is because there was an overlap
    // period where DVTool was using the corrected timestamps without the version number on the .pcap files being
    // rolled.
    else
    {
        // To determine the correct time scaling type, we will read several packets of data from the file, compute some
        // statistics associated with the timestamps and then make a determination based on that.
        const unsigned int                             maxNumPacketsToCheck            = 100U;
        unsigned int                                   microsecondsDeltaTimestampIndex = 0U;
        std::array<unsigned int, maxNumPacketsToCheck> microsecondsDeltaTimestamp;
        bool                                           deltaTimestampInit = false;
        unsigned int                                   lastMicrosecondsTimestamp;
        for (unsigned int iPacket = 0U; iPacket < maxNumPacketsToCheck; iPacket++)
        {
            // Attempt to read the next packet header.
            pcaprec_hdr_t phdr;
            int           numread = fread(&phdr, sizeof phdr, 1, fpLiDAR);

            // If EOF, then stop checking.
            if (feof(fpLiDAR))
            {
                break;
            }

            // Determine the type of packet.
            bool bIsDataPacket = (numread == 1) && (phdr.orig_len == dataPacketLength);
            bool bIsGpsPacket  = (numread == 1) && (phdr.orig_len == gpsPacketLength);

            // Append this microseconds delta timestamp.
            if (deltaTimestampInit && (bIsDataPacket || bIsGpsPacket))
            {
                microsecondsDeltaTimestamp.at(microsecondsDeltaTimestampIndex) =
                    phdr.ts_usec - lastMicrosecondsTimestamp;
                lastMicrosecondsTimestamp = phdr.ts_usec;
                microsecondsDeltaTimestampIndex++;
            }
            else if (bIsDataPacket || bIsGpsPacket)
            {
                lastMicrosecondsTimestamp = phdr.ts_usec;
                deltaTimestampInit        = true;
            }

            // Increment the stream position as necessary.
            fpos_t initialPos;
            fgetpos(fpLiDAR, &initialPos);
            fpos_t posOffset;

#if defined(WIN32)
            posOffset = 0U;
            if (bIsDataPacket)
            {
                posOffset = sizeof(data_packet_t);
            }
            else if (bIsGpsPacket)
            {
                posOffset = sizeof(VelodynePositioningPacket);
            }
            fpos_t newPos = initialPos + posOffset;
#else
            posOffset.__pos = 0U;
            if (bIsDataPacket)
            {
                posOffset.__pos = sizeof(data_packet_t);
            }
            else if (bIsGpsPacket)
            {
                posOffset.__pos = sizeof(VelodynePositioningPacket);
            }
            fpos_t newPos = initialPos;
            newPos.__pos += posOffset.__pos;
#endif
            fsetpos(fpLiDAR, &newPos);
        }

        // Compute the statistics associated with the microseconds timestamp.
        unsigned int numEntries = microsecondsDeltaTimestampIndex;

        // We cannot determine the scaling with 0 or 1 entries.
        if (numEntries <= 1U)
        {
            arbitratedPcapLidarTimeScalingType = PCAPLiDARTimeScalingType::Legacy;
            std::cout << "[Warning] VelodynePCAPReader.cpp::determineLiDARTimeScalingType: could not robustly "
                         "determine the .pcap LiDAR timestmap scaling due to insufficient number of data points. "
                         "Arbitrarily choosing it to be the "
                         "Legacy"
                         " scaling type.\n"
                      << std::endl;
        }
        // Compute the statistics associated with the delta timestamps and make a determination from there.
        else
        {
            auto         minMaxDeltaTimeStamp = std::minmax_element(microsecondsDeltaTimestamp.begin(),
                                                            microsecondsDeltaTimestamp.begin() + numEntries);
            unsigned int minDeltaTime         = *minMaxDeltaTimeStamp.first;
            unsigned int maxDeltaTime         = *minMaxDeltaTimeStamp.second;
            float        meanDeltaTime =
                static_cast<float>(std::accumulate(
                    microsecondsDeltaTimestamp.begin(), microsecondsDeltaTimestamp.begin() + numEntries, 0U)) /
                static_cast<float>(numEntries);

            // Note: these values were obtained emperically by looking at typical results in log files. They may need to
            // be modified if the user has issues with this algorithm making the correct determinations.
            bool minDeltaTimeIndicatesCorrected  = minDeltaTime >= 5;
            bool maxDeltaTimeIndicatesCorrected  = maxDeltaTime >= 25;
            bool meanDeltaTimeIndicatesCorrected = meanDeltaTime >= 7.0F;
            bool minDeltaTimeIndicatesLegacy     = minDeltaTime <= 1;
            bool maxDeltaTimeIndicatesLegacy     = maxDeltaTime <= 5;
            bool meanDeltaTimeIndicatesLegacy    = meanDeltaTime <= 3.0F;

            if (minDeltaTimeIndicatesCorrected && maxDeltaTimeIndicatesCorrected && meanDeltaTimeIndicatesCorrected)
            {
                // Its a safe bet to assume corrected timestamps.
                arbitratedPcapLidarTimeScalingType = PCAPLiDARTimeScalingType::Corrected;
            }
            else if (minDeltaTimeIndicatesLegacy && maxDeltaTimeIndicatesLegacy && meanDeltaTimeIndicatesLegacy)
            {
                // Its a safe bet to assume legacy timestamps.
                arbitratedPcapLidarTimeScalingType = PCAPLiDARTimeScalingType::Legacy;
            }
            else
            {
                // See if we have more votes one way or the other.
                int correctedTypeVotes =
                    minDeltaTimeIndicatesCorrected + maxDeltaTimeIndicatesCorrected + meanDeltaTimeIndicatesCorrected;
                int legacyTypeVotes =
                    minDeltaTimeIndicatesLegacy + maxDeltaTimeIndicatesLegacy + meanDeltaTimeIndicatesLegacy;

                if (correctedTypeVotes > legacyTypeVotes)
                {
                    std::cout << "[Warning] VelodynePCAPReader.cpp::determineLiDARTimeScalingType: chose "
                                 "Corrected"
                                 " timestamp type based on votes with medium confidence.\n"
                              << std::endl;
                    arbitratedPcapLidarTimeScalingType = PCAPLiDARTimeScalingType::Corrected;
                }
                else if (legacyTypeVotes > correctedTypeVotes)
                {
                    std::cout << "[Warning] VelodynePCAPReader.cpp::determineLiDARTimeScalingType: chose "
                                 "Legacy"
                                 " timestamp type based on votes with medium confidence.\n"
                              << std::endl;
                    arbitratedPcapLidarTimeScalingType = PCAPLiDARTimeScalingType::Legacy;
                }
                else
                {
                    // Could not robustly determine which type of timestamps.
                    std::cout << "[Warning] VelodynePCAPReader.cpp::determineLiDARTimeScalingType: could not robustly "
                                 "determine the .pcap LiDAR timestmap scaling due to uncertainty in results. "
                                 "Arbitrarily choosing it to be "
                                 "Legacy"
                                 " scaling type. Consider changing the threshold values used to make this "
                                 "determination if necessary.\n"
                              << std::endl;
                    arbitratedPcapLidarTimeScalingType = PCAPLiDARTimeScalingType::Legacy;
                }
            }
        }
    }

    *pcapLidarTimeScalingType = arbitratedPcapLidarTimeScalingType;
}
