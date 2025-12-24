#include <cstdio>
#pragma once

#include "LidarScan.hpp"


enum
{
    GLSE_SUCCESS          = 0,
    GLSE_EOF              = 1,
    GLSE_FILEIOERR        = 2,
    GLSE_FILEFORMAT_ERROR = 3
};

// Enumeration for specifying the scaling of the raw LiDAR timestamps in the
// .pcap file. DVTool has the incorrect scaling for a while and the framework
// had to multiply the LiDAR timestamps by 1000. This was done for "Legacy"
// timestamp scaling. DVTool was updated to have the correct scaling and the
// framework no longer has to multiply the LiDAR timestamps by 1000. This is
// considered "Corrected" scaling.
enum PCAPLiDARTimeScalingType
{
    Legacy,
    Corrected
};

#if defined(__cplusplus)
extern "C"
#endif
    int
    GetFirstLidarScan(const char* filename, VDYNE::LiDARScan_t* scan);

/* Returns:
0 - if next scan data is successfully copied into the scan buffer
1  no more scans (EOF)
2  error reading/parsing data
*/
#if defined(__cplusplus)
extern "C"
#endif
    int
    GetNextLidarScan(VDYNE::LiDARScan_t* scan);

#if defined(__cplusplus)
extern "C"
#endif
    void
    EndLidarEnumeration();

// Computes the correct LiDAR timestamps, depending on the version of the .pcap file.
// @param phdr_ts_sec the raw seconds timestamp from the .pcap file.
// @param phdr_ts_usec the raw microseconds timestamp from the .pcap file.
// @param pcapLidarTimeScalingType the scaling used in the .pcap file. which determines how the
// timestamp is computed.
// @return the timestamp corresponding to the LiDAR data.
#if defined(__cplusplus)
extern "C"
#endif
    unsigned long long
    getPCAPVersionDependentLiDARTimestamp(const unsigned int             phdr_ts_sec,
                                          const unsigned int             phdr_ts_usec,
                                          const PCAPLiDARTimeScalingType pcapLidarTimeScalingType);

// Determines the LiDAR timestamp scaling used in the .pcap file.
// @param pcapVersionMajor is the major version number of the .pcap file (obtained from the file header).
// @param pcapVersionMinor is the minor version number of the .pcap file (obtained from the file header).
// @param fpLiDAR is the file pointer to the .pcap file. This file must have already been opened prior to calling this
// function.
// @param pcapLidarTimeScalingType is a pointer to the memory location that will store the LiDAR timestamp scaling type.
#if defined(__cplusplus)
extern "C"
#endif
    void
    determineLiDARTimeScalingType(const unsigned short      pcapVersionMajor,
                                  const unsigned short      pcapVersionMinor,
                                  FILE*                     fpLiDAR,
                                  PCAPLiDARTimeScalingType* pcapLidarTimeScalingType);
