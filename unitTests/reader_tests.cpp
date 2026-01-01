#include <gtest/gtest.h>

#include "VelodynePCAPReader.hpp"

TEST(VelodynePcapReaderTest, LegacyTimestampScalingAppliesMultiplier)
{
    const unsigned long long expected = 2000ULL + 3000ULL;
    EXPECT_EQ(
        getPCAPVersionDependentLiDARTimestamp(2, 3, PCAPLiDARTimeScalingType::Legacy),
        expected);
}

TEST(VelodynePcapReaderTest, CorrectedTimestampScalingUsesMicroseconds)
{
    const unsigned long long expected = 1000000ULL + 500ULL;
    EXPECT_EQ(
        getPCAPVersionDependentLiDARTimestamp(1, 500, PCAPLiDARTimeScalingType::Corrected),
        expected);
}

TEST(VelodynePcapReaderTest, DetermineTimeScalingDefaultsToLegacyForOlderVersion)
{
    PCAPLiDARTimeScalingType type = PCAPLiDARTimeScalingType::Corrected;
    determineLiDARTimeScalingType(1, 0, nullptr, &type);
    EXPECT_EQ(type, PCAPLiDARTimeScalingType::Legacy);
}

TEST(VelodynePcapReaderTest, DetermineTimeScalingChoosesCorrectedForNewerVersion)
{
    PCAPLiDARTimeScalingType type = PCAPLiDARTimeScalingType::Legacy;
    determineLiDARTimeScalingType(3, 0, nullptr, &type);
    EXPECT_EQ(type, PCAPLiDARTimeScalingType::Corrected);
}
