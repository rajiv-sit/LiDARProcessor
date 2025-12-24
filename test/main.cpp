#include "engine/LidarEngine.hpp"
#include "sensors/LidarFactory.hpp"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    const std::filesystem::path exePath = std::filesystem::absolute(argv[0]).parent_path();
    const std::filesystem::path defaultPcap = exePath / "data" / "testCase.pcap";
    std::string pcapPath = defaultPcap.string();

    if (argc > 1)
    {
        pcapPath = argv[1];
    }

    auto sensor = lidar::LidarFactory::createSensor("velodyne", pcapPath);
    if (!sensor)
    {
        std::cerr << "Failed to create lidar sensor" << '\n';
        return EXIT_FAILURE;
    }

    lidar::LidarEngine engine(std::move(sensor));
    engine.run();
    return EXIT_SUCCESS;
}
