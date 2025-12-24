# LiDARProcessor

LiDARProcessor is a standalone C++20 application that replays Velodyne `.pcap` traces, converts them into `(x,y,z)` point clouds via a DAT-compatible reader, and renders the result with OpenGL (GLFW + GLEW) plus ImGui-driven controls.

## Overview
- The executable starts at `test/main.cpp` and resolves `data/testCase.pcap` relative to the binary before instantiating a `lidar::LidarEngine` with a factory-created Velodyne sensor interface.
- `VelodyneLidar` wraps `reader/src/VelodynePCAPReader.cpp`, maintains HDL32/VLP16 geometry constants, and produces `BaseLidarSensor::PointCloud` frames that `LidarEngine` buffers and passes to the `Visualizer`.
- `Visualizer` hands points to an OpenGL shader pipeline (`shaders/point.vs/.fs`), offers multiple camera views, classification/height/intensity color modes, altitude zoning in free-orbit mode, and a live ImGui stats/controls docked overlay.

## Building
1. Install [Conan 2.x](https://docs.conan.io/en/latest/) and ensure `cmake` is available.
2. Run the helper script:
   ```bat
   run_debug.bat
   ```
   This sequence removes `build/`, invokes `conan install . -if build --build=missing -s build_type=Debug`, runs `cmake` with the generated toolchain, builds `LiDARProcessor`, and copies `shaders/` plus `data/` into `build/Debug/`.
3. If you prefer manual steps:
   ```bat
   conan install . --output-folder=build --build=missing --settings build_type=Debug
   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -G "Visual Studio 17 2022"
   cmake --build build --config Debug
   ```
4. The build links Eigen3, GLFW, GLEW, GLM, ImGui, and the system OpenGL targets (`CMakeLists.txt:33-63`).

## Running
- Run `build/Debug/LiDARProcessor.exe` (or use `run_debug.bat`, which already executes the binary after building).
- The runtime logs `Preparing sensor <identifier>` and opens a GLFW window with the visualization plus the ImGui overlay.
- The ImGui panel exposes camera view/distance, replay speed, color/alpha modes, point size, clipping, world visualization toggles, and altitude-zone sliders (`visualization/Visualizer.cpp:330-520`).

## Visualizer Experience
- Default rendering splits ground and non-ground buffers to control color/transparency separately; each frame reuses the host-side buffer before uploading to the GPU so the renderer stays responsive (`visualization/Visualizer.cpp:127-189`).
- In **Classification + Free Orbit**, the shader uses five altitude zones (`z < -1.5 m`, `-1.5 ≤ z < 0`, `z = 0`, `0 < z < 1.5`, `z ≥ 1.5`) with a dedicated color palette and legend, letting you see vertical stratification while the slider and scroll clamps start at 0.5 m for close inspection (`visualization/Visualizer.cpp:18-380`, `shaders/point.fs:7-88`).
- Height/intensity modes interpolate between “cool” and “warm” palettes and obey the `Clip height`/`Clip intensity` sliders; the alpha blend respects user transparency or intensity-based visibility (`shaders/point.fs:20-88`).
- Use the stats window (backed by `Visualizer::updatePoints`) to monitor total, ground, non-ground, and GPU point counts while the shader receives min/max height and zone color uniforms each frame (`visualization/Visualizer.cpp:368-410`, `visualization/Visualizer.cpp:115-189`).

## Project Structure
- `architecture/` contains the high-level system overview you are reading.
- `reader/` hosts the DAT-derived reader plus `VDYNE::LidarScan_t` definitions.
- `velodyne/` includes sensors (`VelodyneLidar.cpp`), the engine (`LidarEngine.cpp`), and the factory.
- `visualization/` manages the OpenGL renderer, shader wrapper, and UI logic.
- `shaders/` holds the GLSL vertex/fragment programs that color points by height/intensity/classification.
- `data/` supplies `testCase.pcap` (Velodyne HDL-32E capture) plus INI files referenced by the GUI.

-## Observing the Output
 - With `run_debug.bat`, the `build/Debug/` folder contains the executable, copied shaders, and the `.pcap` data so you can run the binary directly.
 - Try toggling the camera view, zoom, and classification palette to observe how the altitude zone colors slot into the shader, and notice the stats window updating as new scans cycle through.

## Visual Reference
![Visualizer preview](figures/visualizer.png)

Figure 1: The ImGui-driven visualization with altitude zoning, point stats, and shader-driven coloring.

## LiDAR Ecosystem & Roadmap
- **Current support**: only Velodyne HDL/VLP sensors via `reader/src/VelodynePCAPReader.cpp`; additional targets rely on the same `BaseLidarSensor` strategy so new drivers can be slotted in later.
- **Leading automotive/autonomous LiDAR names to consider** (historic & modern references in parentheses show the ecosystems these vendors shape):
  - Velodyne LiDAR – established 3D LiDAR leader (now part of Ouster).
  - Ouster – digital solid-state LiDAR for automotive, industrial, robotics.
  - RoboSense (RoboSense Technology) – Chinese automotive LiDAR provider.
  - Hesai Technology – high-performance LiDAR manufacturer from China.
  - Innoviz Technologies – solid-state automotive LiDAR supplier.
  - Cepton – intelligent LiDAR sensors for vehicles and smart infrastructure.
  - Aeva Technologies – 4D LiDAR delivering range + velocity.
  - AEye, Inc. – digital/active scanning LiDAR + perception systems.
  - Luminar Technologies – legacy automotive LiDAR (filed for bankruptcy in 2025).
  - Quanergy Systems – automotive and industrial LiDAR.
  - Ibeo Automotive Systems – early automotive LiDAR pioneer.
  - Benewake – LiDAR modules for robotics and automotive.
  - Livox – notable (now discontinued) sensor modules.

## Troubleshooting
- If `run_debug.bat` fails to find dependencies, rerun `conan install . --output-folder=build --build=missing --settings build_type=Debug`.
- Missing shader files indicate either `shaders/` was not copied or the executable is not run from the build folder; copy `shaders/` and `data/` into the runtime directory if needed.
