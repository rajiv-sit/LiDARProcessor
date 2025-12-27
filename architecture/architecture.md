# LiDARProcessor Architecture

## 1. Overview
- The `LiDARProcessor` binary (`test/main.cpp`) locates `data/testCase.pcap`, instantiates a Velodyne sensor via `VelodyneFactory`, and hooks it into `lidar::LidarEngine` so the render loop only depends on the abstract sensor interface.
- `LidarEngine` cycles scans every ~33 ms, maintains double-buffered `PointCloud` storage, and feeds the visualizer while keeping replay speed scaling, timestamps, and sensor configuration in lockstep (`velodyne/src/engine/LidarEngine.cpp:10-69`).
- Visualization drives shaders in `shaders/point.vs/.fs`, hosts ImGui controls, and overlays both the virtual sensor hulls and the new free-space map that respect the contour/offset/toggle logic.

## 2. Reader & Sensor
- `reader/src/VelodynePCAPReader.cpp` parses DAT-style HDL32/VLP16 packets via `VDYNE` structures (`reader/include/LidarScan.hpp`), exposing a C++ API so `VelodyneLidar` can consume scans without pulling in larger SDKs.
- `VelodyneLidar` applies vertical-angle tables, filtering, and coordinate transforms to produce `(x,y,z)` frames while the factory supports HDL-32E and VLP-16 variants (`velodyne/src/sensors/VelodyneLidar.cpp`, `velodyne/src/sensors/LidarFactory.cpp`).

## 3. Visualization Pipeline
- `Visualizer` keeps VAOs/VBOs for ground/non-ground points, a shader, and ImGui context—plus world controls for camera mode, point size, color/alpha, clipping, replay speed, and contour overlays (`visualization/Visualizer.cpp`).
- Altitude classification uses fourteen zone labels and color thresholds to assign each point to a bucket when free orbit + classification is enabled (`kZoneLabels`, `kZoneColors`, `kZoneThresholds`).
- The UI now exposes `Show virtual sensor map`, `Show free-space map`, and `Show vehicle contour`, rendering sensor cones, hulls, and the yellow free-space sectors that stop at the closest valid measurement per angular bin.
- `LidarVirtualSensorMapping` exposes 72 angular bins, stores separate ground/non-ground hulls, ignores points beneath `m_floorHeight` or within the inflated contour, and accepts the sensor offset so VCS→ISO alignment stays valid (`mapping/LidarVirtualSensorMapping.cpp`).
- Vehicle contours inflate by `(0.1 m, 0.1 m)` at INI parse time to provide a safety buffer, and world controls surface the inflated contour, transparency, and rotation applied before drawing. 

## 4. Data Flow
- `Visualizer::updatePoints` translates VCS samples relative to the sensor offset, filters ground vs. non-ground via the `Ground height threshold` slider, and sends only non-ground points (still in VCS) to the mapping layer, which subtracts the offset again for contour checks.
- The free-space map draws each sector as a yellow polygon that stretches to the `snapshot.position` or `kVirtualSensorMaxRange`, with a boundary line highlighting the measurement limit, while `drawVirtualSensorsFancy` sticks to the pink/purple palette for shadows, measurements, and the ground hull.

## 5. Directory Snapshot
```
LiDARProcessor
├─ architecture/
│  └─ architecture.md           # this overview
├─ data/
│  ├─ VehicleProfileCustom.ini  # vehicle contour + lidar mount definitions
│  ├─ VehicleProfileFusion.ini
│  ├─ VisualizerSettings.ini
│  └─ testCase.pcap            # HDL-32E capture replayed by the reader
├─ mapping/
│  └─ LidarVirtualSensorMapping.{cpp,hpp}  # sensor bin hulls with contour filtering
├─ reader/
│  └─ VelodynePCAPReader.cpp    # DAT reader feeding Velodyne sensors
├─ shaders/
│  └─ point.{vs,fs}             # GLSL programs for coloring points by height/intensity/classification
├─ visualization/
│  ├─ Visualizer.{cpp,hpp}      # GL/ImGui UI, world controls, overlays, contour translation helpers
│  └─ Shader.cpp                 # GLSL wrapper
├─ velodyne/
│  ├─ sensors/
│  │  ├─ VelodyneLidar.cpp
│  │  └─ LidarFactory.cpp
│  └─ engine/
│     └─ LidarEngine.cpp
├─ run_debug.bat
├─ run_release.bat
├─ CMakeLists.txt
└─ conanfile.py
```

## 6. Build & Runtime
- The `run_debug.bat` helper installs Conan dependencies, generates the toolchain, builds the project, copies shaders/data, and runs the binary—remember to close the GLFW window so the script can exit cleanly.
- The executable prints the sensor model (e.g., `Velodyne HDL-32E`) before the ImGui window appears; use the world toggles, contour sliders, and map buttons to validate free-space coverage and hull alignment.

## 7. Testing & Observability
- ImGui stats show total, ground, non-ground, and GPU point counts, while world controls expose `Ground height threshold`, `Show virtual sensor map`, and `Show free-space map` states.
- Height/isolation palettes are refreshed each frame by the shader uniforms, and the free-space map reuses the sensor polygon builder so the overlay stays consistent with the colored point cloud.
