cmake_minimum_required(VERSION 3.30)
project(LiDARProcessor VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(EXISTS "${CMAKE_BINARY_DIR}/conan_toolchain.cmake")
    include("${CMAKE_BINARY_DIR}/conan_toolchain.cmake")
endif()

set(LIDAR_CORE_SOURCES
    velodyne/src/engine/LidarEngine.cpp
    velodyne/src/sensors/LidarFactory.cpp
    velodyne/src/sensors/VelodyneLidar.cpp
    visualization/Shader.cpp
    visualization/Visualizer.cpp
    mapping/LidarVirtualSensorMapping.cpp
    reader/src/VelodynePCAPReader.cpp
    bindings/imgui_impl_glfw.cpp
    bindings/imgui_impl_opengl3.cpp
)
set(SPLINTER_SOURCES
    splinter/src/bspline.cpp
    splinter/src/bsplinebasis.cpp
    splinter/src/bsplinebasis1d.cpp
    splinter/src/bsplinebuilder.cpp
    splinter/src/datapoint.cpp
    splinter/src/datatable.cpp
    splinter/src/function.cpp
    splinter/src/knots.cpp
    splinter/src/mykroneckerproduct.cpp
    splinter/src/serializer.cpp
    splinter/src/utilities.cpp
)

add_library(Splinter STATIC ${SPLINTER_SOURCES})
target_include_directories(Splinter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/splinter/include)
target_link_libraries(Splinter PUBLIC Eigen3::Eigen)

add_library(LidarCore STATIC ${LIDAR_CORE_SOURCES})
target_include_directories(LidarCore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/velodyne/include
    ${CMAKE_CURRENT_SOURCE_DIR}/reader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/bindings
)
target_link_libraries(LidarCore PRIVATE Splinter)

find_package(glfw3 REQUIRED)
find_package(glew REQUIRED)
find_package(OpenGL REQUIRED)
find_package(imgui REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glm REQUIRED)

if(TARGET CONAN_PKG::glfw)
    target_link_libraries(LidarCore PRIVATE CONAN_PKG::glfw)
elseif(TARGET glfw::glfw)
    target_link_libraries(LidarCore PRIVATE glfw::glfw)
elseif(TARGET glfw)
    target_link_libraries(LidarCore PRIVATE glfw)
endif()

if(TARGET CONAN_PKG::glew)
    target_link_libraries(LidarCore PRIVATE CONAN_PKG::glew)
else()
    target_link_libraries(LidarCore PRIVATE GLEW::GLEW)
endif()

if(TARGET CONAN_PKG::imgui)
    target_link_libraries(LidarCore PRIVATE CONAN_PKG::imgui)
else()
    target_link_libraries(LidarCore PRIVATE imgui::imgui)
endif()

target_link_libraries(LidarCore PRIVATE OpenGL::GL)
target_link_libraries(LidarCore PRIVATE Eigen3::Eigen)
target_link_libraries(LidarCore PRIVATE glm::glm)

add_executable(LiDARProcessor test/main.cpp)
target_link_libraries(LiDARProcessor PRIVATE LidarCore)
set_target_properties(LiDARProcessor PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

set(BUILD_RESOURCES_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
file(MAKE_DIRECTORY ${BUILD_RESOURCES_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/visualization/imgui.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

enable_testing()
find_package(GTest REQUIRED)

add_executable(LidarProcessorTests
    unitTests/mapping_tests.cpp
    unitTests/reader_tests.cpp
    unitTests/velodyne_tests.cpp
)
target_link_libraries(LidarProcessorTests PRIVATE LidarCore GTest::gtest_main glm::glm Eigen3::Eigen)
target_include_directories(LidarProcessorTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/velodyne/include
    ${CMAKE_CURRENT_SOURCE_DIR}/reader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/visualization
    ${CMAKE_CURRENT_SOURCE_DIR}/mapping
    ${CMAKE_CURRENT_SOURCE_DIR}/bindings
)
add_test(NAME LidarProcessorTests COMMAND LidarProcessorTests)
