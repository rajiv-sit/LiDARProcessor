cmake_minimum_required(VERSION 3.30)
project(LiDARProcessor VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(EXISTS "${CMAKE_BINARY_DIR}/conan_toolchain.cmake")
    include("${CMAKE_BINARY_DIR}/conan_toolchain.cmake")
endif()

set(LIDAR_SOURCES
    test/main.cpp
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

add_executable(LiDARProcessor ${LIDAR_SOURCES})

target_include_directories(LiDARProcessor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/velodyne/include
    ${CMAKE_CURRENT_SOURCE_DIR}/reader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/bindings
)

find_package(glfw3 REQUIRED)
find_package(glew REQUIRED)
find_package(OpenGL REQUIRED)
find_package(imgui REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glm REQUIRED)

# Conan packages may expose their targets via CONAN_PKG::
if(TARGET CONAN_PKG::glfw)
    target_link_libraries(LiDARProcessor PRIVATE CONAN_PKG::glfw)
elseif(TARGET glfw::glfw)
    target_link_libraries(LiDARProcessor PRIVATE glfw::glfw)
elseif(TARGET glfw)
    target_link_libraries(LiDARProcessor PRIVATE glfw)
endif()

if(TARGET CONAN_PKG::glew)
    target_link_libraries(LiDARProcessor PRIVATE CONAN_PKG::glew)
else()
    target_link_libraries(LiDARProcessor PRIVATE GLEW::GLEW)
endif()

if(TARGET CONAN_PKG::imgui)
    target_link_libraries(LiDARProcessor PRIVATE CONAN_PKG::imgui)
else()
    target_link_libraries(LiDARProcessor PRIVATE imgui::imgui)
endif()

target_link_libraries(LiDARProcessor PRIVATE OpenGL::GL)
target_link_libraries(LiDARProcessor PRIVATE Eigen3::Eigen)
target_link_libraries(LiDARProcessor PRIVATE glm::glm)

set(BUILD_RESOURCES_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
file(MAKE_DIRECTORY ${BUILD_RESOURCES_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set_target_properties(LiDARProcessor PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
