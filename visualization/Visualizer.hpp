#pragma once

#include "sensors/BaseLidarSensor.hpp"
#include "visualization/Shader.hpp"

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>
#include <vector>

struct GLFWwindow;

namespace visualization
{
using BaseLidarSensor = lidar::BaseLidarSensor;

class Visualizer
{
public:
    Visualizer() = default;
    ~Visualizer();

    bool initialize();
    void updatePoints(const BaseLidarSensor::PointCloud& points);
    void render();
    bool windowShouldClose() const;
    glm::vec3 computeCameraDirection() const;
    glm::vec3 computeCameraUp() const;
    float frameSpeedScale() const;

private:
    struct Vertex
    {
        float x;
        float y;
        float z;
        float intensity;
        float classification;
    };

    enum class CameraMode
    {
        FreeOrbit = 0,
        BirdsEye,
        Front,
        Side,
        Rear
    };

    struct Camera
    {
        float distance = 30.0F;
        float yaw = 45.0F;
        float pitch = 25.0F;
        float fov = 45.0F;
        bool rotating = false;
        double lastX = 0.0;
        double lastY = 0.0;
    };

    enum class ColorMode
    {
        Classification = 0,
        Height,
        Intensity
    };

    enum class AlphaMode
    {
        UserValue = 0,
        Intensity
    };

    struct WorldFrameSettings
    {
        bool enableWorldVisualization = true;
        bool enableGroundPlane = true;
        bool enableNonGroundPlane = true;
        float pointSize = 3.0F;
        ColorMode colorMode = ColorMode::Height;
        AlphaMode alphaMode = AlphaMode::UserValue;
        float clipHeight = 5.0F;
        float clipIntensity = 1.0F;
        float commonTransparency = 0.65F;
        float groundPlaneTransparency = 0.75F;
        float nongroundPlaneTransparency = 0.9F;
        float groundClassificationHeight = 0.15F;
        float replaySpeed = 1.0F;
        std::array<float, 3> groundPlaneColor = {0.1F, 0.7F, 0.1F};
        std::array<float, 3> nonGroundPlaneColor = {1.0F, 0.35F, 0.0F};
    };

    void uploadBuffer();
    void cleanUp();
    void applyUniforms();
    void drawWorldControls();
    void drawColorLegend();
    bool isGroundPoint(const lidar::LidarPoint& point) const noexcept;
    void processCursorPos(double xpos, double ypos);
    void processScroll(double yoffset);
    void processMouseButton(int button, int action);
    int zoneIndexFromHeight(float height) const noexcept;
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    glm::vec3 sampleHeightColor(float normalized) const;
    glm::vec3 sampleIntensityColor(float normalized) const;

    GLFWwindow* m_window = nullptr;
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
    Shader m_shader;
    std::vector<Vertex> m_vertexBuffer;
    std::size_t m_groundPointCount = 0;
    std::size_t m_nonGroundPointCount = 0;
    std::size_t m_gpuCapacity = 0;
    bool m_needsReallocation = false;
    float m_minHeight = 0.0F;
    float m_maxHeight = 1.0F;
    WorldFrameSettings m_worldFrameSettings;
    Camera m_camera;
    CameraMode m_cameraMode = CameraMode::FreeOrbit;
    int m_activeMouseButton = -1;
};

} // namespace visualization
