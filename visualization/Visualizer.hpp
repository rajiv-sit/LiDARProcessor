#pragma once

#include "sensors/BaseLidarSensor.hpp"
#include "mapping/LidarVirtualSensorMapping.hpp"
#include "visualization/Shader.hpp"

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

struct GLFWwindow;

namespace visualization
{
struct VehicleProfileData
{
    std::vector<glm::vec2> contour;
    float distRearAxle = 0.0F;
    float lidarHeightAboveGround = 1.8F;
    float lidarLatPos = 0.0F;
    float lidarLonPos = 0.0F;
    float lidarOrientation = 0.0F;
    float height = 0.0F;
    float length = 0.0F;
    float trackFront = 0.0F;
    float trackRear = 0.0F;
    float wheelBase = 0.0F;
    float width = 0.0F;
    float widthIncludingMirrors = 0.0F;
};
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
        float yaw = 90.0F;
        float pitch = -25.0F;
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
        bool showVirtualSensorMap = false;
        bool showVehicleContour = true;
        std::array<float, 3> vehicleContourColor = {0.15F, 0.7F, 1.0F};
        float vehicleContourTransparency = 0.65F;
        float vehicleContourRotation = 0.0F;
    };

    void uploadBuffer();
    void cleanUp();
    void applyUniforms();
    void drawWorldControls();
    void drawVirtualSensorMap();
    void drawVirtualSensorsFancy();
    void buildVirtualSensorMapVertices();
    void updateVirtualSensorMapBuffer();
    void configureVertexArray(GLuint vao, GLuint vbo);
    void drawColorLegend();
    void drawLidarMountMarker(const glm::vec2& position, float rotationDegrees);
    void drawOverlayLine(const glm::vec2& from,
                         const glm::vec2& to,
                         const glm::vec3& color,
                         float alpha,
                         float elevation = 0.0F);
    bool isGroundPoint(const lidar::LidarPoint& point) const noexcept;
    void processCursorPos(double xpos, double ypos);
    void processScroll(double yoffset);
    void processMouseButton(int button, int action);
    void refreshVehicleProfiles();
    void applyVehicleProfile(int index);
    void drawGrid(float spacing = 0.5F);
    int zoneIndexFromHeight(float height) const noexcept;
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    glm::vec3 sampleHeightColor(float normalized) const;
    glm::vec3 sampleIntensityColor(float normalized) const;
    glm::vec2 directionFromAngle(float angle) const;
    std::vector<glm::vec2> buildSensorPolygon(const mapping::LidarVirtualSensorMapping::SensorSnapshot& snapshot,
                                              float nearRange,
                                              float farRange) const;
    std::vector<glm::vec2> buildSensorMeasurementPolygon(
        const mapping::LidarVirtualSensorMapping::SensorSnapshot& snapshot) const;
    std::vector<glm::vec2> buildSensorShadowPolygon(
        const mapping::LidarVirtualSensorMapping::SensorSnapshot& snapshot) const;
    void drawOverlayPolygon(const std::vector<glm::vec2>& positions, const glm::vec3& color, float alpha);
    void drawSensorPoint(const mapping::LidarVirtualSensorMapping::SensorSnapshot& snapshot,
                         const glm::vec3& color,
                         float alpha);
    void applyForceColor(const glm::vec3& color, float alpha);
    void resetForceColor();
    void updateContourTranslation();
    void updateSensorOffsets();
    float distanceToContour(const glm::vec2& point) const;
    float distanceToSegment(const glm::vec2& a, const glm::vec2& b, const glm::vec2& point) const;
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
    std::vector<Vertex> m_virtualSensorMapVertices;
    GLuint m_mapVao = 0;
    GLuint m_mapVbo = 0;
    GLuint m_overlayVao = 0;
    GLuint m_overlayVbo = 0;
    std::vector<glm::vec2> m_vehicleContour;
    std::vector<std::string> m_vehicleProfileEntries;
    int m_selectedVehicleProfileIndex = 0;
    VehicleProfileData m_currentVehicleProfile;
    glm::vec2 m_lidarVcsPosition = glm::vec2(0.0F);
    float m_lidarOrientationIsoDeg = 0.0F;
    glm::vec2 m_contourTranslation = glm::vec2(0.0F);
    glm::vec2 m_lidarSensorOffset = glm::vec2(0.0F); // (latPos, lonPos) plus rear-axle distance
    std::vector<glm::vec2> m_translatedContour;
    glm::vec2 m_closestContourPoint = glm::vec2(0.0F);
    float m_closestContourDistance = std::numeric_limits<float>::max();
    Camera m_camera;
    CameraMode m_cameraMode = CameraMode::FreeOrbit;
    int m_activeMouseButton = -1;
    mapping::LidarVirtualSensorMapping m_virtualSensorMapping;
    float m_mountHeight = 1.8F;
    float m_floorHeight = -1.5F;
    GLint m_forceColorLoc = -1;
    GLint m_forcedColorLoc = -1;
    GLint m_forcedAlphaLoc = -1;
    GLint m_pointSizeLoc = -1;
    glm::vec2 m_gridMin = glm::vec2(-5.0F);
    glm::vec2 m_gridMax = glm::vec2(5.0F);
    float m_gridSpacing = 10.0F;
};

} // namespace visualization
