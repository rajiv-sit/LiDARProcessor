#include "visualization/Visualizer.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <imgui_impl_glfw.hpp>
#include <imgui_impl_opengl3.hpp>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <system_error>

namespace visualization
{

namespace
{
namespace fs = std::filesystem;

const fs::path kVehicleProfileDirectory{"data"};
constexpr const char* kVehicleProfilePrefix = "VehicleProfile";
constexpr const char* kDefaultVehicleProfileFilename = "VehicleProfileCustom.ini";
constexpr const char* kVertexShaderPath = "shaders/point.vs";
constexpr const char* kFragmentShaderPath = "shaders/point.fs";
constexpr std::array<const char*, 3> kColorModeLabels = {"Classification", "Height", "Intensity"};
constexpr std::array<const char*, 2> kAlphaModeLabels = {"User value", "Intensity"};
constexpr std::array<const char*, 5> kCameraModeLabels = {"Free orbit", "Bird's eye", "Front", "Side", "Rear"};
constexpr float kScrollSpeed = 2.0F;
constexpr std::array<const char*, 14> kZoneLabels = {
    "z < -1.75 m",
    "-1.75 m <= z < -1.50 m",
    "-1.50 m <= z < -1.25 m",
    "-1.25 m <= z < -1.00 m",
    "-1.00 m <= z < -0.75 m",
    "-0.75 m <= z < -0.50 m",
    "-0.50 m <= z < 0.00 m",
    "0.00 m <= z < 0.50 m",
    "0.50 m <= z < 0.75 m",
    "0.75 m <= z < 1.00 m",
    "1.00 m <= z < 1.25 m",
    "1.25 m <= z < 1.50 m",
    "1.50 m <= z < 1.75 m",
    "z >= 1.75 m"};
const std::array<glm::vec3, 14> kZoneColors = {
    glm::vec3(0.05F, 0.25F, 0.85F),
    glm::vec3(0.1F, 0.35F, 0.85F),
    glm::vec3(0.15F, 0.45F, 0.8F),
    glm::vec3(0.2F, 0.55F, 0.7F),
    glm::vec3(0.25F, 0.65F, 0.6F),
    glm::vec3(0.3F, 0.75F, 0.45F),
    glm::vec3(0.3F, 0.85F, 0.3F),
    glm::vec3(0.6F, 0.9F, 0.2F),
    glm::vec3(0.8F, 0.85F, 0.15F),
    glm::vec3(0.9F, 0.7F, 0.1F),
    glm::vec3(0.95F, 0.55F, 0.05F),
    glm::vec3(1.0F, 0.35F, 0.0F),
    glm::vec3(1.0F, 0.15F, 0.05F),
    glm::vec3(0.85F, 0.0F, 0.15F)};
constexpr std::array<float, kZoneColors.size() - 1> kZoneThresholds = {
    -1.75F,
    -1.50F,
    -1.25F,
    -1.00F,
    -0.75F,
    -0.50F,
    0.00F,
    0.50F,
    0.75F,
    1.00F,
    1.25F,
    1.50F,
    1.75F};
constexpr float kDefaultMountHeight = 1.8F;
constexpr float kVirtualSensorMaxRange = 120.0F;
constexpr float kVirtualSensorThickness = 0.5F;
constexpr float kVirtualSensorPointSize = 6.0F;
constexpr float kGridHalfSpan = 50.0F;
constexpr glm::vec2 kContourExpansion(0.1F, 0.1F);

std::string_view trim(std::string_view value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos)
    {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool parseFloat(std::string_view text, float& out)
{
    const auto result = std::from_chars(text.data(), text.data() + text.size(), out);
    return result.ec == std::errc();
}

bool parseInt(std::string_view text, int& out)
{
    const auto result = std::from_chars(text.data(), text.data() + text.size(), out);
    return result.ec == std::errc();
}

std::string_view stripInlineComment(std::string_view value)
{
    const auto semicolon = value.find(';');
    if (semicolon != std::string_view::npos)
    {
        value.remove_suffix(value.size() - semicolon);
    }
    return value;
}

VehicleProfileData loadVehicleProfile(const fs::path& profilePath)
{
    VehicleProfileData profile;
    profile.lidarHeightAboveGround = kDefaultMountHeight;

    std::ifstream file(profilePath);
    if (!file)
    {
        return profile;
    }

    std::string currentSection;
    std::map<int, glm::vec2> contourPoints;
    std::string line;
    while (std::getline(file, line))
    {
        std::string_view trimmedLine = trim(line);
        if (trimmedLine.empty())
        {
            continue;
        }

        const char firstChar = trimmedLine.front();
        if (firstChar == ';' || firstChar == '#')
        {
            continue;
        }

        if (firstChar == '[')
        {
            currentSection = std::string(trimmedLine);
            continue;
        }

        const auto eqPos = trimmedLine.find('=');
        if (eqPos == std::string_view::npos)
        {
            continue;
        }

        std::string_view rawKey = trim(trimmedLine.substr(0, eqPos));
        std::string_view rawValue = trim(trimmedLine.substr(eqPos + 1));
        rawValue = stripInlineComment(rawValue);
        rawValue = trim(rawValue);
        if (rawValue.empty())
        {
            continue;
        }

        if (currentSection == "[Contour]")
        {
            constexpr std::string_view kContourPrefix = "contourPt";
            if (!rawKey.starts_with(kContourPrefix))
            {
                continue;
            }

            int index = 0;
            const auto indexText = rawKey.substr(kContourPrefix.size());
            if (!parseInt(indexText, index))
            {
                continue;
            }

            const auto commaPos = rawValue.find(',');
            if (commaPos == std::string_view::npos)
            {
                continue;
            }

            std::string_view xText = trim(rawValue.substr(0, commaPos));
            std::string_view yText = trim(rawValue.substr(commaPos + 1));
            if (xText.empty() || yText.empty())
            {
                continue;
            }

            float lon = 0.0F;
            float lat = 0.0F;
            if (!parseFloat(xText, lon) || !parseFloat(yText, lat))
            {
                continue;
            }

            contourPoints[index] = glm::vec2(lat, lon); // INI columns are [longitude, latitude]; swap to match VCS (x=lat, y=lon)
            continue;
        }

        if (currentSection == "[Geometry]")
        {
            if (rawKey == "distRearAxle")
            {
                float value = 0.0F;
                if (parseFloat(rawValue, value))
                {
                    profile.distRearAxle = value;
                }
            }
            else if (rawKey == "height")
            {
                float value = 0.0F;
                if (parseFloat(rawValue, value))
                {
                    profile.height = value;
                }
            }
            else if (rawKey == "length")
            {
                float value = 0.0F;
                if (parseFloat(rawValue, value))
                {
                    profile.length = value;
                }
            }
            else if (rawKey == "trackFront")
            {
                float value = 0.0F;
                if (parseFloat(rawValue, value))
                {
                    profile.trackFront = value;
                }
            }
            else if (rawKey == "trackRear")
            {
                float value = 0.0F;
                if (parseFloat(rawValue, value))
                {
                    profile.trackRear = value;
                }
            }
            else if (rawKey == "wheelBase")
            {
                float value = 0.0F;
                if (parseFloat(rawValue, value))
                {
                    profile.wheelBase = value;
                }
            }
            else if (rawKey == "width")
            {
                float value = 0.0F;
                if (parseFloat(rawValue, value))
                {
                    profile.width = value;
                }
            }
            else if (rawKey == "widthIncludingMirrors")
            {
                float value = 0.0F;
                if (parseFloat(rawValue, value))
                {
                    profile.widthIncludingMirrors = value;
                }
            }
            continue;
        }

        if (currentSection == "[LiDAR]")
        {
            if (rawKey == "heightAboveGround")
            {
                float value = 0.0F;
                if (parseFloat(rawValue, value))
                {
                    profile.lidarHeightAboveGround = value;
                }
            }
            else if (rawKey == "latPos")
            {
                parseFloat(rawValue, profile.lidarLatPos);
            }
            else if (rawKey == "lonPos")
            {
                parseFloat(rawValue, profile.lidarLonPos);
            }
            else if (rawKey == "orientation")
            {
                parseFloat(rawValue, profile.lidarOrientation);
            }
            continue;
        }
    }

    profile.contour.reserve(contourPoints.size());
    for (const auto& entry : contourPoints)
    {
        glm::vec2 point = entry.second;
        const glm::vec2 direction(
            std::copysign(1.0F, point.x),
            std::copysign(1.0F, point.y));
        point += direction * kContourExpansion;
        profile.contour.push_back(point);
    }

    return profile;
}

bool vehicleProfileItemsGetter(void* data, int idx, const char** out_text)
{
    if (!data || idx < 0)
    {
        return false;
    }

    const auto* entries = reinterpret_cast<const std::vector<std::string>*>(data);
    if (!entries || idx >= static_cast<int>(entries->size()))
    {
        return false;
    }

    *out_text = entries->at(idx).c_str();
    return true;
}
} // namespace

Visualizer::~Visualizer()
{
    cleanUp();
}

bool Visualizer::initialize()
{
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << '\n';
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    m_window = glfwCreateWindow(1280, 720, "LiDARProcessor", nullptr, nullptr);
    if (!m_window)
    {
        std::cerr << "Failed to create GLFW window" << '\n';
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

    glfwSetWindowUserPointer(m_window, this);
    glfwSetCursorPosCallback(m_window, Visualizer::cursorPosCallback);
    glfwSetScrollCallback(m_window, Visualizer::scrollCallback);
    glfwSetMouseButtonCallback(m_window, Visualizer::mouseButtonCallback);

    if (glewInit() != GLEW_OK)
    {
        std::cerr << "GLEW initialization failed" << '\n';
        return false;
    }

    refreshVehicleProfiles();
    applyVehicleProfile(m_selectedVehicleProfileIndex);

    glEnable(GL_PROGRAM_POINT_SIZE);

    if (!m_shader.load(kVertexShaderPath, kFragmentShaderPath))
    {
        return false;
    }

    m_forceColorLoc = m_shader.uniformLocation("uForceColor");
    m_forcedColorLoc = m_shader.uniformLocation("uForcedColor");
    m_forcedAlphaLoc = m_shader.uniformLocation("uForcedAlpha");
    m_pointSizeLoc = m_shader.uniformLocation("uPointSize");

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    configureVertexArray(m_vao, m_vbo);

    glGenVertexArrays(1, &m_overlayVao);
    glGenBuffers(1, &m_overlayVbo);
    configureVertexArray(m_overlayVao, m_overlayVbo);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    return true;
}

void Visualizer::updatePoints(const BaseLidarSensor::PointCloud& points)
{
    std::vector<Vertex> ground;
    std::vector<Vertex> nonGround;
    BaseLidarSensor::PointCloud nonGroundPoints;
    ground.reserve(points.size());
    nonGround.reserve(points.size());
    nonGroundPoints.reserve(points.size());

    const bool useZoneColors = m_cameraMode == CameraMode::FreeOrbit;
    m_closestContourDistance = std::numeric_limits<float>::max();
    float cloudMinX = std::numeric_limits<float>::max();
    float cloudMaxX = -std::numeric_limits<float>::max();
    float cloudMinY = std::numeric_limits<float>::max();
    float cloudMaxY = -std::numeric_limits<float>::max();
    for (const auto& point : points)
    {
        // Shift LiDAR samples from the sensor frame back into the vehicle frame (front bumper origin).
        const glm::vec2 translatedPosition{
            point.x - m_lidarSensorOffset.x,
            point.y - m_lidarSensorOffset.y};
        lidar::LidarPoint translatedPoint = point;
        translatedPoint.x = translatedPosition.x;
        translatedPoint.y = translatedPosition.y;

        const bool groundPoint = isGroundPoint(point);
        float classification = groundPoint ? 0.0F : 1.0F;
        if (useZoneColors)
        {
            classification = static_cast<float>(zoneIndexFromHeight(point.z));
        }
        if (!groundPoint && !m_translatedContour.empty())
        {
            const float contourDist = distanceToContour(translatedPosition);
            if (contourDist < m_closestContourDistance)
            {
                m_closestContourDistance = contourDist;
                m_closestContourPoint = translatedPosition;
            }
        }
        Vertex vertex{
            translatedPoint.x,
            translatedPoint.y,
            point.z,
            point.intensity,
            classification,
        };

        if (groundPoint)
        {
            ground.push_back(vertex);
        }
        else
        {
            nonGround.push_back(vertex);
            if (point.z >= m_floorHeight)
            {
                nonGroundPoints.push_back(point);
            }
        }

        cloudMinX = std::min(cloudMinX, translatedPoint.x);
        cloudMaxX = std::max(cloudMaxX, translatedPoint.x);
        cloudMinY = std::min(cloudMinY, translatedPoint.y);
        cloudMaxY = std::max(cloudMaxY, translatedPoint.y);
    }

    m_virtualSensorMapping.updatePoints(nonGroundPoints);

    m_groundPointCount = ground.size();
    m_nonGroundPointCount = nonGround.size();

    m_vertexBuffer.clear();
    m_vertexBuffer.reserve(ground.size() + nonGround.size());
    m_vertexBuffer.insert(m_vertexBuffer.end(), ground.begin(), ground.end());
    m_vertexBuffer.insert(m_vertexBuffer.end(), nonGround.begin(), nonGround.end());

    if (m_vertexBuffer.size() > m_gpuCapacity)
    {
        m_gpuCapacity = m_vertexBuffer.size();
        m_needsReallocation = true;
    }

    if (!m_vertexBuffer.empty())
    {
        const auto [minIt, maxIt] = std::minmax_element(
            m_vertexBuffer.begin(),
            m_vertexBuffer.end(),
            [](const Vertex& lhs, const Vertex& rhs) { return lhs.z < rhs.z; });
        m_minHeight = minIt->z;
        m_maxHeight = maxIt->z;
        if (std::fabs(m_maxHeight - m_minHeight) < 1e-3F)
        {
            m_maxHeight = m_minHeight + 1e-3F;
        }
    }

    if (cloudMinX <= cloudMaxX && cloudMinY <= cloudMaxY)
    {
        const glm::vec2 defaultMin(-kGridHalfSpan);
        const glm::vec2 defaultMax(kGridHalfSpan);
        m_gridMin = glm::vec2(
            std::min(cloudMinX, defaultMin.x),
            std::min(cloudMinY, defaultMin.y));
        m_gridMax = glm::vec2(
            std::max(cloudMaxX, defaultMax.x),
            std::max(cloudMaxY, defaultMax.y));
    }
    else
    {
        m_gridMin = glm::vec2(-kGridHalfSpan);
        m_gridMax = glm::vec2(kGridHalfSpan);
    }

    uploadBuffer();
}

void Visualizer::render()
{
    glfwPollEvents();

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    if (m_window)
    {
        glfwGetFramebufferSize(m_window, &framebufferWidth, &framebufferHeight);
        glViewport(0, 0, std::max(framebufferWidth, 1), std::max(framebufferHeight, 1));
    }

    glClearColor(0.05F, 0.05F, 0.08F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_shader.use();
    applyUniforms();

    if (m_worldFrameSettings.enableWorldVisualization && !m_vertexBuffer.empty())
    {
        GLboolean depthMask = GL_TRUE;
        glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
        glDepthMask(GL_FALSE);
        glPointSize(m_worldFrameSettings.pointSize);

        glBindVertexArray(m_vao);
        if (m_worldFrameSettings.enableGroundPlane && m_groundPointCount > 0)
        {
            glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_groundPointCount));
        }
        if (m_worldFrameSettings.enableNonGroundPlane && m_nonGroundPointCount > 0)
        {
            glDrawArrays(
                GL_POINTS,
                static_cast<GLsizei>(m_groundPointCount),
                static_cast<GLsizei>(m_nonGroundPointCount));
        }
        glDepthMask(depthMask);
    }

    if (m_worldFrameSettings.enableWorldVisualization)
    {
        drawGrid(m_gridSpacing);
    }

    if (m_worldFrameSettings.enableWorldVisualization && m_worldFrameSettings.showVirtualSensorMap)
    {
        drawVirtualSensorsFancy();
    }
    if (m_worldFrameSettings.enableWorldVisualization && m_worldFrameSettings.showFreeSpaceMap)
    {
        drawFreeSpaceMap();
    }

    if (m_worldFrameSettings.enableWorldVisualization && m_worldFrameSettings.showVehicleContour)
    {
        const std::vector<glm::vec2>* baseContour = m_translatedContour.empty() ? &m_vehicleContour : &m_translatedContour;
        if (!baseContour->empty())
        {
            const float rotationDegrees = m_worldFrameSettings.vehicleContourRotation;
            const float epsilon = 1e-3F;
            const bool needsRotation = std::fabs(rotationDegrees) > epsilon;
            const float rotationRadians = glm::radians(rotationDegrees);
            const float cosValue = std::cos(rotationRadians);
            const float sinValue = std::sin(rotationRadians);
            auto rotatePoint = [&](const glm::vec2& value) -> glm::vec2 {
                if (!needsRotation)
                {
                    return value;
                }
                return glm::vec2(
                    cosValue * value.x - sinValue * value.y,
                    sinValue * value.x + cosValue * value.y);
            };

            std::vector<glm::vec2> rotatedContour;
            const std::vector<glm::vec2>* contourToDraw = baseContour;
            if (needsRotation)
            {
                rotatedContour.reserve(baseContour->size());
                for (const auto& point : *baseContour)
                {
                    rotatedContour.push_back(rotatePoint(point));
                }
                contourToDraw = &rotatedContour;
            }

            const auto& color = m_worldFrameSettings.vehicleContourColor;
            drawOverlayPolygon(
                *contourToDraw,
                glm::vec3(color[0], color[1], color[2]),
                m_worldFrameSettings.vehicleContourTransparency);

            const glm::vec2 rotatedLidarPosition = rotatePoint(m_lidarVcsPosition);
            drawLidarMountMarker(rotatedLidarPosition, rotationDegrees);
            if (m_closestContourDistance < std::numeric_limits<float>::max())
            {
                const glm::vec2 rotatedClosest = rotatePoint(m_closestContourPoint);
                const float crossSize = 0.3F;
                const glm::vec3 closeColor(1.0F, 0.25F, 0.25F);
                drawOverlayLine(
                    glm::vec2(rotatedClosest.x - crossSize, rotatedClosest.y),
                    glm::vec2(rotatedClosest.x + crossSize, rotatedClosest.y),
                    closeColor,
                    0.85F);
                drawOverlayLine(
                    glm::vec2(rotatedClosest.x, rotatedClosest.y - crossSize),
                    glm::vec2(rotatedClosest.x, rotatedClosest.y + crossSize),
                    closeColor,
                    0.85F);
            }
        }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    drawWorldControls();

    ImGui::Begin("LiDAR Stats");
    ImGui::Text("Total points: %zu", m_vertexBuffer.size());
    ImGui::Text("Ground points: %zu", m_groundPointCount);
    ImGui::Text("Non-ground points: %zu", m_nonGroundPointCount);
    ImGui::Text("GPU capacity: %zu", m_gpuCapacity);
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(m_window);
}

bool Visualizer::windowShouldClose() const
{
    return m_window && glfwWindowShouldClose(m_window);
}

void Visualizer::uploadBuffer()
{
    if (m_vertexBuffer.empty())
    {
        return;
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    const auto byteCount = m_vertexBuffer.size() * sizeof(Vertex);
    if (m_needsReallocation)
    {
        glBufferData(GL_ARRAY_BUFFER, m_gpuCapacity * sizeof(Vertex), m_vertexBuffer.data(), GL_DYNAMIC_DRAW);
        m_needsReallocation = false;
    }
    else
    {
        glBufferSubData(GL_ARRAY_BUFFER, 0, byteCount, m_vertexBuffer.data());
    }
}

void Visualizer::drawFreeSpaceMap()
{
    const auto snapshots = m_virtualSensorMapping.snapshots();
    if (snapshots.empty())
    {
        return;
    }

    const glm::vec3 freespaceColor(1.0F, 0.9F, 0.0F);
    for (const auto& snapshot : snapshots)
    {
        float farRange;
        if (snapshot.valid)
        {
            farRange = glm::clamp(std::sqrt(snapshot.distanceSquared), 0.0F, kVirtualSensorMaxRange);
        }
        else
        {
            farRange = kVirtualSensorMaxRange;
        }

        const auto polygon = buildFreeSpacePolygon(snapshot, farRange);
        if (polygon.size() < 3)
        {
            continue;
        }

        const float alpha = snapshot.valid ? 0.35F : 0.15F;
        const glm::vec3& color = freespaceColor;
        drawOverlayPolygon(polygon, color, alpha);

        if (snapshot.valid)
        {
            drawOverlayLine(polygon[2], polygon[3], color, 0.9F);
        }
    }
}

void Visualizer::cleanUp()
{
    if (m_vbo)
    {
        glDeleteBuffers(1, &m_vbo);
        m_vbo = 0;
    }

    if (m_vao)
    {
        glDeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }

    if (m_overlayVbo)
    {
        glDeleteBuffers(1, &m_overlayVbo);
        m_overlayVbo = 0;
    }

    if (m_overlayVao)
    {
        glDeleteVertexArrays(1, &m_overlayVao);
        m_overlayVao = 0;
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (m_window)
    {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }

    glfwTerminate();
}

void Visualizer::applyUniforms()
{
    const GLint minHeightLoc = m_shader.uniformLocation("uMinHeight");
    if (minHeightLoc >= 0)
    {
        glUniform1f(minHeightLoc, m_minHeight);
    }

    const GLint maxHeightLoc = m_shader.uniformLocation("uMaxHeight");
    if (maxHeightLoc >= 0)
    {
        glUniform1f(maxHeightLoc, m_maxHeight);
    }

    const GLint colorModeLoc = m_shader.uniformLocation("uColorMode");
    if (colorModeLoc >= 0)
    {
        glUniform1i(colorModeLoc, static_cast<GLint>(m_worldFrameSettings.colorMode));
    }

    const GLint alphaModeLoc = m_shader.uniformLocation("uAlphaMode");
    if (alphaModeLoc >= 0)
    {
        glUniform1i(alphaModeLoc, static_cast<GLint>(m_worldFrameSettings.alphaMode));
    }

    const float clipValue = (m_worldFrameSettings.colorMode == ColorMode::Height)
                                ? m_worldFrameSettings.clipHeight
                                : m_worldFrameSettings.clipIntensity;
    const GLint clipLoc = m_shader.uniformLocation("uClipValue");
    if (clipLoc >= 0)
    {
        glUniform1f(clipLoc, clipValue);
    }

    const GLint groundColorLoc = m_shader.uniformLocation("uGroundColor");
    if (groundColorLoc >= 0)
    {
        glUniform3fv(groundColorLoc, 1, m_worldFrameSettings.groundPlaneColor.data());
    }

    const GLint nonGroundColorLoc = m_shader.uniformLocation("uNonGroundColor");
    if (nonGroundColorLoc >= 0)
    {
        glUniform3fv(nonGroundColorLoc, 1, m_worldFrameSettings.nonGroundPlaneColor.data());
    }

    const GLint commonAlphaLoc = m_shader.uniformLocation("uCommonAlpha");
    if (commonAlphaLoc >= 0)
    {
        glUniform1f(commonAlphaLoc, m_worldFrameSettings.commonTransparency);
    }

    const GLint groundAlphaLoc = m_shader.uniformLocation("uGroundPlaneAlpha");
    if (groundAlphaLoc >= 0)
    {
        glUniform1f(groundAlphaLoc, m_worldFrameSettings.groundPlaneTransparency);
    }

    const GLint nonGroundAlphaLoc = m_shader.uniformLocation("uNonGroundPlaneAlpha");
    if (nonGroundAlphaLoc >= 0)
    {
        glUniform1f(nonGroundAlphaLoc, m_worldFrameSettings.nongroundPlaneTransparency);
    }

    const GLint zoneColorsLoc = m_shader.uniformLocation("uZoneColors");
    if (zoneColorsLoc >= 0)
    {
        std::array<GLfloat, kZoneColors.size() * 3> zoneColorData{};
        for (size_t i = 0; i < kZoneColors.size(); ++i)
        {
            zoneColorData[i * 3 + 0] = kZoneColors[i].r;
            zoneColorData[i * 3 + 1] = kZoneColors[i].g;
            zoneColorData[i * 3 + 2] = kZoneColors[i].b;
        }
        glUniform3fv(zoneColorsLoc, static_cast<GLsizei>(kZoneColors.size()), zoneColorData.data());
    }

    const GLint useZoneColorsLoc = m_shader.uniformLocation("uUseZoneColors");
    if (useZoneColorsLoc >= 0)
    {
        const bool useZoneColors =
            m_cameraMode == CameraMode::FreeOrbit && m_worldFrameSettings.colorMode == ColorMode::Classification;
        glUniform1i(useZoneColorsLoc, useZoneColors ? GL_TRUE : GL_FALSE);
    }

    if (m_forceColorLoc >= 0)
    {
        glUniform1i(m_forceColorLoc, GL_FALSE);
    }

    const GLint viewProjLoc = m_shader.uniformLocation("uViewProjection");
    if (viewProjLoc >= 0 && m_window)
    {
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(m_window, &width, &height);
        if (height == 0)
        {
            height = 1;
        }

        const float aspect = static_cast<float>(width) / static_cast<float>(height);
        const glm::mat4 projection =
            glm::perspective(glm::radians(m_camera.fov), aspect, 0.1F, 1000.0F);

        const glm::vec3 direction = computeCameraDirection();
        const glm::vec3 cameraPos = -direction * m_camera.distance;
        const glm::vec3 up = computeCameraUp();
        const glm::mat4 view = glm::lookAt(cameraPos, glm::vec3(0.0F), up);
        const glm::mat4 viewProj = projection * view;
        glUniformMatrix4fv(viewProjLoc, 1, GL_FALSE, glm::value_ptr(viewProj));
    }

    if (m_pointSizeLoc >= 0)
    {
        glUniform1f(m_pointSizeLoc, m_worldFrameSettings.pointSize);
    }
}

void Visualizer::drawWorldControls()
{
    ImGui::Begin("LiDAR Controls");
    if (ImGui::TreeNodeEx("General", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Enable visualization", &m_worldFrameSettings.enableWorldVisualization);
        ImGui::Checkbox("Show virtual sensor map", &m_worldFrameSettings.showVirtualSensorMap);
        ImGui::Checkbox("Show free-space map", &m_worldFrameSettings.showFreeSpaceMap);
        ImGui::Checkbox("Show vehicle contour", &m_worldFrameSettings.showVehicleContour);
        if (!m_vehicleProfileEntries.empty())
        {
            int profileIdx = m_selectedVehicleProfileIndex;
            if (ImGui::Combo(
                    "Vehicle profile",
                    &profileIdx,
                    vehicleProfileItemsGetter,
                    &m_vehicleProfileEntries,
                    static_cast<int>(m_vehicleProfileEntries.size())))
            {
    applyVehicleProfile(profileIdx);
}
        }
        if (m_worldFrameSettings.showVehicleContour)
        {
            ImGuiColorEditFlags colorFlags = ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_Float;
            auto& contourColor = m_worldFrameSettings.vehicleContourColor;
            ImGui::ColorEdit3("Vehicle contour color", contourColor.data(), colorFlags);
            ImGui::SliderFloat(
                "Contour transparency",
                &m_worldFrameSettings.vehicleContourTransparency,
                0.1F,
                1.0F);
            ImGui::SliderFloat(
                "Contour rotation",
                &m_worldFrameSettings.vehicleContourRotation,
                -180.0F,
                180.0F,
                "%.0f\u00B0");
            ImGui::Spacing();
        }
        ImGui::Separator();

        ImGui::SliderFloat("Point size", &m_worldFrameSettings.pointSize, 1.0F, 6.0F);
        ImGui::SliderFloat("Bin size (m)", &m_gridSpacing, 10.0F, 100.0F, "%.0f");

        int cameraModeIdx = static_cast<int>(m_cameraMode);
        if (ImGui::Combo(
                "Camera view",
                &cameraModeIdx,
                kCameraModeLabels.data(),
                static_cast<int>(kCameraModeLabels.size())))
        {
            m_cameraMode = static_cast<CameraMode>(cameraModeIdx);
            m_camera.rotating = false;
            m_activeMouseButton = -1;
        }

        ImGui::SliderFloat("Camera distance", &m_camera.distance, 0.5F, 200.0F);
        ImGui::SliderFloat(
            "Replay speed",
            &m_worldFrameSettings.replaySpeed,
            0.1F,
            2.5F,
            "%.2f");

        int colorModeIdx = static_cast<int>(m_worldFrameSettings.colorMode);
        if (ImGui::Combo("Color mode", &colorModeIdx, kColorModeLabels.data(), static_cast<int>(kColorModeLabels.size())))
        {
            m_worldFrameSettings.colorMode = static_cast<ColorMode>(colorModeIdx);
        }

        int alphaModeIdx = static_cast<int>(m_worldFrameSettings.alphaMode);
        if (ImGui::Combo("Alpha mode", &alphaModeIdx, kAlphaModeLabels.data(), static_cast<int>(kAlphaModeLabels.size())))
        {
            m_worldFrameSettings.alphaMode = static_cast<AlphaMode>(alphaModeIdx);
        }

        if (m_worldFrameSettings.colorMode == ColorMode::Height)
        {
            ImGui::SliderFloat("Clip height", &m_worldFrameSettings.clipHeight, 1.0F, 10.0F);
        }
        if (m_worldFrameSettings.colorMode == ColorMode::Intensity)
        {
            ImGui::SliderFloat("Clip intensity", &m_worldFrameSettings.clipIntensity, 0.1F, 3.0F);
        }

        if (m_worldFrameSettings.colorMode != ColorMode::Classification &&
            m_worldFrameSettings.alphaMode == AlphaMode::UserValue)
        {
            ImGui::SliderFloat(
                "Base transparency", &m_worldFrameSettings.commonTransparency, 0.1F, 1.0F);
        }

        ImGui::SliderFloat(
            "Ground height threshold",
            &m_worldFrameSettings.groundClassificationHeight,
            -2.0F,
            2.0F);
        ImGui::Separator();

        ImGui::Checkbox("Ground plane", &m_worldFrameSettings.enableGroundPlane);
        ImGui::Checkbox("Non-ground plane", &m_worldFrameSettings.enableNonGroundPlane);

        if (m_worldFrameSettings.colorMode == ColorMode::Classification)
        {
            ImGuiColorEditFlags colorFlags = ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_Float;

            std::array<float, 3> groundColor = m_worldFrameSettings.groundPlaneColor;
            if (ImGui::ColorEdit3("Ground color", groundColor.data(), colorFlags))
            {
                m_worldFrameSettings.groundPlaneColor = groundColor;
            }

            if (m_worldFrameSettings.alphaMode == AlphaMode::UserValue)
            {
                ImGui::SliderFloat(
                    "Ground transparency",
                    &m_worldFrameSettings.groundPlaneTransparency,
                    0.1F,
                    1.0F);
            }

            std::array<float, 3> nonGroundColor = m_worldFrameSettings.nonGroundPlaneColor;
            if (ImGui::ColorEdit3("Non-ground color", nonGroundColor.data(), colorFlags))
            {
                m_worldFrameSettings.nonGroundPlaneColor = nonGroundColor;
            }

            if (m_worldFrameSettings.alphaMode == AlphaMode::UserValue)
            {
                ImGui::SliderFloat(
                    "Non-ground transparency",
                    &m_worldFrameSettings.nongroundPlaneTransparency,
                    0.1F,
                    1.0F);
            }
        }

        ImGui::Spacing();
        drawColorLegend();
        ImGui::TreePop();
    }
    ImGui::End();
}

void Visualizer::drawVirtualSensorsFancy()
{
    const auto snapshots = m_virtualSensorMapping.snapshots();
    if (snapshots.empty())
    {
        return;
    }

    GLboolean depthMask = GL_TRUE;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
    glDepthMask(GL_FALSE);

    const glm::vec3 shadowColor(0.55F, 0.15F, 0.85F);
    const glm::vec3 measurementColor(1.0F, 0.25F, 0.65F);
    const glm::vec3 pointColor(0.95F, 0.55F, 0.9F);
    for (const auto& snapshot : snapshots)
    {
        if (!snapshot.valid)
        {
            continue;
        }

        drawOverlayPolygon(buildSensorShadowPolygon(snapshot), shadowColor, 0.12F);
        drawOverlayPolygon(buildSensorMeasurementPolygon(snapshot), measurementColor, 0.7F);
        drawSensorPoint(snapshot, pointColor, 1.0F);
    }

    if (m_pointSizeLoc >= 0)
    {
        glUniform1f(m_pointSizeLoc, m_worldFrameSettings.pointSize);
    }

    glDepthMask(depthMask);

    const auto& groundHull = m_virtualSensorMapping.groundHull();
    if (groundHull.size() >= 3)
    {
        drawOverlayPolygon(groundHull, glm::vec3(0.3F, 0.5F, 1.0F), 0.45F);
    }
}

void Visualizer::configureVertexArray(GLuint vao, GLuint vbo)
{
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        static_cast<GLsizei>(sizeof(Vertex)),
        reinterpret_cast<void*>(offsetof(Vertex, x)));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1,
        1,
        GL_FLOAT,
        GL_FALSE,
        static_cast<GLsizei>(sizeof(Vertex)),
        reinterpret_cast<void*>(offsetof(Vertex, intensity)));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(
        2,
        1,
        GL_FLOAT,
        GL_FALSE,
        static_cast<GLsizei>(sizeof(Vertex)),
        reinterpret_cast<void*>(offsetof(Vertex, classification)));

    glBindVertexArray(0);
}

bool Visualizer::isGroundPoint(const lidar::LidarPoint& point) const noexcept
{
    return point.z <= m_worldFrameSettings.groundClassificationHeight;
}

int Visualizer::zoneIndexFromHeight(float height) const noexcept
{
    for (size_t i = 0; i < kZoneThresholds.size(); ++i)
    {
        if (height < kZoneThresholds[i])
        {
            return static_cast<int>(i);
        }
    }
    return static_cast<int>(kZoneColors.size() - 1);
}

void Visualizer::processCursorPos(double xpos, double ypos)
{
    if (m_cameraMode != CameraMode::FreeOrbit || !m_camera.rotating || m_activeMouseButton == -1)
    {
        m_camera.lastX = xpos;
        m_camera.lastY = ypos;
        return;
    }

    const float dx = static_cast<float>(xpos - m_camera.lastX);
    const float dy = static_cast<float>(ypos - m_camera.lastY);
    m_camera.lastX = xpos;
    m_camera.lastY = ypos;

    m_camera.yaw += dx * 0.35F;
    m_camera.pitch -= dy * 0.35F;
    m_camera.pitch = std::clamp(m_camera.pitch, -89.0F, 89.0F);
}

void Visualizer::processScroll(double yoffset)
{
    m_camera.distance = std::clamp(m_camera.distance - static_cast<float>(yoffset) * kScrollSpeed, 0.5F, 200.0F);
}

void Visualizer::processMouseButton(int button, int action)
{
    if (m_cameraMode != CameraMode::FreeOrbit)
    {
        return;
    }

    const bool rotationButton =
        button == GLFW_MOUSE_BUTTON_RIGHT || button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_MIDDLE;

    if (!rotationButton)
    {
        return;
    }

    if (ImGui::GetIO().WantCaptureMouse && action == GLFW_PRESS)
    {
        return;
    }

    if (action == GLFW_PRESS)
    {
        m_camera.rotating = true;
        m_activeMouseButton = button;
        if (m_window)
        {
            glfwGetCursorPos(m_window, &m_camera.lastX, &m_camera.lastY);
        }
    }
    else if (action == GLFW_RELEASE && button == m_activeMouseButton)
    {
        m_camera.rotating = false;
        m_activeMouseButton = -1;
    }
}

void Visualizer::cursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
    if (auto* self = reinterpret_cast<Visualizer*>(glfwGetWindowUserPointer(window)))
    {
        self->processCursorPos(xpos, ypos);
    }
}

void Visualizer::scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    if (auto* self = reinterpret_cast<Visualizer*>(glfwGetWindowUserPointer(window)))
    {
        self->processScroll(static_cast<float>(yoffset));
    }
}

void Visualizer::mouseButtonCallback(GLFWwindow* window, int button, int action, int /*mods*/)
{
    if (auto* self = reinterpret_cast<Visualizer*>(glfwGetWindowUserPointer(window)))
    {
        self->processMouseButton(button, action);
    }
}

void Visualizer::drawColorLegend()
{
    if (m_worldFrameSettings.colorMode == ColorMode::Classification)
    {
        ImGui::Text("Altitude zones:");
        for (size_t i = 0; i < kZoneColors.size(); ++i)
        {
            ImGui::PushID(static_cast<int>(i));
            ImGui::ColorButton(
                "zone_color",
                ImVec4(kZoneColors[i].r, kZoneColors[i].g, kZoneColors[i].b, 1.0F),
                ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoTooltip,
                ImVec2(16.0F, 16.0F));
            ImGui::PopID();
            ImGui::SameLine();
            ImGui::Text("%s", kZoneLabels[i]);
        }
    }
    else
    {
        ImGui::Text("Color scale:");
        const float width = 180.0F;
        const float height = 10.0F;
        const ImVec2 pos = ImGui::GetCursorScreenPos();
        const auto drawList = ImGui::GetWindowDrawList();
        const int segments = 32;
        for (int i = 0; i < segments; ++i)
        {
            const float t0 = static_cast<float>(i) / segments;
            const float t1 = static_cast<float>(i + 1) / segments;
            const float sample = (t0 + t1) * 0.5F;
            glm::vec3 color = (m_worldFrameSettings.colorMode == ColorMode::Height)
                                  ? sampleHeightColor(sample)
                                  : sampleIntensityColor(sample);
            drawList->AddRectFilled(
                ImVec2(pos.x + width * t0, pos.y),
                ImVec2(pos.x + width * t1, pos.y + height),
                ImGui::ColorConvertFloat4ToU32(ImVec4(color.r, color.g, color.b, 1.0F)));
        }
        ImGui::Dummy(ImVec2(width, height));

        if (m_worldFrameSettings.colorMode == ColorMode::Height)
        {
            ImGui::Text("Min height %.2f  Max height %.2f", m_minHeight, m_maxHeight);
        }
        else if (m_worldFrameSettings.colorMode == ColorMode::Intensity)
        {
            const float clip = std::max(0.001F, m_worldFrameSettings.clipIntensity);
            ImGui::Text("Intensity: 0.00 → %.2f", clip);
        }
    }
}

glm::vec2 Visualizer::directionFromAngle(float angle) const
{
    return glm::vec2(std::cos(angle), std::sin(angle));
}

std::vector<glm::vec2> Visualizer::buildSensorPolygon(
    const mapping::LidarVirtualSensorMapping::SensorSnapshot& snapshot,
    float nearRange,
    float farRange) const
{
    const float normalizedNear = std::min(nearRange, farRange);
    const float normalizedFar = std::max(nearRange, farRange);
    if (normalizedFar <= 0.0F)
    {
        return {};
    }

    if (snapshot.isAngular)
    {
        const glm::vec2 lowerDir = directionFromAngle(snapshot.lowerAngle);
        const glm::vec2 upperDir = directionFromAngle(snapshot.upperAngle);
        const glm::vec2 nearLower = snapshot.reference + lowerDir * normalizedNear;
        const glm::vec2 nearUpper = snapshot.reference + upperDir * normalizedNear;
        const glm::vec2 farUpper = snapshot.reference + upperDir * normalizedFar;
        const glm::vec2 farLower = snapshot.reference + lowerDir * normalizedFar;
        return {nearLower, nearUpper, farUpper, farLower};
    }

    const glm::vec2 baseLower = snapshot.reference + glm::vec2(snapshot.orthMinX, 0.0F);
    const glm::vec2 baseUpper = snapshot.reference + glm::vec2(snapshot.orthMaxX, 0.0F);
    const float side = snapshot.orthSideSign != 0.0F ? snapshot.orthSideSign : 1.0F;
    const glm::vec2 nearOffset(0.0F, side * normalizedNear);
    const glm::vec2 farOffset(0.0F, side * normalizedFar);
    return {baseLower + nearOffset, baseUpper + nearOffset, baseUpper + farOffset, baseLower + farOffset};
}

std::vector<glm::vec2> Visualizer::buildSensorMeasurementPolygon(
    const mapping::LidarVirtualSensorMapping::SensorSnapshot& snapshot) const
{
    float farRange;
    if (snapshot.isAngular)
    {
        farRange = glm::clamp(std::sqrt(snapshot.distanceSquared), 0.0F, kVirtualSensorMaxRange);
    }
    else
    {
        farRange = glm::clamp(std::abs(snapshot.position.y - snapshot.reference.y), 0.0F, kVirtualSensorMaxRange);
    }
    const float nearRange = std::max(farRange - kVirtualSensorThickness, 0.0F);
    return buildSensorPolygon(snapshot, nearRange, farRange);
}

std::vector<glm::vec2> Visualizer::buildSensorShadowPolygon(
    const mapping::LidarVirtualSensorMapping::SensorSnapshot& snapshot) const
{
    return buildSensorPolygon(snapshot, 0.0F, kVirtualSensorMaxRange);
}

std::vector<glm::vec2> Visualizer::buildFreeSpacePolygon(
    const mapping::LidarVirtualSensorMapping::SensorSnapshot& snapshot,
    float farRange) const
{
    return buildSensorPolygon(snapshot, 0.0F, farRange);
}

void Visualizer::drawOverlayPolygon(const std::vector<glm::vec2>& positions,
                                    const glm::vec3& color,
                                    float alpha)
{
    if (positions.size() < 3)
    {
        return;
    }

    std::vector<Vertex> vertices;
    vertices.reserve(positions.size());
    for (const auto& position : positions)
    {
        vertices.push_back(Vertex{position.x, position.y, 0.0F, 0.0F, 0.0F});
    }

    glBindVertexArray(m_overlayVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_overlayVbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                 vertices.data(),
                 GL_DYNAMIC_DRAW);

    applyForceColor(color, alpha);
    glLineWidth(1.5F);
    glDrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(vertices.size()));
    resetForceColor();
}

void Visualizer::drawOverlayLine(const glm::vec2& from,
                                 const glm::vec2& to,
                                 const glm::vec3& color,
                                 float alpha,
                                 float elevation)
{
    Vertex vertices[2] = {
        Vertex{from.x, from.y, elevation, 0.0F, 0.0F},
        Vertex{to.x, to.y, elevation, 0.0F, 0.0F},
    };

    glBindVertexArray(m_overlayVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_overlayVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);

    applyForceColor(color, alpha);
    glLineWidth(2.0F);
    glDrawArrays(GL_LINES, 0, 2);
    resetForceColor();
}

void Visualizer::drawLidarMountMarker(const glm::vec2& position, float rotationDegrees)
{
    const glm::vec3 markerColor(0.1F, 0.95F, 0.35F);
    const float crossHalfSize = 0.3F;
    drawOverlayLine(
        glm::vec2(position.x - crossHalfSize, position.y),
        glm::vec2(position.x + crossHalfSize, position.y),
        markerColor,
        0.8F);
    drawOverlayLine(
        glm::vec2(position.x, position.y - crossHalfSize),
        glm::vec2(position.x, position.y + crossHalfSize),
        markerColor,
        0.8F);

    const float orientationDegrees = m_lidarOrientationIsoDeg + rotationDegrees;
    const float orientationRad = glm::radians(orientationDegrees);
    const glm::vec2 direction(std::cos(orientationRad), std::sin(orientationRad));
    const float arrowLength = 0.6F;
    drawOverlayLine(position, position + direction * arrowLength, glm::vec3(1.0F, 0.85F, 0.05F), 0.9F);
}

void Visualizer::drawSensorPoint(const mapping::LidarVirtualSensorMapping::SensorSnapshot& snapshot,
                                 const glm::vec3& color,
                                 float alpha)
{
    Vertex vertex{snapshot.position.x, snapshot.position.y, 0.0F, 0.0F, 0.0F};
    glBindVertexArray(m_overlayVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_overlayVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex), &vertex, GL_DYNAMIC_DRAW);

    applyForceColor(color, alpha);
    if (m_pointSizeLoc >= 0)
    {
        glUniform1f(m_pointSizeLoc, kVirtualSensorPointSize);
    }
    glDrawArrays(GL_POINTS, 0, 1);
    resetForceColor();

    if (m_pointSizeLoc >= 0)
    {
        glUniform1f(m_pointSizeLoc, m_worldFrameSettings.pointSize);
    }
}

void Visualizer::drawGrid(float spacing)
{
    const float gridSpacing = std::max(0.01F, spacing);
    const glm::vec2 minBounds = m_gridMin;
    const glm::vec2 maxBounds = m_gridMax;
    const float startX = std::floor(minBounds.x / gridSpacing) * gridSpacing;
    const float endX = std::ceil(maxBounds.x / gridSpacing) * gridSpacing;
    const float startY = std::floor(minBounds.y / gridSpacing) * gridSpacing;
    const float endY = std::ceil(maxBounds.y / gridSpacing) * gridSpacing;

    const glm::vec3 gridColor(0.35F, 0.35F, 0.35F);
    const float alpha = 0.2F;

    for (float x = startX; x <= endX; x += gridSpacing)
    {
        drawOverlayLine(glm::vec2(x, startY), glm::vec2(x, endY), gridColor, alpha, m_floorHeight);
    }
    for (float y = startY; y <= endY; y += gridSpacing)
    {
        drawOverlayLine(glm::vec2(startX, y), glm::vec2(endX, y), gridColor, alpha, m_floorHeight);
    }
}

void Visualizer::applyForceColor(const glm::vec3& color, float alpha)
{
    if (m_forceColorLoc >= 0)
    {
        glUniform1i(m_forceColorLoc, GL_TRUE);
    }
    if (m_forcedColorLoc >= 0)
    {
        glUniform3f(m_forcedColorLoc, color.r, color.g, color.b);
    }
    if (m_forcedAlphaLoc >= 0)
    {
        glUniform1f(m_forcedAlphaLoc, alpha);
    }
}

void Visualizer::resetForceColor()
{
    if (m_forceColorLoc >= 0)
    {
        glUniform1i(m_forceColorLoc, GL_FALSE);
    }
    if (m_forcedColorLoc >= 0)
    {
        glUniform3f(m_forcedColorLoc, 0.0F, 0.0F, 0.0F);
    }
    if (m_forcedAlphaLoc >= 0)
    {
        glUniform1f(m_forcedAlphaLoc, 1.0F);
    }
}

void Visualizer::updateContourTranslation()
{
    m_translatedContour.clear();
    m_translatedContour.reserve(m_vehicleContour.size());
    for (const auto& point : m_vehicleContour)
    {
        m_translatedContour.push_back(point + m_contourTranslation);
    }
    updateSensorOffsets();
}

void Visualizer::updateSensorOffsets()
{
    if (m_translatedContour.empty())
    {
        return;
    }

    m_virtualSensorMapping.setVehicleContour(m_translatedContour);
}

float Visualizer::distanceToContour(const glm::vec2& point) const
{
    if (m_translatedContour.size() < 2)
    {
        return std::numeric_limits<float>::max();
    }

    float best = std::numeric_limits<float>::max();
    for (std::size_t idx = 0; idx < m_translatedContour.size(); ++idx)
    {
        const auto& start = m_translatedContour[idx];
        const auto& end = m_translatedContour[(idx + 1) % m_translatedContour.size()];
        best = std::min(best, distanceToSegment(start, end, point));
    }
    return best;
}

float Visualizer::distanceToSegment(const glm::vec2& a, const glm::vec2& b, const glm::vec2& point) const
{
    const glm::vec2 ab = b - a;
    const float abSquared = glm::dot(ab, ab);
    if (abSquared < 1e-6F)
    {
        return glm::length(point - a);
    }

    float t = glm::dot(point - a, ab) / abSquared;
    t = std::clamp(t, 0.0F, 1.0F);
    const glm::vec2 projection = a + ab * t;
    return glm::length(point - projection);
}

void Visualizer::refreshVehicleProfiles()
{
    std::vector<std::string> entries;
    if (fs::exists(kVehicleProfileDirectory) && fs::is_directory(kVehicleProfileDirectory))
    {
        for (const auto& entry : fs::directory_iterator(kVehicleProfileDirectory))
        {
            if (!entry.is_regular_file())
            {
                continue;
            }

            const auto filename = entry.path().filename().string();
            if (!filename.starts_with(kVehicleProfilePrefix) ||
                entry.path().extension() != ".ini")
            {
                continue;
            }

            entries.push_back(filename);
        }
    }

    if (entries.empty())
    {
        entries.push_back(kDefaultVehicleProfileFilename);
    }

    std::sort(entries.begin(), entries.end());

    const auto defaultIt = std::find(entries.begin(), entries.end(), kDefaultVehicleProfileFilename);
    if (defaultIt != entries.end())
    {
        m_selectedVehicleProfileIndex = static_cast<int>(std::distance(entries.begin(), defaultIt));
    }
    else if (m_selectedVehicleProfileIndex >= static_cast<int>(entries.size()))
    {
        m_selectedVehicleProfileIndex = 0;
    }

    m_vehicleProfileEntries = std::move(entries);
}

void Visualizer::applyVehicleProfile(int index)
{
    if (m_vehicleProfileEntries.empty())
    {
        return;
    }

    const int clampedIndex =
        std::clamp(index, 0, static_cast<int>(m_vehicleProfileEntries.size()) - 1);
    m_selectedVehicleProfileIndex = clampedIndex;

    const auto profilePath = kVehicleProfileDirectory / m_vehicleProfileEntries[clampedIndex];
    m_currentVehicleProfile = loadVehicleProfile(profilePath);
    m_vehicleContour = m_currentVehicleProfile.contour;
    m_mountHeight = m_currentVehicleProfile.lidarHeightAboveGround;
    m_floorHeight = -std::fabs(m_mountHeight);
    m_virtualSensorMapping.setFloorHeight(m_floorHeight);
    m_lidarSensorOffset = {
        m_currentVehicleProfile.lidarLatPos,
        -m_currentVehicleProfile.lidarLonPos - m_currentVehicleProfile.distRearAxle};
    m_lidarVcsPosition = -m_lidarSensorOffset;
    m_lidarOrientationIsoDeg = m_currentVehicleProfile.lidarOrientation;
    m_contourTranslation = glm::vec2(0.0F,0.0F);
    m_virtualSensorMapping.setSensorOffset(m_lidarSensorOffset);
    updateContourTranslation();
}

glm::vec3 Visualizer::sampleHeightColor(float normalized) const
{
    const glm::vec3 cool{0.1F, 0.2F, 0.9F};
    const glm::vec3 warm{0.9F, 0.3F, 0.0F};
    const float t = glm::clamp(normalized, 0.0F, 1.0F);
    return glm::mix(cool, warm, t);
}

glm::vec3 Visualizer::sampleIntensityColor(float normalized) const
{
    const glm::vec3 cool{0.1F, 0.9F, 0.35F};
    const glm::vec3 warm{0.9F, 0.3F, 0.0F};
    const float t = glm::clamp(normalized, 0.0F, 1.0F);
    return glm::mix(cool, warm, t);
}

glm::vec3 Visualizer::computeCameraDirection() const
{
    switch (m_cameraMode)
    {
        case CameraMode::BirdsEye:
            return glm::vec3(0.0F, 0.0F, -1.0F);
        case CameraMode::Front:
            return glm::vec3(0.0F, -1.0F, 0.0F);
        case CameraMode::Side:
            return glm::vec3(1.0F, 0.0F, 0.0F);
        case CameraMode::Rear:
            return glm::vec3(0.0F, 1.0F, 0.0F);
        default:
        {
            const float pitchRad = glm::radians(m_camera.pitch);
            const float yawRad = glm::radians(m_camera.yaw);
            return glm::vec3(
                std::cos(pitchRad) * std::cos(yawRad),
                std::cos(pitchRad) * std::sin(yawRad),
                std::sin(pitchRad));
        }
    }
}

glm::vec3 Visualizer::computeCameraUp() const
{
    if (m_cameraMode == CameraMode::BirdsEye)
    {
        return glm::vec3(0.0F, 1.0F, 0.0F);
    }
    return glm::vec3(0.0F, 0.0F, 1.0F);
}

float Visualizer::frameSpeedScale() const
{
    return std::max(0.01F, m_worldFrameSettings.replaySpeed);
}

} // namespace visualization
