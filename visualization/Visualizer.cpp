#include "visualization/Visualizer.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <imgui_impl_glfw.hpp>
#include <imgui_impl_opengl3.hpp>

#include <array>
#include <cmath>
#include <iostream>

namespace visualization
{

namespace
{
constexpr const char* kVertexShaderPath = "shaders/point.vs";
constexpr const char* kFragmentShaderPath = "shaders/point.fs";
constexpr std::array<const char*, 3> kColorModeLabels = {"Classification", "Height", "Intensity"};
constexpr std::array<const char*, 2> kAlphaModeLabels = {"User value", "Intensity"};
constexpr std::array<const char*, 5> kCameraModeLabels = {"Free orbit", "Bird's eye", "Front", "Side", "Rear"};
constexpr float kScrollSpeed = 2.0F;
constexpr std::array<const char*, 5> kZoneLabels = {
    "z < -1.5 m",
    "-1.5 m <= z < 0 m",
    "z = 0 m",
    "0 m < z < 1.5 m",
    "z >= 1.5 m"};
const std::array<glm::vec3, 5> kZoneColors = {
    glm::vec3(0.1F, 0.4F, 0.9F),
    glm::vec3(0.0F, 0.8F, 0.5F),
    glm::vec3(1.0F, 0.9F, 0.3F),
    glm::vec3(1.0F, 0.55F, 0.0F),
    glm::vec3(0.9F, 0.1F, 0.1F)};
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

    glEnable(GL_PROGRAM_POINT_SIZE);

    if (!m_shader.load(kVertexShaderPath, kFragmentShaderPath))
    {
        return false;
    }

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
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
    ground.reserve(points.size());
    nonGround.reserve(points.size());

    const bool useZoneColors = m_cameraMode == CameraMode::FreeOrbit;
    for (const auto& point : points)
    {
        const bool groundPoint = isGroundPoint(point);
        float classification = groundPoint ? 0.0F : 1.0F;
        if (useZoneColors)
        {
            classification = static_cast<float>(zoneIndexFromHeight(point.z));
        }
        Vertex vertex{
            point.x,
            point.y,
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
        }
    }

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
}

void Visualizer::drawWorldControls()
{
    ImGui::Begin("LiDAR Controls");
    if (ImGui::TreeNodeEx("General", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Enable visualization", &m_worldFrameSettings.enableWorldVisualization);
        ImGui::Separator();

        ImGui::SliderFloat("Point size", &m_worldFrameSettings.pointSize, 1.0F, 6.0F);

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

bool Visualizer::isGroundPoint(const lidar::LidarPoint& point) const noexcept
{
    return point.z <= m_worldFrameSettings.groundClassificationHeight;
}

int Visualizer::zoneIndexFromHeight(float height) const noexcept
{
    if (height < -1.5F)
    {
        return 0;
    }
    if (height < 0.0F)
    {
        return 1;
    }
    if (std::fabs(height) < 1e-3F)
    {
        return 2;
    }
    if (height < 1.5F)
    {
        return 3;
    }
    return 4;
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
