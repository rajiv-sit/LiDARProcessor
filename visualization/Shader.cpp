#include "visualization/Shader.hpp"

#include <fstream>
#include <iostream>
#include <iterator>

namespace visualization
{

Shader::~Shader()
{
    if (m_program)
    {
        glDeleteProgram(m_program);
    }
}

bool Shader::load(const std::string& vertexPath, const std::string& fragmentPath)
{
    const auto vertexSource = loadSource(vertexPath);
    const auto fragmentSource = loadSource(fragmentPath);

    if (vertexSource.empty() || fragmentSource.empty())
    {
        return false;
    }

    const GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    if (!compileShader(vertexShader, vertexSource))
    {
        glDeleteShader(vertexShader);
        return false;
    }

    const GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    if (!compileShader(fragmentShader, fragmentSource))
    {
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return false;
    }

    m_program = glCreateProgram();
    glAttachShader(m_program, vertexShader);
    glAttachShader(m_program, fragmentShader);
    glLinkProgram(m_program);

    GLint linkStatus = GL_FALSE;
    glGetProgramiv(m_program, GL_LINK_STATUS, &linkStatus);
    if (linkStatus != GL_TRUE)
    {
        GLint length = 0;
        glGetProgramiv(m_program, GL_INFO_LOG_LENGTH, &length);
        std::string log(length, '\0');
        glGetProgramInfoLog(m_program, length, nullptr, log.data());
        std::cerr << "Shader link error: " << log << '\n';
        glDeleteProgram(m_program);
        m_program = 0;
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return false;
    }

    glDetachShader(m_program, vertexShader);
    glDetachShader(m_program, fragmentShader);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return true;
}

void Shader::use() const
{
    if (m_program)
    {
        glUseProgram(m_program);
    }
}

GLuint Shader::id() const noexcept
{
    return m_program;
}

GLint Shader::uniformLocation(const std::string& name) const
{
    if (!m_program)
    {
        return -1;
    }
    return glGetUniformLocation(m_program, name.c_str());
}

std::string Shader::loadSource(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "Unable to open shader file: " << path << '\n';
        return {};
    }

    std::string source;
    file.seekg(0, std::ios::end);
    source.reserve(static_cast<size_t>(file.tellg()));
    file.seekg(0, std::ios::beg);
    source.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return source;
}

bool Shader::compileShader(GLuint shader, const std::string& source)
{
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint status = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE)
    {
        GLint length = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        std::string log(length, '\0');
        glGetShaderInfoLog(shader, length, nullptr, log.data());
        std::cerr << "Shader compile error: " << log << '\n';
        return false;
    }

    return true;
}

} // namespace visualization
