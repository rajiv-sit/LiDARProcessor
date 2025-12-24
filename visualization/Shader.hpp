#pragma once

#include <GL/glew.h>
#include <string>

namespace visualization
{

class Shader
{
public:
    Shader() = default;
    ~Shader();

    bool load(const std::string& vertexPath, const std::string& fragmentPath);
    void use() const;
    GLuint id() const noexcept;
    GLint uniformLocation(const std::string& name) const;

private:
    GLuint m_program = 0;

    static std::string loadSource(const std::string& path);
    bool compileShader(GLuint shader, const std::string& source);
};

} // namespace visualization
