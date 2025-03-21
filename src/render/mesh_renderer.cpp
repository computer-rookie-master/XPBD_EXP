#include "render/mesh_renderer.h"
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aNormal;
    uniform mat4 mvp;
    uniform mat4 model;
    out vec3 Normal;
    out vec3 FragPos;
    void main()
    {
        gl_Position = mvp * vec4(aPos, 1.0);
        FragPos = vec3(model * vec4(aPos, 1.0));
        Normal = mat3(transpose(inverse(model))) * aNormal;
    }
)";

const char* fragmentShaderSource = R"(
    #version 330 core
    in vec3 Normal;
    in vec3 FragPos;
    out vec4 FragColor;
    uniform vec3 lightPos;
    uniform vec3 viewPos;
    uniform vec3 objectColor;
    uniform vec3 lightColor;
    void main()
    {
        float ambientStrength = 0.1;
        vec3 ambient = ambientStrength * lightColor;

        vec3 norm = normalize(Normal);
        vec3 lightDir = normalize(lightPos - FragPos);
        float diff = max(dot(norm, lightDir), 0.0);
        vec3 diffuse = diff * lightColor;

        float specularStrength = 0.5;
        vec3 viewDir = normalize(viewPos - FragPos);
        vec3 reflectDir = reflect(-lightDir, norm);
        float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
        vec3 specular = specularStrength * spec * lightColor;

        vec3 result = (ambient + diffuse + specular) * objectColor;
        FragColor = vec4(result, 1.0);
    }
)";

MeshRenderer::MeshRenderer() : shaderProgram(0)
{
}

MeshRenderer::~MeshRenderer()
{
    if (shaderProgram != 0) 
    {
        glDeleteProgram(shaderProgram);
    }
}

void MeshRenderer::initialize()
{
    shaderProgram = createShaderProgram();
}

void MeshRenderer::render(Mesh* mesh, const glm::mat4& mvp, const glm::vec3& lightPos, 
                          const glm::vec3& viewPos, const glm::vec3& objectPos, const glm::quat& rotation) 
{
    if (mesh == nullptr) 
    {
        std::cerr << "Error: MeshRenderer::render received null Mesh pointer" << std::endl;
        return;
    }

    glUseProgram(shaderProgram);

    glm::mat4 rotationMatrix = glm::mat4_cast(rotation);
    glm::mat4 model = glm::translate(glm::mat4(1.0f), objectPos) * rotationMatrix;

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "mvp"), 1, GL_FALSE, glm::value_ptr(mvp));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));

    glUniform3fv(glGetUniformLocation(shaderProgram, "lightPos"), 1, glm::value_ptr(lightPos));
    glUniform3fv(glGetUniformLocation(shaderProgram, "viewPos"), 1, glm::value_ptr(viewPos));
    glm::vec3 objectColor(1.0f, 0.5f, 0.2f);
    glm::vec3 lightColor(1.0f, 1.0f, 1.0f);
    glUniform3fv(glGetUniformLocation(shaderProgram, "objectColor"), 1, glm::value_ptr(objectColor));
    glUniform3fv(glGetUniformLocation(shaderProgram, "lightColor"), 1, glm::value_ptr(lightColor));

    glBindVertexArray(mesh->getVAO());
    glDrawElements(GL_TRIANGLES, mesh->getIndexCount(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

GLuint MeshRenderer::compileShader(GLenum type, const char* source) 
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) 
    {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    return shader;
}

GLuint MeshRenderer::createShaderProgram() 
{
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);

    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) 
    {
        char infoLog[512];
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        std::cerr << "Shader program linking failed: " << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return program;
}