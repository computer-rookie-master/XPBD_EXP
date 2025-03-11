#include "render/sphere_renderer.h"
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

// 着色器
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
        // 环境光
        float ambientStrength = 0.1;
        vec3 ambient = ambientStrength * lightColor;

        // 漫反射
        vec3 norm = normalize(Normal);
        vec3 lightDir = normalize(lightPos - FragPos);
        float diff = max(dot(norm, lightDir), 0.0);
        vec3 diffuse = diff * lightColor;

        // 镜面反射
        float specularStrength = 0.5;
        vec3 viewDir = normalize(viewPos - FragPos);
        vec3 reflectDir = reflect(-lightDir, norm);
        float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
        vec3 specular = specularStrength * spec * lightColor;

        vec3 result = (ambient + diffuse + specular) * objectColor;
        FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f); //橙色
    }
)";

SphereRenderer::SphereRenderer(float radius, int sectors, int stacks)
{
    createSphere(radius, sectors, stacks);
    vertexCount = vertices.size() / 6;
    indexCount = indices.size();
}

SphereRenderer::~SphereRenderer()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteProgram(shaderProgram);
}

void SphereRenderer::initialize()
{
    // 创建并编辑着色器
    shaderProgram = createShaderProgram();

    // 设置VAO和VBO
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat),
                vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
                indices.data(), GL_STATIC_DRAW);

    // 位置属性
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // 法显属性
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void SphereRenderer::render(const glm::mat4& mvp, const glm::vec3& lightPos, const glm::vec3& viewPos)
{
    glUseProgram(shaderProgram);
    // 传递mvp矩阵
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "mvp"), 1, GL_FALSE, glm::value_ptr(mvp));
    
    // 传递模型矩阵（用于法线变换）
    glm::mat4 model = glm::mat4(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    
    // 传递光照参数
    glUniform3fv(glGetUniformLocation(shaderProgram, "lightPos"), 1, glm::value_ptr(lightPos));
    glUniform3fv(glGetUniformLocation(shaderProgram, "viewPos"), 1, glm::value_ptr(viewPos));
    glm::vec3 objectColor(1.0f, 0.5f, 0.2f); // 橙色
    glm::vec3 lightColor(1.0f, 1.0f, 1.0f); // 白色
    glUniform3fv(glGetUniformLocation(shaderProgram, "objectColor"), 1, glm::value_ptr(objectColor));
    glUniform3fv(glGetUniformLocation(shaderProgram, "lightColor"), 1, glm::value_ptr(lightColor));

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
}

void SphereRenderer::createSphere(float radius, int sectors, int stacks)
{
    float x, y, z, xy;
    float nx, ny, nz;
    float sectorStep = 2 * 3.14159f / sectors;
    float stackStep = 3.14159f / stacks;

    // 生成顶点和法线
    for (int i = 0; i <= stacks; ++i)
    {
        float stackAngle = 3.14159f / 2 - i * stackStep;
        xy = radius * cosf(stackAngle);
        z = radius * sinf(stackAngle);

        for (int j = 0; j <= stacks; ++j)
        {
            float sectorAngle = j * sectorStep;
            x = xy * cosf(sectorAngle);
            y = xy * sinf(sectorAngle);
            // 顶点位置
            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
            // 法线
            nx = x / radius;
            ny = y / radius;
            nz = z / radius;
            vertices.push_back(nx);
            vertices.push_back(ny);
            vertices.push_back(nz);
        }
    }

    // 生成索引
    for (int i = 0; i < stacks; ++i)
    {
        for (int j = 0; j < sectors; ++j)
        {
            // 计算四个顶点的索引
            int k1 = i * (sectors + 1) + j;
            int k2 = k1 + 1;
            int k3 = (i + 1) * (sectors + 1) + j;
            int k4 = k3 + 1;

            // 第一个三角形
            indices.push_back(k1);
            indices.push_back(k3);
            indices.push_back(k4);

            // 第二个三角形
            indices.push_back(k1);
            indices.push_back(k4);
            indices.push_back(k2);
        }
    }
}

GLuint SphereRenderer::compileShader(GLenum type, const char* source)
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
        std::cout << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    return shader;
}

GLuint SphereRenderer::createShaderProgram()
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
        std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return program;
}