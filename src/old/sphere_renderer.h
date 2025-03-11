#ifndef SPHERE_RENDERER_H
#define SPHERE_RENDERER_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <vector>

class SphereRenderer {
public:
    SphereRenderer(float radius = 0.1f, int sectors = 20, int stacks = 20);
    ~SphereRenderer();
    
    void render(const glm::mat4& mvp, const glm::vec3& lightPos, const glm::vec3& viewPos);
    void initialize();

private:
    GLuint VAO, VBO, EBO;
    GLuint shaderProgram;
    std::vector<GLfloat> vertices;      // x, y, z, nx, ny, nz
    std::vector<GLuint> indices;
    int vertexCount;
    int indexCount;

    void createSphere(float radius, int sectors, int stacks);
    GLuint compileShader(GLenum type, const char* source);
    GLuint createShaderProgram();
};

#endif