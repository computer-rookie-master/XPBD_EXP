#ifndef MESH_RENDERER_H
#define MESH_RENDERER_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include "mesh.h"

class MeshRenderer 
{
public:
    MeshRenderer();
    ~MeshRenderer();
    
    void render(Mesh* mesh, const glm::mat4& mvp, const glm::vec3& lightPos, 
                const glm::vec3& viewPos, const glm::vec3& objectPos, const glm::quat& rotation);
    void initialize();

private:
    GLuint shaderProgram;

    GLuint compileShader(GLenum type, const char* source);
    GLuint createShaderProgram();
};

#endif