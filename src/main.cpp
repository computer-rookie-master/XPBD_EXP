#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "physics/xpbd.h"
#include "render/mesh_renderer.h"
#include "render/sphere_mesh.h"
#include "render/cube_mesh.h"
#include "physics/entity.h"
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

int main()
{
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "XPBD Simulation", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "Renderer: " << glGetString(GL_RENDERER) << std::endl;

    XPBDSystem xpbdSystem;

    // 创建 MeshRenderer
    MeshRenderer renderer;
    renderer.initialize();

    // 创建 Mesh 对象
    SphereMesh* sphereMesh1 = new SphereMesh(0.1f, 50, 50);
    std::cout << "SphereMesh1 created with " << sphereMesh1->getPositions().size() << " vertices, "
              << sphereMesh1->getIndices().size() << " indices" << std::endl;
    sphereMesh1->initialize();

    SphereMesh* sphereMesh2 = new SphereMesh(0.1f, 50, 50);
    std::cout << "SphereMesh2 created with " << sphereMesh2->getPositions().size() << " vertices, "
              << sphereMesh2->getIndices().size() << " indices" << std::endl;
    sphereMesh2->initialize();

    // 创建 Entity 对象（移除 Collider 参数）
    Entity* sphereEntity1 = new Entity(sphereMesh1, glm::vec3(0.0f, 0.0f, 0.0f), 1.0f);
    Entity* sphereEntity2 = new Entity(sphereMesh2, glm::vec3(0.1f, 0.5f, 0.0f), 1.0f);

    // 添加到 XPBDSystem
    xpbdSystem.addObject(sphereEntity1);
    xpbdSystem.addObject(sphereEntity2);

    xpbdSystem.initialize();

    glEnable(GL_DEPTH_TEST);

    glm::vec3 lightPos(1.2f, 1.0f, 2.0f);
    glm::vec3 viewPos(0.0f, 0.0f, 2.0f);

    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 2.0f),
                                     glm::vec3(0.0f, 0.0f, 0.0f),
                                     glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);

        const auto& objects = xpbdSystem.getObjects();
        for (size_t i = 0; i < objects.size(); ++i)
        {
            std::cout << "Rendering object " << i << " at " << objects[i]->getPosition().x << std::endl;
            glBindVertexArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
            glUseProgram(0);

            glm::mat4 model = glm::translate(glm::mat4(1.0f), objects[i]->getPosition());
            glm::mat4 mvp = projection * view * model;
            renderer.render(objects[i]->getMesh(), mvp, lightPos, viewPos, objects[i]->getPosition());
        }

        xpbdSystem.run();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 清理资源（移除 Collider 的删除）
    delete sphereEntity1;
    delete sphereEntity2;
    delete sphereMesh1;
    delete sphereMesh2;

    glfwTerminate();
    return 0;
}