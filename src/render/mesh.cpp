#include "render/mesh.h"
#include <fstream>
#include <sstream>
#include <iostream>

Mesh::Mesh(const std::vector<glm::vec3>& positions, const std::vector<glm::vec3>& normals,
    const std::vector<GLuint>& indices)
: positions(positions), normals(normals), indices(indices) 
{
    // 构建 vertexData
    for (size_t i = 0; i < positions.size(); ++i) {
        vertexData.push_back(positions[i].x);
        vertexData.push_back(positions[i].y);
        vertexData.push_back(positions[i].z);
        vertexData.push_back(normals[i].x);
        vertexData.push_back(normals[i].y);
        vertexData.push_back(normals[i].z);
    }
    vertexDataSize = vertexData.size();
    indexCount = indices.size();
}

Mesh::~Mesh()
{
    if (VAO != 0) glDeleteVertexArrays(1, &VAO);
    if (VBO != 0) glDeleteBuffers(1, &VBO);
    if (EBO != 0) glDeleteBuffers(1, &EBO);
}

void Mesh::initialize()
{
    // 清理现有资源
    if (VAO != 0) {
        glDeleteVertexArrays(1, &VAO);
        VAO = 0;
    }
    if (VBO != 0) {
        glDeleteBuffers(1, &VBO);
        VBO = 0;
    }
    if (EBO != 0) {
        glDeleteBuffers(1, &EBO);
        EBO = 0;
    }

    // 生成新的资源
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    if (VAO == 0 || VBO == 0 || EBO == 0) {
        std::cerr << "Failed to generate OpenGL resources: VAO=" << VAO 
                  << ", VBO=" << VBO << ", EBO=" << EBO << std::endl;
        return;
    }

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(GLfloat),
                 vertexData.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
                 indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    std::cout << "Initialized Mesh with VAO: " << VAO 
              << ", VBO: " << VBO << ", EBO: " << EBO << std::endl;
}

bool Mesh::loadFromOBJ(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    std::vector<glm::vec3> tempPositions;
    std::vector<glm::vec3> tempNormals;
    std::string line;

    positions.clear();
    normals.clear();
    indices.clear();
    vertexData.clear();

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v") // 顶点
        {
            glm::vec3 pos;
            iss >> pos.x >> pos.y >> pos.z;
            tempPositions.push_back(pos);
        }
        else if (type == "vn") // 法线
        {
            glm::vec3 normal;
            iss >> normal.x >> normal.y >> normal.z;
            tempNormals.push_back(normal);
        }
        else if (type == "f") // 面
        {
            std::string v1, v2, v3;
            iss >> v1 >> v2 >> v3;

            // 解析每个顶点的索引（格式：vertex//normal）
            GLuint idx1, idx2, idx3;
            GLuint nidx1, nidx2, nidx3;

            sscanf(v1.c_str(), "%u//%u", &idx1, &nidx1);
            sscanf(v2.c_str(), "%u//%u", &idx2, &nidx2);
            sscanf(v3.c_str(), "%u//%u", &idx3, &nidx3);

            idx1--; idx2--; idx3--;
            nidx1--; nidx2--; nidx3--;

            positions.push_back(tempPositions[idx1]);
            positions.push_back(tempPositions[idx2]);
            positions.push_back(tempPositions[idx3]);
            normals.push_back(tempNormals[nidx1]);
            normals.push_back(tempNormals[nidx2]);
            normals.push_back(tempNormals[nidx3]);

            indices.push_back(positions.size() - 3);
            indices.push_back(positions.size() - 2);
            indices.push_back(positions.size() - 1);

            vertexData.push_back(tempPositions[idx1].x);
            vertexData.push_back(tempPositions[idx1].y);
            vertexData.push_back(tempPositions[idx1].z);
            vertexData.push_back(tempNormals[nidx1].x);
            vertexData.push_back(tempNormals[nidx1].y);
            vertexData.push_back(tempNormals[nidx1].z);

            vertexData.push_back(tempPositions[idx2].x);
            vertexData.push_back(tempPositions[idx2].y);
            vertexData.push_back(tempPositions[idx2].z);
            vertexData.push_back(tempNormals[nidx2].x);
            vertexData.push_back(tempNormals[nidx2].y);
            vertexData.push_back(tempNormals[nidx2].z);

            vertexData.push_back(tempPositions[idx3].x);
            vertexData.push_back(tempPositions[idx3].y);
            vertexData.push_back(tempPositions[idx3].z);
            vertexData.push_back(tempNormals[nidx3].x);
            vertexData.push_back(tempNormals[nidx3].y);
            vertexData.push_back(tempNormals[nidx3].z);
        }
    }

    vertexDataSize = vertexData.size();
    indexCount = indices.size();

    file.close();
    std::cout << "Loaded OBJ: " << filename << " with " << positions.size() << " vertices and "
              << indices.size() / 3 << " triangles." << std::endl;
    return true;
}