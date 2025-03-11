#include "render/sphere_mesh.h"
#include <iostream>

SphereMesh::SphereMesh(float radius, int sectors, int stacks) : Mesh()
{
    float sectorStep = 2 * 3.14159f / sectors;
    float stackStep = 3.14159f / stacks;

    positions.clear();
    normals.clear();
    indices.clear();

    // 生成顶点和法线
    for (int i = 0; i <= stacks; ++i) 
    {
        float stackAngle = 3.14159f / 2 - i * stackStep;
        float xy = radius * cosf(stackAngle);
        float z = radius * sinf(stackAngle);

        for (int j = 0; j <= sectors; ++j) 
        {
            float sectorAngle = j * sectorStep;
            float x = xy * cosf(sectorAngle);
            float y = xy * sinf(sectorAngle);

            positions.push_back(glm::vec3(x, y, z));
            normals.push_back(glm::normalize(glm::vec3(x, y, z)));
        }
    }

    // 生成索引
    for (int i = 0; i < stacks; ++i) 
    {
        for (int j = 0; j < sectors; ++j) 
        {
            int k1 = i * (sectors + 1) + j;
            int k2 = k1 + 1;
            int k3 = (i + 1) * (sectors + 1) + j;
            int k4 = k3 + 1;

            indices.push_back(k1);
            indices.push_back(k3);
            indices.push_back(k4);

            indices.push_back(k1);
            indices.push_back(k4);
            indices.push_back(k2);
        }
    }

    // 构建 vertexData
    vertexData.clear();
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