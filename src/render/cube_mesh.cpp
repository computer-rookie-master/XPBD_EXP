#include "render/cube_mesh.h"

CubeMesh::CubeMesh(float length, float width, float height) : Mesh()
{
    float halfLength = length / 2.0f;
    float halfWidth = width / 2.0f;
    float halfHeight = height / 2.0f;

    positions.clear();
    normals.clear();
    indices.clear();

    std::vector<glm::vec3> tempPositions = 
    {
        {-halfLength, -halfHeight, -halfWidth}, // 0: 前左下
        { halfLength, -halfHeight, -halfWidth}, // 1: 前右下
        { halfLength,  halfHeight, -halfWidth}, // 2: 前右上
        {-halfLength,  halfHeight, -halfWidth}, // 3: 前左上
        {-halfLength, -halfHeight,  halfWidth}, // 4: 后左下
        { halfLength, -halfHeight,  halfWidth}, // 5: 后右下
        { halfLength,  halfHeight,  halfWidth}, // 6: 后右上
        {-halfLength,  halfHeight,  halfWidth}  // 7: 后左上
    };

    std::vector<glm::vec3> tempNormals = 
    {
        { 0.0f,  0.0f, -1.0f}, // 前
        { 0.0f,  0.0f,  1.0f}, // 后
        {-1.0f,  0.0f,  0.0f}, // 左
        { 1.0f,  0.0f,  0.0f}, // 右
        { 0.0f,  1.0f,  0.0f}, // 上
        { 0.0f, -1.0f,  0.0f}  // 下
    };

    auto addFace = [&](int v0, int v1, int v2, int v3, const glm::vec3& normal) 
    {
        int baseIdx = positions.size();
        positions.push_back(tempPositions[v0]);
        positions.push_back(tempPositions[v1]);
        positions.push_back(tempPositions[v2]);
        positions.push_back(tempPositions[v3]);

        for (int i = 0; i < 4; ++i) 
        {
            normals.push_back(normal);
        }

        indices.push_back(baseIdx);
        indices.push_back(baseIdx + 1);
        indices.push_back(baseIdx + 2);

        indices.push_back(baseIdx);
        indices.push_back(baseIdx + 2);
        indices.push_back(baseIdx + 3);
    };

    addFace(0, 1, 2, 3, tempNormals[0]); // 前
    addFace(5, 4, 7, 6, tempNormals[1]); // 后
    addFace(4, 0, 3, 7, tempNormals[2]); // 左
    addFace(1, 5, 6, 2, tempNormals[3]); // 右
    addFace(3, 2, 6, 7, tempNormals[4]); // 上
    addFace(4, 5, 1, 0, tempNormals[5]); // 下

    // 构建 vertexData
    vertexData.clear();
    for (size_t i = 0; i < positions.size(); ++i) 
    {
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