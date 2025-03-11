#ifndef MESH_H
#define MESH_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <vector>
#include <string>

class Mesh
{
    public:
        Mesh() : VAO(0), VBO(0), EBO(0), vertexDataSize(0), indexCount(0) {}
        Mesh(const std::vector<glm::vec3>& positions, const std::vector<glm::vec3>& normals,
            const std::vector<GLuint>& indices);
        ~Mesh();

        // 初始化 OpenGL 资源
        void initialize();
        // 获取 OpenGL 资源和数据
        GLuint getVAO() const { return VAO; }
        int getIndexCount() const { return indexCount; }
        const std::vector<glm::vec3>& getPositions() const { return positions; }
        const std::vector<glm::vec3>& getNormals() const { return normals; }
        const std::vector<GLuint>& getIndices() const { return indices; }

        // 加载 OBJ 文件
        bool loadFromOBJ(const std::string& filename);

    protected:
        std::vector<glm::vec3> positions; // 顶点位置
        std::vector<glm::vec3> normals;   // 法线
        std::vector<GLuint> indices;      // 索引

        GLuint VAO, VBO, EBO;             // OpenGL 资源
        std::vector<GLfloat> vertexData;  // 顶点数据（位置 + 法线）
        int vertexDataSize;               // vertexData 大小
        int indexCount;                   // 索引数量
};

#endif