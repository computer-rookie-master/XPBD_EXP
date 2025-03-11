#ifndef COLLIDER_H
#define COLLIDER_H

#include <glm/glm.hpp>
#include <vector>

enum ColliderType {
    COLLIDER_TYPE_SPHERE,
    COLLIDER_TYPE_CONVEX_HULL
};

struct ColliderSphere {
    float radius;
    glm::vec3 center;
};

struct ColliderConvexHull {
    std::vector<glm::vec3> vertices;  // 顶点
    std::vector<std::vector<unsigned int>> faces;  // 面（每个面由顶点索引组成）
};

class Collider
{
    public:
        ColliderType type;
        union {
            ColliderSphere sphere;
            ColliderConvexHull convexHull;
        };

        // 构造函数
        Collider();
        explicit Collider(float radius);  // 球体，添加 explicit 防止隐式转换
        Collider(const std::vector<glm::vec3>& vertices, const std::vector<std::vector<unsigned int>>& faces);  // 凸包

        // 析构函数
        ~Collider();

        // 禁止拷贝和赋值（避免资源管理问题）
        Collider(const Collider&) = delete;
        Collider& operator=(const Collider&) = delete;

        // GJK 所需的支撑函数
        glm::vec3 getSupportPoint(const glm::vec3& direction) const;
};

#endif