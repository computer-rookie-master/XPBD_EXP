#include "physics/collider.h"
#include <glm/glm.hpp>
#include <vector>
#include <algorithm>

Collider::Collider() : type(COLLIDER_TYPE_SPHERE)
{
    sphere.radius = 0.0f;
    sphere.center = glm::vec3(0.0f);
}

Collider::Collider(float radius) : type(COLLIDER_TYPE_SPHERE)
{
    sphere.radius = radius;
    sphere.center = glm::vec3(0.0f);
}

Collider::Collider(const std::vector<glm::vec3>& vertices, const std::vector<std::vector<unsigned int>>& faces) : type(COLLIDER_TYPE_CONVEX_HULL)
{
    convexHull.vertices = vertices;
    convexHull.faces = faces;
}

Collider::~Collider()
{
    // 目前无需释放资源，因为 std::vector 自动管理内存
}

glm::vec3 Collider::getSupportPoint(const glm::vec3& direction) const
{
    if (type == COLLIDER_TYPE_SPHERE)
    {
        // 球体的支撑点是球心加上沿 direction 方向缩放的半径
        glm::vec3 normalizedDir = glm::normalize(direction);
        return sphere.center + normalizedDir * sphere.radius;
    }
    else if (type == COLLIDER_TYPE_CONVEX_HULL)
    {
        // 凸包的支撑点是顶点中点乘 direction 最大的顶点
        glm::vec3 maxPoint = convexHull.vertices[0];
        float maxDot = glm::dot(convexHull.vertices[0], direction);

        for (const auto& vertex : convexHull.vertices)
        {
            float dot = glm::dot(vertex, direction);
            if (dot > maxDot)
            {
                maxDot = dot;
                maxPoint = vertex;
            }
        }
        return maxPoint;
    }
    return glm::vec3(0.0f); // 默认值，防止未定义行为
}