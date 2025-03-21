#include "physics/collision_narrow_phase.h"
#include <cmath>
#include <iostream>
#include <limits>

CollisionNarrowPhase::CollisionNarrowPhase() {}

CollisionNarrowPhase::~CollisionNarrowPhase() {}

float CollisionNarrowPhase::computeSDF(const Entity* entity, const glm::vec3& point) 
{
    // 获取实体的网格和位置
    const Mesh* mesh = entity->getMesh();
    glm::vec3 entityPos = entity->getPosition();
    const std::vector<glm::vec3>& vertices = mesh->getPositions();
    const std::vector<unsigned int>& indices = mesh->getIndices();

    if (vertices.empty() || indices.empty())
    {
        // 默认球形 SDF（以中心为原点，半径 0.1）
        float distance = glm::length(point - entityPos) - 0.1f;
        return distance;
    }

    // 计算点到网格表面的最小距离（局部坐标转换为世界坐标）
    float minDistance = std::numeric_limits<float>::max();
    glm::vec3 closestPoint;

    // 遍历所有三角形（假设 indices 每 3 个为一组）
    for (size_t i = 0; i < indices.size(); i += 3) {
        // 获取三角形顶点（世界坐标）
        glm::vec3 v0 = entityPos + vertices[indices[i]];
        glm::vec3 v1 = entityPos + vertices[indices[i + 1]];
        glm::vec3 v2 = entityPos + vertices[indices[i + 2]];

        // 计算点到三角形的最近点
        glm::vec3 edge0 = v1 - v0;
        glm::vec3 edge1 = v2 - v0;
        glm::vec3 v0p = point - v0;

        float a = glm::dot(edge0, edge0);
        float b = glm::dot(edge0, edge1);
        float c = glm::dot(edge1, edge1);
        float d = glm::dot(edge0, v0p);
        float e = glm::dot(edge1, v0p);
        float det = a * c - b * b;

        float s = (c * d - b * e) / det;
        float t = (a * e - b * d) / det;

        // 限制 s 和 t 到三角形内
        s = glm::clamp(s, 0.0f, 1.0f);
        t = glm::clamp(t, 0.0f, 1.0f);
        if (s + t > 1.0f) {
            s = glm::clamp(s, 0.0f, 1.0f - t);
            t = glm::clamp(t, 0.0f, 1.0f - s);
        }

        // 计算最近点
        glm::vec3 closest = v0 + s * edge0 + t * edge1;
        float distance = glm::length(point - closest);
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = closest;
        }
    }

    // 判断内外（使用最近点的法线方向）
    glm::vec3 direction = point - closestPoint;
    // 假设网格有法线数据（这里简化为从中心指向外部）
    glm::vec3 avgNormal(0.0f);
    for (size_t i = 0; i < indices.size(); i += 3) {
        glm::vec3 v0 = vertices[indices[i]];
        glm::vec3 v1 = vertices[indices[i + 1]];
        glm::vec3 v2 = vertices[indices[i + 2]];
        avgNormal += glm::normalize(glm::cross(v1 - v0, v2 - v0));
    }
    avgNormal = glm::normalize(avgNormal);

    // 若 direction 与法线反向，则点在内部
    float sign = (glm::dot(direction, avgNormal) >= 0.0f) ? 1.0f : -1.0f;
    return sign * minDistance;
}

bool CollisionNarrowPhase::resolveSDFCollision(const Entity* entityA, const glm::vec3& posA,
                                               const Entity* entityB, const glm::vec3& posB,
                                               float& penetration, glm::vec3& normal) 
{
    // 计算两实体中心的 SDF
    float sdfAatB = computeSDF(entityA, posB); // B 中心相对于 A 的 SDF
    float sdfBatA = computeSDF(entityB, posA); // A 中心相对于 B 的 SDF

    // 计算两中心间的距离
    glm::vec3 direction = posB - posA;
    float distance = glm::length(direction);

    // 检查是否碰撞
    if (sdfAatB < 0.0f || sdfBatA < 0.0f) {
        // 至少一个中心在另一个物体内部，确认碰撞
        penetration = std::max(std::abs(sdfAatB), std::abs(sdfBatA));

        // 法线方向：从深入较深的物体指向另一个
        normal = (distance > 0.0001f) ? direction / distance : glm::vec3(0.0f, 1.0f, 0.0f);
        if (sdfAatB < sdfBatA) {
            normal = -normal; // B 更深入 A，法线从 B 指向 A
        }

        return true;
    }

    // 若中心间距离小于 SDF 和（正值情况），仍可能碰撞
    float threshold = sdfAatB + sdfBatA;
    if (distance < threshold && threshold > 0.0f) {
        penetration = threshold - distance;
        normal = (distance > 0.0001f) ? direction / distance : glm::vec3(0.0f, 1.0f, 0.0f);
        if (sdfAatB < sdfBatA) {
            normal = -normal;
        }
        return true;
    }

    return false;
}

bool CollisionNarrowPhase::detectCollision(const Entity* entityA, const glm::vec3& posA,
                                           const Entity* entityB, const glm::vec3& posB,
                                           float& penetration, glm::vec3& normal) 
{
    // 调用 SDF 碰撞解析
    bool collided = resolveSDFCollision(entityA, posA, entityB, posB, penetration, normal);

    if (collided) {
        std::cout << "Collision detected: Penetration = " << penetration
                  << ", Normal = (" << normal.x << ", " << normal.y << ", " << normal.z << ")\n";
    }
    return collided;
}