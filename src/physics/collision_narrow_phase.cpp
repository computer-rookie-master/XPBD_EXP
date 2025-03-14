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

    // 计算点到网格表面的最小距离（局部坐标转换为世界坐标）
    float minDistance = std::numeric_limits<float>::max();
    for (const auto& vertex : vertices) {
        glm::vec3 worldVertex = entityPos + vertex; // 局部坐标转为世界坐标
        float distance = glm::length(point - worldVertex);
        minDistance = std::min(minDistance, distance);
    }

    // 简单近似：假设表面距离为正，内部为负
    // 这里仅基于顶点的最小距离，未精确区分内外（可后续优化）
    return minDistance; // 当前为无符号距离，后续可添加签名判断
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

    // 粗略估计碰撞：如果中心间距离小于两 SDF 的和，认为可能碰撞
    float threshold = sdfAatB + sdfBatA;
    if (distance < threshold) {
        // 穿透深度近似为 threshold 减去实际距离
        penetration = threshold - distance;

        // 法线方向：从 A 到 B（或反向取决于具体情况）
        normal = (distance > 0.0001f) ? direction / distance : glm::vec3(1.0f, 0.0f, 0.0f);

        // 如果 sdfAatB 较小，说明 B 更靠近 A，调整法线方向
        if (sdfAatB < sdfBatA) {
            normal = -normal; // 指向从 B 到 A
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