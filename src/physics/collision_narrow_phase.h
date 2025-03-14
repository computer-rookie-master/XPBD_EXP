#ifndef COLLISION_NARROW_PHASE_H
#define COLLISION_NARROW_PHASE_H

#include <glm/glm.hpp>
#include <vector>
#include "physics/entity.h"

class CollisionNarrowPhase
{
    public:

        CollisionNarrowPhase(); // 添加构造函数
        ~CollisionNarrowPhase(); // 可选：析构函数
        
        // 检测两个实体是否碰撞，并返回穿透深度和法线
        bool detectCollision(const Entity* entityA, const glm::vec3& posA,
            const Entity* entityB, const glm::vec3& posB,
            float& penetration, glm::vec3& normal);

    private:
        // 计算实体在给定位置的 SDF 值
        float computeSDF(const Entity* entity, const glm::vec3& point);

        // 根据两个实体的 SDF 计算碰撞信息
        bool resolveSDFCollision(const Entity* entityA, const glm::vec3& posA,
                                const Entity* entityB, const glm::vec3& posB,
                                float& penetration, glm::vec3& normal);
};

#endif