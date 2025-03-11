#ifndef COLLISION_NARROW_PHASE_H
#define COLLISION_NARROW_PHASE_H

#include <glm/glm.hpp>
#include <vector>
#include "physics/collider.h"

class CollisionNarrowPhase
{
    public:

        CollisionNarrowPhase(); // 添加构造函数
        ~CollisionNarrowPhase(); // 可选：析构函数
        
        // GJK 碰撞检测，返回是否碰撞及最终单纯形
        bool gjkCollision(const Collider* colliderA, const glm::vec3& posA,
                const Collider* colliderB, const glm::vec3& posB,
                std::vector<glm::vec3>& simplex);

        // EPA 计算穿透深度和法线
        bool epaPenetration(const Collider* colliderA, const glm::vec3& posA,
                    const Collider* colliderB, const glm::vec3& posB,
                    const std::vector<glm::vec3>& initialSimplex,
                    float& penetration, glm::vec3& normal);

    private:
        // 多面体面结构
        struct PolytopeFace {
            int indices[3]; // 顶点索引
            glm::vec3 normal; // 法线
            float distance; // 到原点的距离

            PolytopeFace(int i0, int i1, int i2, const std::vector<glm::vec3>& vertices)
            {
                indices[0] = i0;
                indices[1] = i1;
                indices[2] = i2;
                glm::vec3 v0 = vertices[i0];
                glm::vec3 v1 = vertices[i1];
                glm::vec3 v2 = vertices[i2];
                normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
                distance = glm::dot(normal, v0);
                if (distance < 0) {
                    normal = -normal;
                    distance = -distance;
                    std::swap(indices[0], indices[1]);
                }
            }
        };

        // 辅助函数改为非静态
        glm::vec3 support(const Collider* collider, const glm::vec3& pos, const glm::vec3& direction);
        bool doSimplex(std::vector<glm::vec3>& simplex, glm::vec3& direction);
        bool sameDirection(const glm::vec3& direction, const glm::vec3& ao);
};

#endif