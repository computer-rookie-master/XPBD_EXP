#include "physics/xpbd.h"
#include "physics/collision_narrow_phase.h"
#include <iostream>
#include <cmath>

XPBDSystem::XPBDSystem() : gravity(-9.81f), timeStep(1.0f / 60.0f)
{
    std::cout << "XPBD System Initialized" << std::endl;
}

void XPBDSystem::addObject(Entity* entity)
{
    if (entity == nullptr)
    {
        std::cerr << "Error: Attempt to add null Entity to XPBDSystem" << std::endl;
        return;
    }
    if (entity->getMesh() == nullptr)
    {
        std::cerr << "Error: Entity has null Mesh" << std::endl;
        return;
    }
    if (entity->getMesh()->getVAO() == 0)
    {
        entity->getMesh()->initialize();
    }
    objects.push_back(entity);
    broadPhase.addObject(entity);
}

void XPBDSystem::run()
{
    for (auto& obj : objects)
    {
        // 应用重力
        glm::vec3 gravityForce(0.0f, gravity * obj->getMass() * timeStep, 0.0f);
        obj->addForce(gravityForce);

        // 应用力，更新速度和位置
        glm::vec3 acceleration = obj->getForce() / obj->getMass();
        obj->setVelocity(obj->getVelocity() + acceleration * timeStep);
        glm::vec3 newPosition = obj->getPosition() + obj->getVelocity() * timeStep;

        // 地面碰撞检测
        Collider* collider = obj->getCollider();
        if (collider != nullptr && collider->type == COLLIDER_TYPE_SPHERE)
        {
            float radius = collider->sphere.radius;
            if (newPosition.y - radius < -0.1f) // 地面 y = -0.1
            {
                newPosition.y = -0.1f + radius;
                glm::vec3 vel = obj->getVelocity();
                vel.y = -vel.y * 0.8f; // 反弹并施加阻尼
                obj->setVelocity(vel);
            }
        }
        obj->setPosition(newPosition);

        // 清除力
        obj->clearForces();
    }

    // 更新宽阶段
    broadPhase.update();

    // 宽阶段：收集可能碰撞对
    std::vector<std::pair<Entity*, Entity*>> potentialCollisions;
    broadPhase.collectCollisionPairs(potentialCollisions);

    // 窄阶段处理
    for (const auto& pair : potentialCollisions)
    {
        Entity* obj1 = pair.first;
        Entity* obj2 = pair.second;
        Collider* collider1 = obj1->getCollider();
        Collider* collider2 = obj2->getCollider();

        if (collider1 && collider2)
        {
            std::vector<glm::vec3> simplex;
            if (narrowPhase.gjkCollision(collider1, obj1->getPosition(),
                                        collider2, obj2->getPosition(),
                                        simplex))
            {
                std::cout << "Collision between " << obj1 << " and " << obj2 << " detected!" << std::endl;

                // 使用 EPA 计算穿透深度和法线
                float penetration = 0.0f;
                glm::vec3 normal(0.0f);
                if (narrowPhase.epaPenetration(collider1, obj1->getPosition(),
                                             collider2, obj2->getPosition(),
                                             simplex, penetration, normal))
                {
                    // 确保法线方向正确（指向 obj1 到 obj2）
                    glm::vec3 direction = obj2->getPosition() - obj1->getPosition();
                    if (glm::dot(normal, direction) < 0)
                        normal = -normal;

                    // 碰撞响应：基于 EPA 的穿透深度和法线
                    glm::vec3 correction = normal * (penetration * 0.5f);
                    obj1->setPosition(obj1->getPosition() - correction);
                    obj2->setPosition(obj2->getPosition() + correction);

                    glm::vec3 relativeVelocity = obj1->getVelocity() - obj2->getVelocity();
                    float velocityAlongNormal = glm::dot(relativeVelocity, normal);
                    if (velocityAlongNormal > 0) continue;

                    float restitution = 0.8f;
                    float impulse = -(1 + restitution) * velocityAlongNormal;
                    impulse /= (1 / obj1->getMass() + 1 / obj2->getMass());

                    glm::vec3 impulseVector = normal * impulse;
                    obj1->setVelocity(obj1->getVelocity() - impulseVector / obj1->getMass());
                    obj2->setVelocity(obj2->getVelocity() + impulseVector / obj2->getMass());
                }
                else
                {
                    std::cerr << "EPA failed, using fallback response" << std::endl;
                    // 备用方案：使用简化的碰撞响应
                    glm::vec3 distanceVector = obj2->getPosition() - obj1->getPosition();
                    float distance = glm::length(distanceVector);
                    if (distance < 0.0001f) distance = 0.0001f;
                    normal = distanceVector / distance;
                    penetration = 0.1f;

                    glm::vec3 correction = normal * (penetration * 0.5f);
                    obj1->setPosition(obj1->getPosition() - correction);
                    obj2->setPosition(obj2->getPosition() + correction);

                    glm::vec3 relativeVelocity = obj1->getVelocity() - obj2->getVelocity();
                    float velocityAlongNormal = glm::dot(relativeVelocity, normal);
                    if (velocityAlongNormal > 0) continue;

                    float restitution = 0.8f;
                    float impulse = -(1 + restitution) * velocityAlongNormal;
                    impulse /= (1 / obj1->getMass() + 1 / obj2->getMass());

                    glm::vec3 impulseVector = normal * impulse;
                    obj1->setVelocity(obj1->getVelocity() - impulseVector / obj1->getMass());
                    obj2->setVelocity(obj2->getVelocity() + impulseVector / obj2->getMass());
                }
            }
        }
    }
}