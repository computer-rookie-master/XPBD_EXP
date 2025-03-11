#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <glm/glm.hpp>
#include <vector>
#include "physics/entity.h"

class Constraint
{
public:
    virtual ~Constraint() = default;
    virtual void solve(float timeStep, int iteration) = 0; // 求解约束
    virtual float getCompliance() const { return compliance; } // 返回顺应性
    virtual void setCompliance(float c) { compliance = c; } // 设置顺应性

protected:
    float compliance; // 顺应性 (1/stiffness)，单位为 dt^2
    std::vector<Entity*> entities; // 参与约束的实体
};

class DistanceConstraint : public Constraint
{
public:
    DistanceConstraint(Entity* e1, Entity* e2, float restLength, float stiffness = 1000.0f)
    {
        entities.push_back(e1);
        entities.push_back(e2);
        this->restLength = restLength;
        compliance = 1.0f / stiffness; // 刚度转换为顺应性
    }

    void solve(float timeStep, int iteration) override
    {
        Entity* e1 = entities[0];
        Entity* e2 = entities[1];
        glm::vec3 p1 = e1->getPosition();
        glm::vec3 p2 = e2->getPosition();
        glm::vec3 delta = p2 - p1;
        float currentDistance = glm::length(delta);
        if (currentDistance < 0.0001f) currentDistance = 0.0001f; // 避免除零

        float constraint = currentDistance - restLength;
        glm::vec3 gradient = delta / currentDistance;
        float w1 = 1.0f / e1->getMass();
        float w2 = 1.0f / e2->getMass();
        float denominator = w1 + w2 + compliance / (timeStep * timeStep);
        float lambda = -constraint / denominator;

        glm::vec3 correction = gradient * (lambda / (w1 + w2));
        e1->setPosition(p1 - correction * w1);
        e2->setPosition(p2 + correction * w2);
    }

private:
    float restLength; // 目标距离
};

class CollisionConstraint : public Constraint
{
public:
    CollisionConstraint(Entity* e1, Entity* e2, float penetration, const glm::vec3& normal)
    {
        entities.push_back(e1);
        entities.push_back(e2);
        this->penetration = penetration;
        this->normal = normal;
        compliance = 0.0f; // 刚性碰撞
    }

    void solve(float timeStep, int iteration) override
    {
        Entity* e1 = entities[0];
        Entity* e2 = entities[1];
        glm::vec3 p1 = e1->getPosition();
        glm::vec3 p2 = e2->getPosition();

        float constraint = penetration;
        glm::vec3 gradient = normal;
        float w1 = 1.0f / e1->getMass();
        float w2 = 1.0f / e2->getMass();
        float denominator = w1 + w2 + compliance / (timeStep * timeStep);
        float lambda = -constraint / denominator;

        glm::vec3 correction = gradient * (lambda / (w1 + w2));
        e1->setPosition(p1 - correction * w1);
        e2->setPosition(p2 + correction * w2);

        // 速度更新（反弹）
        glm::vec3 v1 = e1->getVelocity();
        glm::vec3 v2 = e2->getVelocity();
        glm::vec3 relativeVelocity = v1 - v2;
        float velocityAlongNormal = glm::dot(relativeVelocity, normal);
        if (velocityAlongNormal > 0) return;

        float restitution = 0.8f;
        float impulse = -(1.0f + restitution) * velocityAlongNormal;
        impulse /= (w1 + w2);
        glm::vec3 impulseVector = normal * impulse;
        e1->setVelocity(v1 - impulseVector * w1);
        e2->setVelocity(v2 + impulseVector * w2);
    }

private:
    float penetration;
    glm::vec3 normal;
};

#endif