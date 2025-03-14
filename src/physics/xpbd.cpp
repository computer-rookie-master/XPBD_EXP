#include "physics/xpbd.h"
#include "physics/collision_narrow_phase.h"
#include "physics/physics_util.h"
#include <GL/glew.h>
#include <iostream>
#include <cmath>
#include <vector>

XPBDSystem::XPBDSystem() : gravity(-9.81f), timeStep(1.0f / 60.0f) {}

void XPBDSystem::initialize()
{
    for (auto& obj : objects)
    {
        float m = 1.0f;
        std::vector<glm::vec3> vertices = obj->getMesh()->getPositions();
        glm::mat4x4 I_ref = glm::mat4x4(0.0f);
        for (int i = 0; i < vertices.size(); ++i)
        {
            obj->setMass(obj->getMass() + m);
            float diag = m * get_magnitude(vertices[i]);
            I_ref[0][0] += diag;
            I_ref[1][1] += diag;
            I_ref[2][2] += diag;
            I_ref[0][0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0][1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0][2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1][0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1][1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1][2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2][0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2][1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2][2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3][3] = 1.0f;
        obj->setIRef(I_ref);
    }
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
        // 应用重力和阻尼
        obj->setLinearVelocity(obj->getLinearVelocity() + glm::vec3(0.0f, gravity * timeStep, 0.0f));
        obj->setLinearVelocity(obj->getLinearVelocity() * 0.8f);
        obj->setAngularVelocity(obj->getAngularVelocity() * 0.8f);

        // 地面碰撞检测（假设球形，基于 Mesh 数据）
        const std::vector<glm::vec3>& vertices = obj->getMesh()->getPositions();
        float radius = 0.1f; // 默认半径，可改进为从 Mesh 计算
        if (obj->getPosition().y - radius < -0.1f)
        {
            glm::vec3 collisionPoint(obj->getPosition().x, obj->getPosition().y - radius, obj->getPosition().z);
            glm::vec3 normal(0.0f, 1.0f, 0.0f);
            glm::vec3 temp = obj->getLinearVelocity();
            temp.y = -temp.y * 0.6f; // 反弹系数 0.6
            obj->setLinearVelocity(temp);
        }
    }

    // 宽相检测
    broadPhase.update();
    std::vector<std::pair<Entity*, Entity*>> potentialCollisions;
    broadPhase.collectCollisionPairs(potentialCollisions);

    // 窄相处理
    for (const auto& pair : potentialCollisions)
    {
        Entity* obj1 = pair.first;
        Entity* obj2 = pair.second;

        float penetration = 0.0f;
        glm::vec3 normal(0.0f);
        if (narrowPhase.detectCollision(obj1, obj1->getPosition(), obj2, obj2->getPosition(), penetration, normal))
        {
            std::cout << "Collision between " << obj1 << " and " << obj2 << " detected!" << std::endl;

            // 确保法线方向正确（指向 obj2 到 obj1）
            glm::vec3 direction = obj2->getPosition() - obj1->getPosition();
            if (glm::dot(normal, direction) < 0)
                normal = -normal;

            applyImpulse(obj1, obj2, normal);

            // 修正位置
            glm::vec3 correction = normal * (penetration * 0.5f);
            obj1->setPosition(obj1->getPosition() - correction);
            obj2->setPosition(obj2->getPosition() + correction);
        }
    }

    // 更新位置和旋转
    for (auto& obj : objects)
    {
        glm::vec3 x_0 = obj->getPosition();
        glm::quat q_0 = obj->getRotation();

        glm::vec3 x = x_0 + obj->getLinearVelocity() * timeStep;

        glm::vec3 dw = 0.5f * obj->getAngularVelocity() * timeStep;
        glm::quat qw(dw.x, dw.y, dw.z, 0.0f);
        glm::quat q = Add(q_0, qw * q_0);

        obj->setPosition(x);
        obj->setRotation(q);
    }
}

void XPBDSystem::applyImpulse(Entity* obj1, Entity* obj2, glm::vec3 N)
{
    // 假设碰撞点为两实体中心的平均位置
    glm::vec3 P = (obj1->getPosition() + obj2->getPosition()) * 0.5f;

    // 处理 obj1 的冲量
    std::vector<glm::vec3> vertices1 = obj1->getMesh()->getPositions();
    glm::mat4x4 R1 = glm::mat4_cast(obj1->getRotation());
    glm::mat4x4 I_rot1 = R1 * obj1->getIRef() * glm::transpose(R1);
    glm::mat4x4 I_inv1 = glm::inverse(I_rot1);

    glm::vec3 r_i1 = P - obj1->getPosition();
    glm::vec3 Rri1 = MultiplyVector(R1, r_i1);
    glm::vec3 v1 = obj1->getLinearVelocity() + glm::cross(obj1->getAngularVelocity(), Rri1);

    // 处理 obj2 的冲量
    std::vector<glm::vec3> vertices2 = obj2->getMesh()->getPositions();
    glm::mat4x4 R2 = glm::mat4_cast(obj2->getRotation());
    glm::mat4x4 I_rot2 = R2 * obj2->getIRef() * glm::transpose(R2);
    glm::mat4x4 I_inv2 = glm::inverse(I_rot2);

    glm::vec3 r_i2 = P - obj2->getPosition();
    glm::vec3 Rri2 = MultiplyVector(R2, r_i2);
    glm::vec3 v2 = obj2->getLinearVelocity() + glm::cross(obj2->getAngularVelocity(), Rri2);

    // 相对速度
    glm::vec3 v_rel = v1 - v2;
    glm::vec3 v_N = glm::dot(v_rel, N) * N;
    glm::vec3 v_T = v_rel - v_N;

    // 反弹和摩擦系数
    float restitution = 0.5f;
    float friction = 0.2f;
    glm::vec3 v_N_new = -restitution * v_N;
    float a = glm::max(1.0f - friction * (1.0f + restitution) * get_magnitude(v_N) / (get_magnitude(v_T) + 1e-6f), 0.0f);
    glm::vec3 v_T_new = a * v_T;
    glm::vec3 v_new = v_N_new + v_T_new;

    // 计算冲量 J
    glm::mat4x4 Rri_star1 = get_Cross_Matrix(Rri1);
    glm::mat4x4 identity_matrix(1.0f);
    glm::mat4x4 K1 = Matrix_Subtraction(Matrix_Multiply_Float(identity_matrix, obj1->getInverseMass()), 
                                        Rri_star1 * I_inv1 * Rri_star1);
    glm::mat4x4 Rri_star2 = get_Cross_Matrix(Rri2);
    glm::mat4x4 K2 = Matrix_Subtraction(Matrix_Multiply_Float(identity_matrix, obj2->getInverseMass()), 
                                        Rri_star2 * I_inv2 * Rri_star2);
    glm::mat4x4 K = K1 + K2;
    glm::vec3 J = MultiplyVector(glm::inverse(K), v_new - v_rel);

    // 应用冲量
    obj1->setLinearVelocity(obj1->getLinearVelocity() + obj1->getInverseMass() * J);
    obj1->setAngularVelocity(obj1->getAngularVelocity() + MultiplyVector(I_inv1, glm::cross(Rri1, J)));
    obj2->setLinearVelocity(obj2->getLinearVelocity() - obj2->getInverseMass() * J);
    obj2->setAngularVelocity(obj2->getAngularVelocity() + MultiplyVector(I_inv2, glm::cross(Rri2, -J)));
}