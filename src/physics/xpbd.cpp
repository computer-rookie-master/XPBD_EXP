#include "physics/xpbd.h"
#include "physics/collision_narrow_phase.h"
#include "physics/physics_util.h"
#include <GL/glew.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>

XPBDSystem::XPBDSystem() : gravity(-9.81f), timeStep(1.0f / 120.0f) {}

void XPBDSystem::initialize()
{
    for (auto& obj : objects)
    {
        // 排除地面
        if (obj->getMass() == 0) continue;

        float m = 1.0f;
        float mass = 0;
        std::vector<glm::vec3> vertices = obj->getMesh()->getPositions();
        glm::mat4x4 I_ref = glm::mat4x4(0.0f);
        for (int i = 0; i < vertices.size(); ++i)
        {
            mass += m;
            float diag = m * get_sqr_magnitude(vertices[i]);
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
        obj->setMass(mass);
        obj->setIRef(I_ref);
        std::cout << " 物体惯量矩阵计算完成" << obj << "：mass：" << mass << "I_ref：" << I_ref[0][0] << std::endl;
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
        // 排除地面
        if (obj->getMass() == 0) continue;
        // 应用重力和阻尼
        obj->setLinearVelocity(obj->getLinearVelocity() + glm::vec3(0.0f, gravity * timeStep, 0.0f));
        obj->setLinearVelocity(obj->getLinearVelocity() * 0.99f);
        obj->setAngularVelocity(obj->getAngularVelocity() * 0.99f);
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
        
        bool isGroundInvolved = (obj1->getMass() == 0 || obj2->getMass() == 0);
            
        if(isGroundInvolved)
        {
            Entity* movingObj = (obj1->getMass() != 0) ? obj1 : obj2;
            applyImpulseWithGround(movingObj, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        }
        else
        {
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
            }
        }
    }

    // 更新位置和旋转
    for (auto& obj : objects)
    {
        glm::vec3 ve = obj->getLinearVelocity();
        std::cout << " 物体" << obj << "此刻的速度是" << ve.x << " " << ve.y << " " << ve.z << std::endl;

        glm::vec3 x_0 = obj->getPosition();
        glm::quat q_0 = obj->getRotation();

        glm::vec3 x = x_0 + obj->getLinearVelocity() * timeStep;

        glm::vec3 dw = 0.5f * obj->getAngularVelocity() * timeStep;
        glm::quat qw(dw.x, dw.y, dw.z, 0.0f);
        glm::quat q = glm::normalize(Add(q_0, qw * q_0));

        obj->setPosition(x);
        obj->setRotation(q);
    }
}

void XPBDSystem::applyImpulse(Entity* obj1, Entity* obj2, glm::vec3 N)
{
    // 假设碰撞点为两物体中心的连线上的点
    glm::vec3 T1 = obj1->getPosition();
    glm::vec3 T2 = obj2->getPosition();
    glm::vec3 P = (T1 + T2) * 0.5f; // 碰撞点近似为两中心的中点

    // 计算相对于各自中心的碰撞点偏移
    glm::vec3 r_i1 = P - T1;
    glm::vec3 r_i2 = P - T2;

    // 获取旋转矩阵
    glm::mat4x4 R1 = glm::mat4_cast(obj1->getRotation());
    glm::mat4x4 R2 = glm::mat4_cast(obj2->getRotation());

    // 变换到世界坐标
    glm::vec3 Rri1 = glm::mat3(R1) * r_i1;
    glm::vec3 Rri2 = glm::mat3(R2) * r_i2;

    // 计算碰撞点的速度
    glm::vec3 v_collision1 = obj1->getLinearVelocity() + glm::cross(obj1->getAngularVelocity(), r_i1);
    glm::vec3 v_collision2 = obj2->getLinearVelocity() + glm::cross(obj2->getAngularVelocity(), r_i2);

    // 相对速度
    glm::vec3 v_rel = v_collision1 - v_collision2;
    glm::vec3 v_N = glm::dot(v_rel, N) * N;
    glm::vec3 v_T = v_rel - v_N;

    // 反弹和摩擦系数
    float restitution = 0.5f;
    float friction = 0.2f;
    glm::vec3 v_N_new = -restitution * v_N;
    float v_T_len = glm::length(v_T);
    float v_N_len = glm::length(v_N);
    float a = glm::max(1.0f - friction * (1.0f + restitution) * v_N_len / (v_T_len + 0.01f), 0.0f);
    std::cout << "a is " << a << ", v_N_len: " << v_N_len << ", v_T_len: " << v_T_len << std::endl;
    glm::vec3 v_T_new = a * v_T;
    glm::vec3 v_new = v_N_new + v_T_new;

    // 计算冲量 J
    glm::mat4x4 I_rot1 = R1 * obj1->getIRef() * glm::transpose(R1);
    glm::mat4x4 I_inv1 = glm::inverse(I_rot1);
    glm::mat4x4 I_rot2 = R2 * obj2->getIRef() * glm::transpose(R2);
    glm::mat4x4 I_inv2 = glm::inverse(I_rot2);

    glm::mat4x4 Rri_star1 = get_Cross_Matrix(r_i1);
    glm::mat4x4 Rri_star2 = get_Cross_Matrix(r_i2);
    glm::mat4x4 identity_matrix(1.0f);
    glm::mat4x4 K1 = Matrix_Subtraction(Matrix_Multiply_Float(identity_matrix, obj1->getInverseMass()), Rri_star1 * I_inv1 * Rri_star1);
    glm::mat4x4 K2 = Matrix_Subtraction(Matrix_Multiply_Float(identity_matrix, obj2->getInverseMass()), Rri_star2 * I_inv2 * Rri_star2);
    glm::mat4x4 K = K1 + K2;
    std::cout << "K matrix: " << K[0][0] << " " << K[1][1] << " " << K[2][2] << std::endl;
    glm::vec3 J = glm::mat3(glm::inverse(K)) * (v_new - v_rel);
    std::cout << "J is " << J.x << " " << J.y << " " << J.z << std::endl;

    // 应用冲量
    obj1->setLinearVelocity(obj1->getLinearVelocity() + obj1->getInverseMass() * J);
    obj1->setAngularVelocity(obj1->getAngularVelocity() + glm::mat3(I_inv1) * glm::cross(r_i1, J));
    obj2->setLinearVelocity(obj2->getLinearVelocity() - obj2->getInverseMass() * J);
    obj2->setAngularVelocity(obj2->getAngularVelocity() + glm::mat3(I_inv2) * glm::cross(r_i2, -J));

    // 静止检测
    glm::vec3 v1 = obj1->getLinearVelocity();
    if (glm::length(v1) < 0.05f)
    {
        obj1->setLinearVelocity(glm::vec3(0.0f));
        obj1->setAngularVelocity(glm::vec3(0.0f));
        std::cout << "Object " << obj1 << " stopped due to low velocity" << std::endl;
    }

    glm::vec3 v2 = obj2->getLinearVelocity();
    if (glm::length(v2) < 0.05f)
    {
        obj2->setLinearVelocity(glm::vec3(0.0f));
        obj2->setAngularVelocity(glm::vec3(0.0f));
        std::cout << "Object " << obj2 << " stopped due to low velocity" << std::endl;
    }
}

void XPBDSystem::applyImpulseWithGround(Entity* obj, glm::vec3 P, glm::vec3 N)
{
    std::vector<glm::vec3> vertices = obj->getMesh()->getPositions();

    glm::mat4x4 R = glm::mat4_cast(obj->getRotation());
    glm::vec3 T = obj->getPosition();

    glm::vec3 sum(0.0f, 0.0f, 0.0f);
    int collisionNum = 0;

    for (int i = 0; i < vertices.size(); ++i)
    {
        glm::vec3 r_i = vertices[i];
        glm::vec3 Rri = glm::mat3(R) * r_i;
        glm::vec3 x_i = T + Rri;

        float d = glm::dot(x_i - P, N);
        if (d < 0.0f)
        {
            glm::vec3 v_i = obj->getLinearVelocity() + glm::cross(obj->getAngularVelocity(), Rri);
            float v_N_size = glm::dot(v_i, N);
            if (v_N_size < 0.0f)
            {
                sum += r_i;
                collisionNum ++;
            }
        }
    }

    if (collisionNum == 0) return;

    glm::mat4x4 I_rot = R * obj->getIRef() * glm::transpose(R);
    glm::mat4x4 I_inverse = glm::inverse(I_rot);
    glm::vec3 r_collision = sum / (float)collisionNum;
    glm::vec3 Rr_collision = glm::mat3(R) * r_collision;
    glm::vec3 v_collision = obj->getLinearVelocity() + glm::cross(obj->getAngularVelocity(), Rr_collision);

    float restitution = 0.5f;
    float friction = 0.2f;
    glm::vec3 v_N = glm::dot(v_collision, N) * N;
    glm::vec3 v_T = v_collision - v_N;
    glm::vec3 v_N_new = -restitution * v_N;
    float v_T_len = glm::length(v_T);
    float v_N_len = glm::length(v_N);
    float a = glm::max(1.0f - friction * (1.0f + restitution) * v_N_len / v_T_len, 0.0f);
    std::cout << "a is " << a << ", v_N_len: " << v_N_len << ", v_T_len: " << v_T_len << std::endl;
    glm::vec3 v_T_new = a * v_T;
    glm::vec3 v_new = v_N_new + v_T_new;

    glm::mat4x4 Rri_star = get_Cross_Matrix(Rr_collision);
    glm::mat4x4 identity_matrix(1.0f);
    glm::mat4x4 K = Matrix_Subtraction(Matrix_Multiply_Float(identity_matrix, 1.0f / obj->getMass()), Rri_star * I_inverse * Rri_star);
    std::cout << "K matrix: " << K[0][0] << " " << K[1][1] << " " << K[2][2] << std::endl;
    glm::vec3 J = glm::mat3(glm::inverse(K)) * (v_new - v_collision);
    std::cout << " J is " << J.x << " " << J.y << " " << J.z << std::endl;

    obj->setLinearVelocity(obj->getLinearVelocity() + 1.0f / obj->getMass() * J);
    obj->setAngularVelocity(obj->getAngularVelocity() + glm::mat3(I_inverse) * glm::cross(Rr_collision, J));

    // 静止检测
    glm::vec3 v = obj->getLinearVelocity();
    if (glm::length(v) < 0.02f)
    {
        obj->setLinearVelocity(glm::vec3(0.0f));
        obj->setAngularVelocity(glm::vec3(0.0f));
        std::cout << "Object stopped due to low velocity" << std::endl;
    }
}