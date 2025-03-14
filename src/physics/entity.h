#ifndef ENEITY_H
#define ENEITY_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "render/mesh.h"

class Entity
{
    public:
        Entity(Mesh* mesh, const glm::vec3& position, float mass = 1.0f);
        ~Entity();

        // 获取和设置物理属性
        Mesh* getMesh() const { return mesh; }

        glm::vec3 getPosition() const { return position; }
        void setPosition(const glm::vec3& pos) { position = pos; }

        glm::vec3 getLinearVelocity() const { return linear_velocity; }
        void setLinearVelocity(const glm::vec3& vel) { linear_velocity = vel; }

        glm::vec3 getAngularVelocity() const { return angular_velocity; }
        void setAngularVelocity(const glm::vec3& ang_vel) { angular_velocity = ang_vel; }

        glm::quat getRotation() const { return rotation; }
        void setRotation(const glm::quat& rot) { rotation = rot; }

        float getMass() const { return mass; }
        void setMass(const float m) { mass = m; }

        float getInverseMass() const { return (mass != 0.0f) ? 1.0f / mass : 0.0f; }

        glm::mat4x4 getIRef() const { return I_ref; }

        glm::vec3 getForce() const { return force; }
        void addForce(const glm::vec3& f) { force += f; }
        
        glm::vec3 getTorque() const { return torque; }
        void addTorque(const glm::vec3& t) { torque += t; }
        
        void setIRef(const glm::mat4x4& ref) { I_ref = ref; }
        
        void clearForces() { force = glm::vec3(0.0f); torque = glm::vec3(0.0f); }
        
        bool isFixed() const { return fixed; }
        void setFixed(bool f) { fixed = f; }

    private:
        Mesh* mesh;
        glm::vec3 position; // 世界位置
        glm::vec3 linear_velocity;
        glm::vec3 angular_velocity;
        glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // 四元数表示旋转
        float mass;
        float inverse_mass;
        glm::mat4x4 I_ref;
        glm::vec3 force;
        glm::vec3 torque;
        bool fixed;
};

#endif