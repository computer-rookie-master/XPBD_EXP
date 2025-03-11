#ifndef ENEITY_H
#define ENEITY_H

#include <glm/glm.hpp>
#include "render/mesh.h"
#include "physics/collider.h"

class Entity
{
    public:
        Entity(Mesh* mesh, const glm::vec3& position, float mass = 1.0f, Collider* collider = nullptr);
        ~Entity();

        // 获取和设置物理属性
        Mesh* getMesh() const { return mesh; }
        glm::vec3 getPosition() const { return position; }
        glm::vec3 getVelocity() const { return velocity; }
        float getMass() const { return mass; }
        glm::vec3 getForce() const { return force; }
        Collider* getCollider() const { return collider; }

        void setPosition(const glm::vec3& pos) { position = pos; }
        void setVelocity(const glm::vec3& vel) { velocity = vel; }
        void addForce(const glm::vec3& f) { force += f; }
        void clearForces() { force = glm::vec3(0.0f); }
    private:
        Mesh* mesh;
        glm::vec3 position; // 世界位置
        glm::vec3 velocity;
        float mass;
        glm::vec3 force;
        Collider* collider;
};

#endif