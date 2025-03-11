#include "physics/entity.h"
#include <iostream>

Entity::Entity(Mesh* m, const glm::vec3& pos, float mas, Collider* c)
    : mesh(m), position(pos), velocity(0.0f), mass(mas), force(0.0f), collider(c)
{
    if (mesh == nullptr) {
        std::cerr << "Error: Entity created with null Mesh pointer" << std::endl;
    }
    if (collider == nullptr) {
        std::cerr << "Warning: Entity created with null Collider pointer" << std::endl;
    }
}

Entity::~Entity()
{
    delete collider; // 清理动态分配的 Collider
}