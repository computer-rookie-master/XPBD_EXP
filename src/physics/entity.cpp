#include "physics/entity.h"
#include <iostream>

Entity::Entity(Mesh* m, const glm::vec3& pos, float mas)
    : mesh(m), position(pos), linear_velocity(0.0f), angular_velocity(0.0f), rotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)), mass(mas), inverse_mass((mas != 0.0f) ? 1.0f / mas : 0.0f), force(0.0f), torque(0.0f), fixed(false)
{
    if (mesh == nullptr) {
        std::cerr << "Error: Entity created with null Mesh pointer" << std::endl;
    }
}

Entity::~Entity()
{
    // 其他资源（如 mesh）由调用者管理，避免双重删除
}