#ifndef PHYSICS_UTIL_H
#define PHYSICS_UTIL_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

float get_sqr_magnitude(glm::vec3 vec);

glm::vec3 MultiplyVector(const glm::mat4x4& mat, const glm::vec3& vec);

glm::mat4x4 get_Cross_Matrix(glm::vec3 a);

glm::mat4x4 Matrix_Subtraction(glm::mat4x4 a, glm::mat4x4 b);

glm::mat4x4 Matrix_Multiply_Float(glm::mat4x4 a, float b);

glm::quat Add(glm::quat a, glm::quat b);

#endif