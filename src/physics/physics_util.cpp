#include "physics_util.h"

float get_sqr_magnitude(glm::vec3 vec)
{
    return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}

glm::vec3 MultiplyVector(const glm::mat4x4& mat, const glm::vec3& vec)
{
    // 将 vec 扩展为 vec4，w = 0（方向向量）
    glm::vec4 temp = mat * glm::vec4(vec, 0.0f);
    // 只返回前三个分量
    return glm::vec3(temp.x, temp.y, temp.z);
}

glm::mat4x4 get_Cross_Matrix(glm::vec3 a)
{
    glm::mat4x4 A = {0};
    A[0][0] = 0;
    A[0][1] = -a[2];
    A[0][2] = a[1];
    A[1][0] = a[2];
    A[1][1] = 0;
    A[1][2] = -a[0];
    A[2][0] = -a[1];
    A[2][1] = a[0];
    A[2][2] = 0;
    A[3][3] = 1;
    return A;
}

glm::mat4x4 Matrix_Subtraction(glm::mat4x4 a, glm::mat4x4 b)
{
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            a[i][j] -= b[i][j];
        }
    }
    return a;
}

glm::mat4x4 Matrix_Multiply_Float(glm::mat4x4 a, float b)
{
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            a[i][j] *= b;
        }
    }
    return a;
}

glm::quat Add(glm::quat a, glm::quat b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
    return a;
}