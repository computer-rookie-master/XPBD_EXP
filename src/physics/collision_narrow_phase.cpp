#include "physics/collision_narrow_phase.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <set>

CollisionNarrowPhase::CollisionNarrowPhase() {}

CollisionNarrowPhase::~CollisionNarrowPhase() {}

glm::vec3 CollisionNarrowPhase::support(const Collider* collider, const glm::vec3& pos, const glm::vec3& direction)
{
    return pos + collider->getSupportPoint(direction);
}

bool CollisionNarrowPhase::sameDirection(const glm::vec3& direction, const glm::vec3& ao)
{
    return glm::dot(direction, ao) > 0;
}

bool CollisionNarrowPhase::doSimplex(std::vector<glm::vec3>& simplex, glm::vec3& direction)
{
    if (simplex.size() == 1)
    {
        glm::vec3 a = simplex[0];
        if (glm::length(a) < 0.0001f) return true; // 原点
        direction = -a;
        return false;
    }
    else if (simplex.size() == 2)
    {
        glm::vec3 a = simplex[1];
        glm::vec3 b = simplex[0];
        glm::vec3 ab = b - a;
        glm::vec3 ao = -a;

        float t = glm::dot(ao, ab) / glm::dot(ab, ab); // 投影参数
        if (t <= 0.0f)
        {
            simplex = {a};
            direction = -a;
        }
        else if (t >= 1.0f)
        {
            simplex = {b};
            direction = -b;
        }
        else
        {
            glm::vec3 closest = a + t * ab;
            direction = -closest;
        }
        return false;
    }
    else if (simplex.size() == 3)
    {
        glm::vec3 a = simplex[2];
        glm::vec3 b = simplex[1];
        glm::vec3 c = simplex[0];
        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ao = -a;

        glm::vec3 n = glm::cross(ab, ac);
        if (glm::dot(n, ao) > 0)
            n = -n; // 确保朝向原点

        float d1 = glm::dot(glm::cross(ab, n), ao);
        float d2 = glm::dot(glm::cross(ac, n), ao);
        float d3 = glm::dot(glm::cross(glm::vec3(0.0f) - ab - ac, n), ao);

        if (d1 > 0 && d2 > 0 && d3 > 0)
            return true; // 原点在三角形内

        if (d1 <= 0 && d2 > 0)
        {
            simplex = {b, a};
            return doSimplex(simplex, direction);
        }
        else if (d2 <= 0 && d1 > 0)
        {
            simplex = {c, a};
            return doSimplex(simplex, direction);
        }
        else
        {
            simplex = {b, c, a};
            direction = n;
            return false;
        }
    }
    else if (simplex.size() == 4)
    {
        glm::vec3 a = simplex[3];
        glm::vec3 b = simplex[2];
        glm::vec3 c = simplex[1];
        glm::vec3 d = simplex[0];
        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ad = d - a;
        glm::vec3 ao = -a;

        glm::vec3 abc = glm::cross(ab, ac);
        glm::vec3 acd = glm::cross(ac, ad);
        glm::vec3 adb = glm::cross(ad, ab);

        if (glm::dot(abc, ao) > 0)
        {
            simplex = {a, b, c};
            return doSimplex(simplex, direction);
        }
        else if (glm::dot(acd, ao) > 0)
        {
            simplex = {a, c, d};
            return doSimplex(simplex, direction);
        }
        else if (glm::dot(adb, ao) > 0)
        {
            simplex = {a, d, b};
            return doSimplex(simplex, direction);
        }
        else
        {
            return true; // 原点在四面体内
        }
    }
    return false;
}

bool CollisionNarrowPhase::gjkCollision(const Collider* colliderA, const glm::vec3& posA,
                                      const Collider* colliderB, const glm::vec3& posB,
                                      std::vector<glm::vec3>& simplex)
{
    simplex.clear();
    glm::vec3 direction = posA - posB; // 初始方向

    if (glm::length(direction) == 0.0f)
    {
        direction = glm::vec3(1.0f, 0.0f, 0.0f); // 默认方向
    }
    else if (colliderA->type == COLLIDER_TYPE_SPHERE && colliderB->type == COLLIDER_TYPE_SPHERE)
    {
        float radiusSum = colliderA->sphere.radius + colliderB->sphere.radius;
        if (glm::length(direction) < radiusSum) // 如果两球重叠，调整方向
            direction = glm::normalize(direction) * radiusSum * 0.1f; // 小偏移
    }

    std::cout << "GJK Start: Initial direction = " << direction.x << std::endl;
    glm::vec3 a = support(colliderA, posA, direction) - support(colliderB, posB, -direction);
    simplex.push_back(a);
    direction = -a;
    std::cout << "First point: " << a.x << std::endl;

    int iterations = 0;
    const int maxIterations = 64;

    while (iterations < maxIterations)
    {
        a = support(colliderA, posA, direction) - support(colliderB, posB, -direction);
        std::cout << "Iteration " << iterations << ": New point = " << a.x
                  << ", Dot = " << glm::dot(a, direction) << std::endl;
        if (glm::dot(a, direction) <= 0)
        {
            std::cout << "No collision: Cannot approach origin" << std::endl;
            return false;
        }
        simplex.push_back(a);

        if (doSimplex(simplex, direction))
        {
            std::cout << "Collision detected!" << std::endl;
            return true;
        }

        iterations++;
    }

    std::cout << "No collision: Max iterations reached" << std::endl;
    return false;
}

bool CollisionNarrowPhase::epaPenetration(const Collider* colliderA, const glm::vec3& posA,
                                        const Collider* colliderB, const glm::vec3& posB,
                                        const std::vector<glm::vec3>& initialSimplex,
                                        float& penetration, glm::vec3& normal)
{
    if (initialSimplex.size() != 4) {
        std::cerr << "EPA Error: Initial simplex must be a tetrahedron (4 points)" << std::endl;
        return false;
    }

    // 初始化多面体顶点
    std::vector<glm::vec3> vertices = initialSimplex;

    // 初始化多面体面（四面体的四个面）
    std::vector<PolytopeFace> faces = {
        PolytopeFace(0, 1, 2, vertices), // ABC
        PolytopeFace(0, 3, 1, vertices), // ADB
        PolytopeFace(0, 2, 3, vertices), // ACD
        PolytopeFace(1, 3, 2, vertices)  // BDC
    };

    const float tolerance = 0.0001f;
    const int maxIterations = 64;
    int iteration = 0;

    while (iteration < maxIterations)
    {
        // 找到离原点最近的面
        float minDistance = std::numeric_limits<float>::max();
        int closestFaceIndex = -1;

        for (int i = 0; i < faces.size(); ++i)
        {
            if (faces[i].distance < minDistance)
            {
                minDistance = faces[i].distance;
                closestFaceIndex = i;
            }
        }

        if (closestFaceIndex == -1) {
            std::cerr << "EPA Error: No closest face found" << std::endl;
            return false;
        }

        // 获取最近面的法线
        glm::vec3 searchDir = faces[closestFaceIndex].normal;

        // 计算新的支撑点
        glm::vec3 newPoint = support(colliderA, posA, searchDir) - support(colliderB, posB, -searchDir);
        float newDistance = glm::dot(newPoint, searchDir);

        // 检查是否收敛
        if (newDistance - minDistance < tolerance)
        {
            penetration = minDistance;
            normal = searchDir;
            std::cout << "EPA Converged: Penetration = " << penetration
                      << ", Normal = " << normal.x << std::endl;
            return true;
        }

        // 将新点添加到顶点列表
        int newVertexIndex = vertices.size();
        vertices.push_back(newPoint);

        // 收集需要移除的面和新的边
        std::vector<PolytopeFace> newFaces;
        std::set<std::pair<int, int>> edges;

        for (int i = 0; i < faces.size(); ++i)
        {
            if (glm::dot(newPoint, faces[i].normal) - faces[i].distance > 0)
            {
                // 面可见，移除并收集边缘
                int idx0 = faces[i].indices[0];
                int idx1 = faces[i].indices[1];
                int idx2 = faces[i].indices[2];
                edges.insert({std::min(idx0, idx1), std::max(idx0, idx1)});
                edges.insert({std::min(idx1, idx2), std::max(idx1, idx2)});
                edges.insert({std::min(idx2, idx0), std::max(idx2, idx0)});
            }
            else
            {
                // 面不可见，保留
                newFaces.push_back(faces[i]);
            }
        }

        // 根据边创建新面
        for (const auto& edge : edges)
        {
            int idx0 = edge.first;
            int idx1 = edge.second;
            newFaces.emplace_back(newVertexIndex, idx0, idx1, vertices);
        }

        faces = std::move(newFaces);
        iteration++;
    }

    std::cerr << "EPA Error: Max iterations reached" << std::endl;
    return false;
}