### **使用 SDF 处理碰撞的一般逻辑**
SDF 是一种表示物体表面距离的函数，对于空间中的任意点，返回该点到最近表面的距离，并通过正负号区分点在物体内部还是外部。通常用于物理模拟、渲染和碰撞检测。以下是使用 SDF 处理碰撞的核心逻辑：

#### **1. SDF 的定义**
- **正值**：点在物体外部，距离为点到最近表面的正值。
- **负值**：点在物体内部，距离为点到最近表面的负值。
- **零值**：点恰好在物体表面。

例如，对于一个球体，SDF 可以定义为：
\[ SDF(p) = \|p - c\| - r \]
- \( p \)：查询点。
- \( c \)：球心。
- \( r \)：半径。
- 若 \( SDF(p) > 0 \)，点在球外；若 \( < 0 \)，点在球内。

#### **2. 碰撞检测逻辑**
- **输入**：两个物体 \( A \) 和 \( B \)，分别有 SDF 函数 \( SDF_A \) 和 \( SDF_B \)，以及它们的位置 \( posA \) 和 \( posB \)。
- **步骤**：
  1. **计算对方中心点的 SDF 值**：
     - \( SDF_A(posB) \)：物体 \( B \) 中心相对于 \( A \) 的有符号距离。
     - \( SDF_B(posA) \)：物体 \( A \) 中心相对于 \( B \) 的有符号距离。
  2. **判断是否碰撞**：
     - 计算两中心间距离 \( d = \|posB - posA\| \)。
     - 如果 \( d < SDF_A(posB) + SDF_B(posA) \)（忽略负值情况），可能发生碰撞。
     - 更精确地，若 \( SDF_A(posB) < 0 \) 或 \( SDF_B(posA) < 0 \)，表明一个物体的中心已进入另一个物体的内部，确认碰撞。
  3. **计算穿透深度**：
     - 若 \( SDF_A(posB) < 0 \)，穿透深度可近似为 \( |SDF_A(posB)| \)。
     - 若 \( SDF_B(posA) < 0 \)，穿透深度可近似为 \( |SDF_B(posA)| \)。
     - 取两者中的较大值或某种组合（如最小值）作为最终穿透深度。
  4. **确定法线方向**：
     - 法线通常沿两中心连线方向（\( posB - posA \)），或反向（取决于哪个物体更深入）。
     - 更精确的法线可通过 SDF 梯度计算：\( \nabla SDF_A(posB) \) 或 \( \nabla SDF_B(posA) \)。

#### **3. 优点**
- **精确性**：能处理复杂形状的碰撞（如凹面物体），不仅仅是球或盒子。
- **穿透信息**：直接提供穿透深度和方向，便于位置修正。
- **平滑性**：SDF 是连续函数，避免离散几何方法的不连续性。

#### **4. 局限性**
- **计算成本**：对于复杂网格，精确 SDF 计算较慢，需优化（如近似或预计算）。
- **内外判断**：需要准确区分点在物体内部还是外部，依赖网格的闭合性和法线一致性。

#### **5. 典型实现**
- **简单形状**：如球体、立方体，直接用解析公式计算 SDF。
- **网格物体**：
  - 遍历顶点或三角形，计算到最近表面的距离。
  - 通过点到三角形的投影和法线方向判断内外。
- **碰撞响应**：
  - 使用穿透深度沿法线方向分离物体。
  - 结合冲量调整速度。

---

### **当前代码的问题**
你的 `computeSDF` 实现：
- 只计算无符号距离（`minDistance`），未区分内外。
- `resolveSDFCollision` 使用 \( d < SDF_A + SDF_B \) 判断碰撞，但未利用负值信息。
- 法线方向基于中心连线，未考虑 SDF 梯度。

目标：
- 将 `computeSDF` 改为真正的有符号距离场。
- 优化 `resolveSDFCollision`，利用内外信息提高精度。

---

### **修改后的代码**
以下是修改后的 `collision_narrow_phase.cpp`，将 SDF 转换为有符号距离场，并细化碰撞处理逻辑。

```cpp
#include "physics/collision_narrow_phase.h"
#include <cmath>
#include <iostream>
#include <limits>

CollisionNarrowPhase::CollisionNarrowPhase() {}

CollisionNarrowPhase::~CollisionNarrowPhase() {}

float CollisionNarrowPhase::computeSDF(const Entity* entity, const glm::vec3& point) 
{
    // 获取实体的网格和位置
    const Mesh* mesh = entity->getMesh();
    glm::vec3 entityPos = entity->getPosition();
    const std::vector<glm::vec3>& vertices = mesh->getPositions();
    const std::vector<unsigned int>& indices = mesh->getIndices();

    if (vertices.empty() || indices.empty()) {
        // 默认球形 SDF（以中心为原点，半径 0.1）
        float distance = glm::length(point - entityPos) - 0.1f;
        return distance;
    }

    // 计算点到网格表面的最小距离
    float minDistance = std::numeric_limits<float>::max();
    glm::vec3 closestPoint;

    // 遍历所有三角形（假设 indices 每 3 个为一组）
    for (size_t i = 0; i < indices.size(); i += 3) {
        // 获取三角形顶点（世界坐标）
        glm::vec3 v0 = entityPos + vertices[indices[i]];
        glm::vec3 v1 = entityPos + vertices[indices[i + 1]];
        glm::vec3 v2 = entityPos + vertices[indices[i + 2]];

        // 计算点到三角形的最近点
        glm::vec3 edge0 = v1 - v0;
        glm::vec3 edge1 = v2 - v0;
        glm::vec3 v0p = point - v0;

        float a = glm::dot(edge0, edge0);
        float b = glm::dot(edge0, edge1);
        float c = glm::dot(edge1, edge1);
        float d = glm::dot(edge0, v0p);
        float e = glm::dot(edge1, v0p);
        float det = a * c - b * b;

        float s = (c * d - b * e) / det;
        float t = (a * e - b * d) / det;

        // 限制 s 和 t 到三角形内
        s = glm::clamp(s, 0.0f, 1.0f);
        t = glm::clamp(t, 0.0f, 1.0f);
        if (s + t > 1.0f) {
            s = glm::clamp(s, 0.0f, 1.0f - t);
            t = glm::clamp(t, 0.0f, 1.0f - s);
        }

        // 计算最近点
        glm::vec3 closest = v0 + s * edge0 + t * edge1;
        float distance = glm::length(point - closest);
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = closest;
        }
    }

    // 判断内外（使用最近点的法线方向）
    glm::vec3 direction = point - closestPoint;
    // 假设网格有法线数据（这里简化为从中心指向外部）
    glm::vec3 avgNormal(0.0f);
    for (size_t i = 0; i < indices.size(); i += 3) {
        glm::vec3 v0 = vertices[indices[i]];
        glm::vec3 v1 = vertices[indices[i + 1]];
        glm::vec3 v2 = vertices[indices[i + 2]];
        avgNormal += glm::normalize(glm::cross(v1 - v0, v2 - v0));
    }
    avgNormal = glm::normalize(avgNormal);

    // 若 direction 与法线反向，则点在内部
    float sign = (glm::dot(direction, avgNormal) >= 0.0f) ? 1.0f : -1.0f;
    return sign * minDistance;
}

bool CollisionNarrowPhase::resolveSDFCollision(const Entity* entityA, const glm::vec3& posA,
                                               const Entity* entityB, const glm::vec3& posB,
                                               float& penetration, glm::vec3& normal) 
{
    // 计算两实体中心的 SDF
    float sdfAatB = computeSDF(entityA, posB); // B 中心相对于 A 的 SDF
    float sdfBatA = computeSDF(entityB, posA); // A 中心相对于 B 的 SDF

    // 计算两中心间的距离
    glm::vec3 direction = posB - posA;
    float distance = glm::length(direction);

    // 检查是否碰撞
    if (sdfAatB < 0.0f || sdfBatA < 0.0f) {
        // 至少一个中心在另一个物体内部，确认碰撞
        penetration = std::max(std::abs(sdfAatB), std::abs(sdfBatA));

        // 法线方向：从深入较深的物体指向另一个
        normal = (distance > 0.0001f) ? direction / distance : glm::vec3(0.0f, 1.0f, 0.0f);
        if (sdfAatB < sdfBatA) {
            normal = -normal; // B 更深入 A，法线从 B 指向 A
        }

        return true;
    }

    // 若中心间距离小于 SDF 和（正值情况），仍可能碰撞
    float threshold = sdfAatB + sdfBatA;
    if (distance < threshold && threshold > 0.0f) {
        penetration = threshold - distance;
        normal = (distance > 0.0001f) ? direction / distance : glm::vec3(0.0f, 1.0f, 0.0f);
        if (sdfAatB < sdfBatA) {
            normal = -normal;
        }
        return true;
    }

    return false;
}

bool CollisionNarrowPhase::detectCollision(const Entity* entityA, const glm::vec3& posA,
                                           const Entity* entityB, const glm::vec3& posB,
                                           float& penetration, glm::vec3& normal) 
{
    // 调用 SDF 碰撞解析
    bool collided = resolveSDFCollision(entityA, posA, entityB, posB, penetration, normal);

    if (collided) {
        std::cout << "Collision detected: Penetration = " << penetration
                  << ", Normal = (" << normal.x << ", " << normal.y << ", " << normal.z << ")\n";
    }
    return collided;
}
```

---

### **改动说明**
#### **1. `computeSDF`**
- **从无符号到有符号**：
  - 原代码只计算最小距离，未区分内外。
  - 新代码：
    - 遍历三角形，计算点到最近表面的距离（使用 barycentric 坐标）。
    - 通过法线方向判断内外：
      - 计算平均法线（假设网格法线一致）。
      - 若点到最近点的方向与法线同向，则为正（外部）；反向则为负（内部）。
- **优化**：
  - 若网格无索引，默认按球形 SDF 处理。
  - 使用三角形而非顶点，提高精度。

#### **2. `resolveSDFCollision`**
- **碰撞判断**：
  - 原代码仅用 \( d < SDF_A + SDF_B \)，未利用负值。
  - 新逻辑：
    - 若 \( SDF_A(posB) < 0 \) 或 \( SDF_B(posA) < 0 \)，确认碰撞（中心进入内部）。
    - 若 \( d < SDF_A + SDF_B \) 且均为正值，视为表面接近碰撞。
- **穿透深度**：
  - 若有负值，取绝对值最大者作为穿透深度。
  - 若均为正值，仍用 \( threshold - distance \)。
- **法线方向**：
  - 根据哪个 SDF 值更小（更深入）调整方向。
  - 默认沿中心连线，距离过小时用默认法线 `(0, 1, 0)`。

#### **3. `detectCollision`**
- 未改动，仅保留日志输出。

---

### **效果**
- **小球与地面**：
  - 小球中心进入地面时，`sdfAatB < 0`，触发碰撞。
  - 穿透深度精确反映深入程度，法线指向 `(0, 1, 0)`。
- **小球与小球**：
  - 两球接近时，若 \( d < SDF_A + SDF_B \)，检测碰撞。
  - 若一球中心进入另一球，穿透深度更大，法线更准确。
- **精度提升**：
  - SDF 区分内外，避免误判。
  - 三角形计算比顶点更精确。

---