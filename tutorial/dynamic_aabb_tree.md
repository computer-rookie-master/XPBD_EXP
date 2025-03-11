### 设计目标
- **二叉树结构**：
  - 使用指针实现动态 AABB 树，每个节点有左子节点（`left`）和右子节点（`right`），以及父节点（`parent`）。
  - 叶节点存储 `Entity*`，内部节点存储两个子节点的 AABB 并集。
- **树更新**：
  - 每帧更新叶节点的 AABB，并自下而上调整父节点的 AABB。
  - 插入新节点时，动态平衡树（当前使用简单插入，后续可优化为 SAH - Surface Area Heuristic）。
- **碰撞检测**：
  - 使用树遍历（递归）收集可能碰撞的物体对，时间复杂度降至 \(O(n \log n)\)。

---

### 修改文件

#### 1. 修改 `src/physics/collision_broad_phase.h`
- **目标**：调整 `AABBNode` 结构，移除 `std::vector<AABBNode*>`，并声明树根节点。
- **内容**：
```cpp
#ifndef COLLISION_BROAD_PHASE_H
#define COLLISION_BROAD_PHASE_H

#include <glm/glm.hpp>
#include <vector>
#include "physics/entity.h"
#include "physics/collider.h"

struct AABB {
    glm::vec3 min;
    glm::vec3 max;
};

struct AABBNode {
    AABB aabb;
    Entity* entity;  // 叶节点存储实体
    AABBNode* left;
    AABBNode* right;
    AABBNode* parent;
    bool isLeaf;

    AABBNode() : entity(nullptr), left(nullptr), right(nullptr), parent(nullptr), isLeaf(false) {}
};

class CollisionBroadPhase
{
public:
    CollisionBroadPhase();
    ~CollisionBroadPhase();

    void addObject(Entity* entity);
    void update();
    void collectCollisionPairs(std::vector<std::pair<Entity*, Entity*>>& pairs);

private:
    AABBNode* root;  // 树的根节点

    AABB computeAABB(const Collider* collider, const glm::vec3& position);
    void insertAABBNode(AABBNode* node);
    void updateAABBNode(AABBNode* node);
    void collectCollisionPairs(AABBNode* node1, AABBNode* node2, std::vector<std::pair<Entity*, Entity*>>& pairs);
    bool checkAABBCollision(const AABB& aabb1, const AABB& aabb2);
    AABB mergeAABB(const AABB& aabb1, const AABB& aabb2);
    void deleteTree(AABBNode* node);  // 辅助删除树
};

#endif
```

- **变化**：
  - 移除 `std::vector<AABBNode*> aabbTree`，改为单个 `root` 指针。
  - 添加 `updateAABBNode` 和 `mergeAABB` 函数声明，用于更新树结构。
  - 添加 `deleteTree` 用于清理内存。

#### 2. 修改 `src/physics/collision_broad_phase.cpp`
- **目标**：实现二叉树的插入、更新和遍历逻辑。
- **内容**：
```cpp
#include "physics/collision_broad_phase.h"
#include <iostream>
#include <cmath>
#include <algorithm>

CollisionBroadPhase::CollisionBroadPhase() : root(nullptr)
{
    std::cout << "CollisionBroadPhase Initialized" << std::endl;
}

CollisionBroadPhase::~CollisionBroadPhase()
{
    deleteTree(root);
}

void CollisionBroadPhase::deleteTree(AABBNode* node)
{
    if (!node) return;
    deleteTree(node->left);
    deleteTree(node->right);
    delete node;
}

AABB CollisionBroadPhase::computeAABB(const Collider* collider, const glm::vec3& position)
{
    AABB aabb;
    aabb.min = glm::vec3(std::numeric_limits<float>::max());
    aabb.max = glm::vec3(-std::numeric_limits<float>::max());

    if (collider->type == COLLIDER_TYPE_SPHERE)
    {
        float radius = collider->sphere.radius;
        aabb.min = position - glm::vec3(radius);
        aabb.max = position + glm::vec3(radius);
    }
    else if (collider->type == COLLIDER_TYPE_CONVEX_HULL)
    {
        for (const auto& vertex : collider->convexHull.vertices)
        {
            glm::vec3 worldVertex = position + vertex;
            aabb.min = glm::min(aabb.min, worldVertex);
            aabb.max = glm::max(aabb.max, worldVertex);
        }
    }
    return aabb;
}

bool CollisionBroadPhase::checkAABBCollision(const AABB& aabb1, const AABB& aabb2)
{
    return (aabb1.min.x <= aabb2.max.x && aabb1.max.x >= aabb2.min.x) &&
           (aabb1.min.y <= aabb2.max.y && aabb1.max.y >= aabb2.min.y) &&
           (aabb1.min.z <= aabb2.max.z && aabb1.max.z >= aabb2.min.z);
}

AABB CollisionBroadPhase::mergeAABB(const AABB& aabb1, const AABB& aabb2)
{
    AABB result;
    result.min = glm::min(aabb1.min, aabb2.min);
    result.max = glm::max(aabb1.max, aabb2.max);
    return result;
}

void CollisionBroadPhase::insertAABBNode(AABBNode* node)
{
    if (!root)
    {
        root = node;
        return;
    }

    // 简单插入：找到一个叶节点，将其分裂
    std::vector<AABBNode*> stack;
    stack.push_back(root);

    while (!stack.empty())
    {
        AABBNode* current = stack.back();
        stack.pop_back();

        if (current->isLeaf)
        {
            // 将当前叶节点分裂
            AABBNode* newParent = new AABBNode();
            newParent->left = current;
            newParent->right = node;
            newParent->parent = current->parent;
            newParent->isLeaf = false;
            current->parent = newParent;
            node->parent = newParent;

            // 更新父节点的 AABB
            newParent->aabb = mergeAABB(current->aabb, node->aabb);
            if (newParent->parent)
            {
                AABBNode* parent = newParent->parent;
                while (parent)
                {
                    parent->aabb = mergeAABB(parent->left->aabb, parent->right->aabb);
                    parent = parent->parent;
                }
            }
            else
            {
                root = newParent;
            }
            break;
        }
        else
        {
            if (current->left) stack.push_back(current->left);
            if (current->right) stack.push_back(current->right);
        }
    }
}

void CollisionBroadPhase::updateAABBNode(AABBNode* node)
{
    if (!node) return;

    if (node->isLeaf && node->entity)
    {
        node->aabb = computeAABB(node->entity->getCollider(), node->entity->getPosition());
    }
    else
    {
        updateAABBNode(node->left);
        updateAABBNode(node->right);
        node->aabb = mergeAABB(node->left->aabb, node->right->aabb);
    }
}

void CollisionBroadPhase::update()
{
    updateAABBNode(root);
}

void CollisionBroadPhase::collectCollisionPairs(AABBNode* node1, AABBNode* node2, std::vector<std::pair<Entity*, Entity*>>& pairs)
{
    if (!node1 || !node2) return;

    if (checkAABBCollision(node1->aabb, node2->aabb))
    {
        if (node1->isLeaf && node2->isLeaf)
        {
            pairs.emplace_back(node1->entity, node2->entity);
        }
        else
        {
            if (node1->isLeaf || (!node1->left && !node1->right))
            {
                collectCollisionPairs(node1, node2->left, pairs);
                collectCollisionPairs(node1, node2->right, pairs);
            }
            else if (node2->isLeaf || (!node2->left && !node2->right))
            {
                collectCollisionPairs(node1->left, node2, pairs);
                collectCollisionPairs(node1->right, node2, pairs);
            }
            else
            {
                collectCollisionPairs(node1->left, node2->left, pairs);
                collectCollisionPairs(node1->left, node2->right, pairs);
                collectCollisionPairs(node1->right, node2->left, pairs);
                collectCollisionPairs(node1->right, node2->right, pairs);
            }
        }
    }
}

void CollisionBroadPhase::addObject(Entity* entity)
{
    if (!entity || !entity->getCollider())
    {
        std::cerr << "Error: Cannot add entity to CollisionBroadPhase without collider" << std::endl;
        return;
    }

    AABBNode* node = new AABBNode();
    node->entity = entity;
    node->isLeaf = true;
    node->aabb = computeAABB(entity->getCollider(), entity->getPosition());
    insertAABBNode(node);
}

void CollisionBroadPhase::collectCollisionPairs(std::vector<std::pair<Entity*, Entity*>>& pairs)
{
    pairs.clear();
    if (root)
    {
        collectCollisionPairs(root, root, pairs);
    }
}
```

- **变化**：
  - `insertAABBNode` 实现了一个简单的二叉树插入逻辑：找到一个叶节点，将其分裂为新的父节点，并插入新节点。
  - `updateAABBNode` 递归更新所有节点的 AABB，从叶节点向上合并。
  - `collectCollisionPairs` 实现递归树遍历，检查 AABB 重叠，仅在叶节点对之间生成碰撞对。
  - `deleteTree` 确保析构时清理所有节点。

#### 3. 其他文件
- **无需修改**：
  - `xpbd.h`, `xpbd.cpp`, `collider.h`, `collider.cpp`, `entity.h`, `entity.cpp`, `main.cpp`, `CMakeLists.txt` 保持不变，因为 `CollisionBroadPhase` 的更改是内部实现，接口保持一致。

---

### 实现说明
- **二叉树结构**：
  - 当前实现是一个不平衡的二叉树，插入时简单分裂叶节点。
  - 树的深度可能因插入顺序而异，未来可优化为平衡树（如使用 SAH 或重构算法）。

- **性能**：
  - 宽阶段复杂度从 \(O(n^2)\) 降至 \(O(n \log n)\)，因为树遍历只检查重叠的子树。
  - 对于当前 3 个物体，效果可能不明显，但随着物体数量增加，性能提升会更显著。

- **局限性**：
  - 树未实现动态平衡，可能导致某些情况下深度过高（退化为链表）。
  - 未优化 AABB 合并（可加入面积启发式 SAH）。

---