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

    std::vector<AABBNode*> stack;
    stack.push_back(root);

    while (!stack.empty())
    {
        AABBNode* current = stack.back();
        stack.pop_back();

        if (current->isLeaf)
        {
            AABBNode* newParent = new AABBNode();
            newParent->left = current;
            newParent->right = node;
            newParent->parent = current->parent;
            newParent->isLeaf = false;
            current->parent = newParent;
            node->parent = newParent;

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
            if (node1->entity == node2->entity) return; // 过滤自碰撞
            // 确保 pair 是按指针地址排序的，避免重复
            Entity* first = std::min(node1->entity, node2->entity);
            Entity* second = std::max(node1->entity, node2->entity);
            pairs.emplace_back(first, second);
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
    // 去重（确保唯一性）
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    std::cout << "Potential collisions: " << pairs.size() << std::endl;
    for (const auto& pair : pairs)
    {
        std::cout << "Pair: (" << pair.first << ", " << pair.second << ")" << std::endl;
    }
}