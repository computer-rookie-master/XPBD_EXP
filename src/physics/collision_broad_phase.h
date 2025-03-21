#ifndef COLLISION_BROAD_PHASE_H
#define COLLISION_BROAD_PHASE_H

#include <glm/glm.hpp>
#include <vector>
#include "physics/entity.h"

struct AABB 
{
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
        AABB computeAABB(const Entity* entity);

    private:
        AABBNode* root;
        
        void insertAABBNode(AABBNode* node);
        void updateAABBNode(AABBNode* node);
        void collectCollisionPairs(AABBNode* node1, AABBNode* node2, std::vector<std::pair<Entity*, Entity*>>& pairs);
        bool checkAABBCollision(const AABB& aabb1, const AABB& aabb2);
        AABB mergeAABB(const AABB& aabb1, const AABB& aabb2);
        void deleteTree(AABBNode* node);

        void printTree(AABBNode* node, int depth = 0);
};

#endif