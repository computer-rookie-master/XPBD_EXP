#include <gtest/gtest.h>
#include "physics/collision_broad_phase.h"
#include "physics/entity.h"
#include "render/sphere_mesh.h"

// 测试夹具类，用于设置通用测试环境
class CollisionBroadPhaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 初始化测试所需的资源
        sphereMesh1 = new SphereMesh(0.1f, 20, 20);
        sphereMesh1->initialize();
        sphereMesh2 = new SphereMesh(0.1f, 20, 20);
        sphereMesh2->initialize();

        entity1 = new Entity(sphereMesh1, glm::vec3(0.0f, 0.0f, 0.0f), 1.0f);
        entity2 = new Entity(sphereMesh2, glm::vec3(0.2f, 0.0f, 0.0f), 1.0f);

        broadPhase = new CollisionBroadPhase();
    }

    void TearDown() override {
        delete broadPhase;
        delete entity1;
        delete entity2;
        delete sphereMesh1;
        delete sphereMesh2;
    }

    SphereMesh* sphereMesh1;
    SphereMesh* sphereMesh2;
    Entity* entity1;
    Entity* entity2;
    CollisionBroadPhase* broadPhase;
};

// 测试1：验证添加对象后 AABB 计算正确
TEST_F(CollisionBroadPhaseTest, AddObjectAndComputeAABB) {
    broadPhase->addObject(entity1);
    broadPhase->update();

    // 假设 computeAABB 返回的 AABB 是基于网格顶点的
    AABB aabb = broadPhase->computeAABB(entity1);
    EXPECT_NEAR(aabb.min.x, -0.1f, 0.001f); // 球体半径 0.1
    EXPECT_NEAR(aabb.max.x, 0.1f, 0.001f);
    EXPECT_NEAR(aabb.min.y, -0.1f, 0.001f);
    EXPECT_NEAR(aabb.max.y, 0.1f, 0.001f);
    EXPECT_NEAR(aabb.min.z, -0.1f, 0.001f);
    EXPECT_NEAR(aabb.max.z, 0.1f, 0.001f);
}

// 测试2：验证两个重叠实体被检测为潜在碰撞对
TEST_F(CollisionBroadPhaseTest, DetectOverlappingEntities) {
    broadPhase->addObject(entity1);
    broadPhase->addObject(entity2); // 距离 0.2，小于两球直径 0.2，预期重叠
    broadPhase->update();

    std::vector<std::pair<Entity*, Entity*>> pairs;
    broadPhase->collectCollisionPairs(pairs);

    ASSERT_EQ(pairs.size(), 1); // 应检测到一个碰撞对
    EXPECT_EQ(pairs[0].first, entity1);
    EXPECT_EQ(pairs[0].second, entity2);
}

// 测试3：验证两个不重叠实体不生成碰撞对
TEST_F(CollisionBroadPhaseTest, NoCollisionForSeparatedEntities) {
    entity2->setPosition(glm::vec3(0.5f, 0.0f, 0.0f)); // 距离 0.5，大于两球直径 0.2
    broadPhase->addObject(entity1);
    broadPhase->addObject(entity2);
    broadPhase->update();

    std::vector<std::pair<Entity*, Entity*>> pairs;
    broadPhase->collectCollisionPairs(pairs);

    EXPECT_EQ(pairs.size(), 0); // 不应检测到碰撞对
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}