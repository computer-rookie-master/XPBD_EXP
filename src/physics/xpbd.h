#ifndef XPBD_H
#define XPBD_H

#include <glm/glm.hpp>
#include <vector>
#include "physics/entity.h"
#include "physics/collision_broad_phase.h"
#include "physics/collision_narrow_phase.h"

class XPBDSystem
{
    public:
        XPBDSystem();
        void addObject(Entity* entity);
        void run();
        void initialize();
        void applyImpulse(Entity* obj1, Entity* obj2, glm::vec3 N);
        const std::vector<Entity*>& getObjects() const { return objects; }

    private:
        std::vector<Entity*> objects;
        float gravity;
        float timeStep;
        CollisionBroadPhase broadPhase;
        CollisionNarrowPhase narrowPhase;
};

#endif