#ifndef SPHERE_MESH_H
#define SPHERE_MESH_H

#include "mesh.h"

class SphereMesh : public Mesh
{
    public:
        SphereMesh(float radius = 0.1f, int sectors = 20, int stacks = 20);
};

#endif