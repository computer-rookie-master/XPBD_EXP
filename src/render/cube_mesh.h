#ifndef CUBE_MESH_H
#define CUBE_MESH_H

#include "mesh.h"

class CubeMesh : public Mesh 
{
    public:
        CubeMesh(float length = 0.2f, float width = 0.2f, float height = 0.2f); // 支持长宽高参数
};

#endif