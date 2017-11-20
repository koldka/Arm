#ifndef VERTEX_H
#define VERTEX_H

#include "halfedge.h"

class Halfedge;

class Vertex
{
public:
    Vertex(float x, float y, float z);

    float x();
    float y();
    float z();
    Halfedge * h();
    unsigned long id();

    void setX(float x);
    void setY(float y);
    void setZ(float z);
    void setH(Halfedge * h);
    void setId(unsigned long id);

private:
    float _x;
    float _y;
    float _z;
    Halfedge * _h;
    unsigned long _id;
};

#endif // VERTEX_H
