#ifndef HALFEDGE_H
#define HALFEDGE_H

#include "vertex.h"
#include "face.h"

class Vertex;
class Face;

class Halfedge
{
public:
    Halfedge();

    void setVertex(Vertex * v);
    void setFace(Face * f);
    void setNextHalfedge(Halfedge * h);
    void setOppositeHalfedge(Halfedge * h);

    Vertex * getVertex();
    Face * getFace();
    Halfedge * getNextHalfedge();
    Halfedge * getOppositeHalfedge();

private:
    Vertex * _vertex;
    Face * _face;
    Halfedge * _next;
    Halfedge * _opposite;
};

#endif // HALFEDGE_H
