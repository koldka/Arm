#include "halfedge.h"

Halfedge::Halfedge()
{

}

void
Halfedge::setVertex(Vertex *v)
{
    this->_vertex = v;
}

void
Halfedge::setFace(Face *f)
{
    this->_face = f;
}

void
Halfedge::setNextHalfedge(Halfedge *h)
{
    this->_next = h;
}

void
Halfedge::setOppositeHalfedge(Halfedge *h)
{
    this->_opposite = h;
}

Vertex *
Halfedge::getVertex()
{
    return this->_vertex;
}

Face *
Halfedge::getFace()
{
    return this->_face;
}

Halfedge *
Halfedge::getNextHalfedge()
{
    return this->_next;
}

Halfedge *
Halfedge::getOppositeHalfedge()
{
    return this->_opposite;
}
