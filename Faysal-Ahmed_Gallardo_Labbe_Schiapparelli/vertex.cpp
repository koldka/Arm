#include "vertex.h"

Vertex::Vertex(float x, float y, float z)
{
    this->_x = x;
    this->_y = y;
    this->_z = z;
}

float
Vertex::x()
{
    return this->_x;
}

float
Vertex::y()
{
    return this->_y;
}

float
Vertex::z()
{
    return this->_z;
}

Halfedge *
Vertex::h()
{
    return this->_h;
}

unsigned long
Vertex::id()
{
    return this->_id;
}

void
Vertex::setX(float x)
{
    this->_x = x;
}

void
Vertex::setY(float y)
{
    this->_y = y;
}

void
Vertex::setZ(float z)
{
    this->_z = z;
}

void
Vertex::setH(Halfedge * h)
{
    this->_h = h;
}

void
Vertex::setId(unsigned long id)
{
    this->_id = id;
}
