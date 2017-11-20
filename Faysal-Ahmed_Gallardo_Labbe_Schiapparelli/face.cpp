#include "face.h"

Face::Face(int nbVertex, uint8_t c)
{
    this->vertices.resize(nbVertex);
    this->normals.resize(nbVertex);
    this->nbVertex = nbVertex;
    this->color = c;
}

Vertex *
Face::getVertex(int i)
{
    return this->vertices.at(i);
}

int
Face::getNbVertex()
{
    return this->nbVertex;
}

uint8_t
Face::getColor()
{
    return this->color;
}

Halfedge *
Face::getH()
{
    return this->h;
}

Vertex *
Face::getNormal(int i)
{
    return this->normals.at(i);
}

void
Face::setVertex(int i, Vertex * v)
{
    this->vertices.at(i) = v;
}

void
Face::setColor(uint8_t c)
{
    this->color = c;
}

void
Face::setColorIf(uint8_t c)
{
    if (c > this->color)
        this->color = c;
}

void
Face::setH(Halfedge * h)
{
    this->h = h;
}

void
Face::setNormal(int i, Vertex * n)
{
    this->normals.at(i) = n;
}
