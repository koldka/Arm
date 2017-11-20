#include "object.h"

Object::Object(Halfedge ** edges, unsigned int nbEdges, unsigned int nbVertices, unsigned int nbFaces, int nbVertex)
{
    this->nbVertices = nbVertices;
    this->nbFaces = nbFaces;
    this->edges = edges;
    this->nbEdges = nbEdges;
    if(nbVertex < 4)
        this->vef = false;
    else
        this->vef = true;
}

Object::Object(Face ** faces, unsigned int nbFaces, int nbVertex)
{
    this->faces = faces;
    this->nbFaces = nbFaces;
    if(nbVertex < 4)
        this->vef = false;
    else
        this->vef = true;
}

Face **
Object::getFaces()
{
    return this->faces;
}

unsigned int
Object::getNbFaces()
{
    return this->nbFaces;
}

Vertex **
Object::getVertices()
{
    return this->vertices;
}

unsigned int
Object::getNbVertices()
{
    return this->nbVertices;
}

Halfedge **
Object::getHalfedges()
{
    return this->edges;
}

unsigned int
Object::getNbEdges()
{
    return this->nbEdges;
}

bool
Object::isVEF()
{
    return this->vef;
}

void
Object::setVEF(bool vef)
{
    this->vef = vef;
}
