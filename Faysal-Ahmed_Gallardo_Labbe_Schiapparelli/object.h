#ifndef OBJECT_H
#define OBJECT_H

#include "vertex.h"
#include "face.h"

class Vertex;
class Face;
class Halfedge;

class Object
{
public:
    Object(Halfedge ** edges, unsigned int nbEdges, unsigned int nbVertices, unsigned int nbFaces, int nbVertex);
    Object(Face ** faces, unsigned int nbFaces, int nbVertex);

    Face ** getFaces();
    unsigned int getNbFaces();
    Vertex ** getVertices();
    unsigned int getNbVertices();
    Halfedge ** getHalfedges();
    unsigned int getNbEdges();
    bool isVEF();
    void setVEF(bool vef);

private:
    Vertex ** vertices;
    Face ** faces;
    unsigned int nbFaces;
    unsigned int nbVertices;
    Halfedge ** edges;
    unsigned int nbEdges;
    bool vef;
};

#endif // OBJECT_H
