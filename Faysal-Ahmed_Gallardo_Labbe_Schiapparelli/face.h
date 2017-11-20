#ifndef FACE_H
#define FACE_H

#include "vertex.h"
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

class Vertex;
class Halfedge;

class Face
{
public:
    Face(int nbVertex, uint8_t c);

    Vertex * getVertex(int i);
    uint8_t getColor();
    Halfedge * getH();
    Vertex * getNormal(int i);
    int getNbVertex();

    void setVertex(int i, Vertex * v);
    void setColor(uint8_t c);
    void setColorIf(uint8_t c);
    void setH(Halfedge * h);
    void setNormal(int i, Vertex * n);

private:
    uint8_t color;
    Halfedge * h;
    int nbVertex;
    std::vector<Vertex*> vertices;
    std::vector<Vertex*> normals;
};

#endif // FACE_H
