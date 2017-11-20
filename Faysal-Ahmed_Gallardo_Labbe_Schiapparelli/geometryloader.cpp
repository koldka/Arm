#include "geometryloader.h"
#include "displaywidget.h"
#include "ImportOBJ.h"
#include <QVector2D>
#include <QVector3D>
#include <QDebug>
#include <QFileInfo>
#include <iostream>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <GL/glut.h>

typedef Kernel::Vector_3 Vector;
typedef Kernel::Point_3 Point;

struct VertexData
{
    QVector3D position;
    float color;
};

GeometryLoader::GeometryLoader(QString path)
        : indexBuf(QOpenGLBuffer::IndexBuffer)
{
    initializeOpenGLFunctions();

    arrayBuf.create();

    QFileInfo fi(path);
    int maxValue = 0;
    if (fi.completeSuffix() == "obj")
    {
       P = FileManager::loadOBJ(path.toStdString().c_str(), &maxValue);
       initGeometryOBJ();
    }
    else if (fi.completeSuffix() == "pgm3d")
    {
        obj = FileManager::loadPGM3D(path.toStdString().c_str());
        maxValue = 4;
        initGeometryPGM(maxValue);
    }
}

GeometryLoader::~GeometryLoader()
{
    arrayBuf.destroy();
}

void
GeometryLoader::saveGeometry(QString path)
{
    FileManager::saveOBJ(path.toStdString().c_str(), P);
}

void GeometryLoader::initGeometryPGM(int maxValue)
{
    Face ** faces = obj->getFaces();
    unsigned int nbFaces = obj->getNbFaces();

    VertexData * vertices = (VertexData*)malloc(nbFaces*maxValue *sizeof(VertexData));

    for (unsigned int i = 0; i < nbFaces; ++i)
    {
        Face * f = faces[i];
        int nbVertex = f->getNbVertex();
        QVector3D prec;
        for (int j = 0; j < maxValue; ++j)
        {
            if (f->getColor() != 0)
            {
                VertexData vd;
                QVector3D vec;
                if(j >= nbVertex){
                    vec = prec;
                } else {
                    Vertex * v = f->getVertex(j);
                    vec = QVector3D(v->x(),v->y(),v->z());
                    prec = vec;
                }
                vd.position = vec;
                vd.color = f->getColor();
                vertices[i*maxValue + j] = vd;
            }
        }
    }

    // Transfer vertex data to VBO 0
    arrayBuf.bind();

    arrayBuf.allocate(vertices, nbFaces * maxValue * sizeof(VertexData));
}

int searchVertex(Polyhedron::Vertex *haystack, int length, Point needle)
{
    for (int i = 0; i < length; ++i) {
        if (haystack[i].point() == needle)
            return i;
    }
    throw "Vertex not found";
}

Vector facet_normal(Polyhedron::Facet_handle facet)
{
    auto hfc = facet->facet_begin();
    CGAL_assertion( CGAL::circulator_size(hfc) >= 3 );

    Point v[3];
    for (int i = 0; i < 3; ++i, hfc++) {
        v[i] = hfc->vertex()->point();
    }
    auto n = cross_product( v[ 1 ] - v[ 2 ], v[ 1 ] - v[ 0 ] );
    return ( 1.0/sqrt( n.squared_length() ) ) * n;
}

void GeometryLoader::initGeometryOBJ(){
    nf = P.size_of_facets();

    for (auto it = P.facets_begin(); it != P.facets_end(); ++it) {
        auto hfc = it->facet_begin();
        if(CGAL::circulator_size(hfc) > maxValue){
            maxValue = circulator_size(hfc);
        }
    }
    VertexData * test = (VertexData*)malloc(maxValue*nf*sizeof(VertexData));

    int i = 0;
    for (auto it = P.facets_begin(); it != P.facets_end(); ++it, ++i) {
        auto hfc = it->facet_begin();
        for (int j = 0; j < maxValue; ++j, ++hfc) {
            if(j < CGAL::circulator_size(hfc)){
                test[i*maxValue + j].position = QVector3D(hfc->vertex()->point().x(), hfc->vertex()->point().y(), hfc->vertex()->point().z());
                test[i*maxValue + j].color = 255;
            } else {
                test[i*maxValue + j].position = test[i*maxValue + CGAL::circulator_size(hfc) - 1].position;
                test[i*maxValue + j].color = 255;
            }
        }
    }

   arrayBuf.bind();
   arrayBuf.allocate(test, nf * maxValue * sizeof(VertexData));
}

void GeometryLoader::setEdge(){
    edge = !(edge);
}

void GeometryLoader::drawGeometry(QOpenGLShaderProgram *program, float alpha)
{
    // Tell OpenGL which VBOs to use
    arrayBuf.bind();

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(VertexData));

    int vertexColor = program->attributeLocation("a_color");
    program->enableAttributeArray(vertexColor);
    program->setAttributeBuffer(vertexColor, GL_FLOAT, sizeof(QVector3D), 1, sizeof(VertexData));

    int vertexAlpha = program->attributeLocation("a_alpha");
    program->setAttributeValue(vertexAlpha, ((alpha * 1.f) / 100.f));

    if(this->edge)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Draw cube geometry using indices from VBO 1
    if (maxValue != 3){
        glDrawArrays(GL_QUADS, 0, nf * maxValue * sizeof(VertexData));
    } else {
        glDrawArrays(GL_TRIANGLES, 0,  nf * maxValue * sizeof(VertexData));
    }
}
