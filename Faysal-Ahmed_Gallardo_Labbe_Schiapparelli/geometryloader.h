#ifndef ARM_PROJECT_GEOMETRYLOADER_H
#define ARM_PROJECT_GEOMETRYLOADER_H

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QString>
#include <CGAL/IO/Geomview_stream.h>
#include <CGAL/IO/Polyhedron_geomview_ostream.h>

#include "ImportOBJ.h"
#include "object.h"
#include "filemanager.h"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;

class GeometryLoader : protected QOpenGLFunctions {

public:
    GeometryLoader(QString path);
    ~GeometryLoader();

    void drawGeometry(QOpenGLShaderProgram *program, float alpha);
    void saveGeometry(QString path);
    void setEdge();

private:
    void initGeometryPGM(int maxValue);
    void initGeometryOBJ();

    QOpenGLBuffer arrayBuf;
    QOpenGLBuffer indexBuf;
    Object * obj;
    Polyhedron P;
    int nf = 0;
    int maxValue = 3;
    bool edge = false;
};


#endif //ARM_PROJECT_GEOMETRYLOADER_H
