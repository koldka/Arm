#ifndef FILEMANAGER_H
#define FILEMANAGER_H


#include "object.h"
#include "ImportOBJ.h"
#include <CGAL/IO/Geomview_stream.h>
#include <CGAL/IO/Polyhedron_geomview_ostream.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;

class FileManager
{
public:
    static Object * loadPGM3D(const char* path);
    static Polyhedron loadOBJ(const char* path, int *maxValue);
    static void saveOBJ(const char *path, Polyhedron P);
};


#endif // FILEMANAGER_H
