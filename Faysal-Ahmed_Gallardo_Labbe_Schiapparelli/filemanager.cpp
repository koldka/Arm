#include "filemanager.h"
#include "halfedge.h"
#include "vertex.h"
#include "face.h"
#include "ImportOBJ.h"
#include <map>
#include <fstream>
#include <iostream>
#include <QFileDialog>
#include <CGAL/Exact_spherical_kernel_3.h>
#include <CGAL/Random.h>
#include <CGAL/IO/print_wavefront.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <boost/function_output_iterator.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/foreach.hpp>
#include <CGAL/Surface_mesh.h>

class Vertex;
class Face;
class Halfedge;

typedef CGAL::Exact_spherical_kernel_3         Spherical_k;
typedef CGAL::Point_3<Spherical_k>             Point_3;
typedef CGAL::Sphere_3<Spherical_k>            Sphere_3;

std::pair<unsigned int, unsigned int>
getCoVoxel(unsigned int i, unsigned int j, int sizes[3])
{
    int z = i / (sizes[0]*sizes[1]);
    int y = (i-(z*sizes[0]*sizes[1]))/sizes[0];
    int x = i-(z*sizes[0]*sizes[1])-(y*sizes[0]);
    unsigned int k,l;
    switch (j) {
    case 0: //Front
        if (z == 0)
            l = 6;
        else
            l = 2;
        k = i - sizes[0]*sizes[1];
        break;
    case 1: //Right
        if (x == sizes[0]-1)
            l = 6;
        else
            l = 3;
        k = i + 1;
        break;
    case 2: //Back
        if (z == sizes[2]-1)
            l = 6;
        else
            l = 0;
        k = i + sizes[0]*sizes[1];
        break;
    case 3: //Left
        if (x == 0)
            l = 6;
        else
            l = 1;
        k = i - 1;
        break;
    case 4: //Bottom
        if (y == sizes[1]-1)
            l = 6;
        else
            l = 5;
        k = i + sizes[0];
        break;
    case 5: //Up
        if (y == 0)
            l = 6;
        else
            l = 4;
        k = i - sizes[0];

        break;
    default:
        l = 6;
        break;
    }
    return std::pair<unsigned int, unsigned int>(k,l);
}


Face *
createFace(unsigned int i, unsigned int j, int sizes[3], std::map<std::pair<unsigned int, unsigned int>,unsigned int> * faces, std::vector<Face *> * facesId, unsigned int * cpt)
{
    std::pair<unsigned int, unsigned int> pos(i,j);
    if (faces->find(pos) == faces->end())
    {
        int z = i / (sizes[0]*sizes[1]);
        int y = (i-(z*sizes[0]*sizes[1]))/sizes[0];
        int x = i-(z*sizes[0]*sizes[1])-(y*sizes[0]);

        //Start remove - do it in the drawing part
        x -= sizes[0] / 2;
        y -= sizes[1] / 2;
        z -= sizes[2] / 2;
        //End remove

        Vertex * v0;
        Vertex * v1;
        Vertex * v2;
        Vertex * v3;

        switch (j) {
        case 0: //Front
            v0 = new Vertex(x - .5f, y - .5f, z - .5f);
            v1 = new Vertex(x - .5f, y - .5f, z + .5f);
            v2 = new Vertex(x + .5f, y - .5f, z + .5f);
            v3 = new Vertex(x + .5f, y - .5f, z - .5f);
            break;
        case 1: //Right
            v0 = new Vertex(x + .5f, y - .5f, z - .5f);
            v1 = new Vertex(x + .5f, y + .5f, z - .5f);
            v2 = new Vertex(x + .5f, y + .5f, z + .5f);
            v3 = new Vertex(x + .5f, y - .5f, z + .5f);
            break;
        case 2: //Back
            v0 = new Vertex(x + .5f, y - .5f, z + .5f);
            v1 = new Vertex(x + .5f, y + .5f, z + .5f);
            v2 = new Vertex(x - .5f, y + .5f, z + .5f);
            v3 = new Vertex(x - .5f, y - .5f, z + .5f);
            break;
        case 3: //Left
            v0 = new Vertex(x - .5f, y - .5f, z + .5f);
            v1 = new Vertex(x - .5f, y + .5f, z + .5f);
            v2 = new Vertex(x - .5f, y + .5f, z - .5f);
            v3 = new Vertex(x - .5f, y - .5f, z - .5f);
            break;
        case 4: //Bottom
            v0 = new Vertex(x - .5f, y + .5f, z - .5f);
            v1 = new Vertex(x - .5f, y + .5f, z + .5f);
            v2 = new Vertex(x + .5f, y + .5f, z + .5f);
            v3 = new Vertex(x + .5f, y + .5f, z - .5f);
            break;
        case 5: //Up
            v0 = new Vertex(x - .5f, y - .5f, z - .5f);
            v1 = new Vertex(x + .5f, y - .5f, z - .5f);
            v2 = new Vertex(x + .5f, y - .5f, z + .5f);
            v3 = new Vertex(x - .5f, y - .5f, z + .5f);
            break;
        default:
            return nullptr;
            break;
        }

        v0->setId((*cpt)++);
        v1->setId((*cpt)++);
        v2->setId((*cpt)++);
        v3->setId((*cpt)++);

        Face * f = new Face(4,0);
        f->setVertex(0, v0);
        f->setVertex(1, v1);
        f->setVertex(2, v2);
        f->setVertex(3, v3);
        facesId->push_back(f);
        faces->insert(std::pair<std::pair<unsigned int, unsigned int>, unsigned int>(pos,facesId->size()-1));

        std::pair<unsigned int, unsigned int> kl = getCoVoxel(i,j,sizes);
        //Check that k is in the image
        if (kl.second != 6)
        {
            //Check couple k, l doesnt exist
            if (faces->find(kl) == faces->end())
            {
                //Add the face to the couple
                faces->insert(std::pair<std::pair<unsigned int, unsigned int>, unsigned int>(kl,facesId->size()-1));
            }
        }

        return f;
    }

    Face * f = facesId->at(faces->at(pos));
    return f;
}

Object *
FileManager::loadPGM3D(const char *path)
{
    //Read data
    std::ifstream file(path);
    std::string format;
    std::getline(file,format);

    assert (format.compare("PGM3D") == 0 && "Not a PGM3D.");

    int sizes[3];
    int maxValue;
    //Get sizes
    file >> sizes[0] >> sizes[1] >> sizes[2];

    //Get max value
    file >> maxValue;

    long size = sizes[0] * sizes[1] * sizes[2];
    uint8_t * data = (uint8_t*)malloc(size* sizeof(uint8_t));

    long cpt(0);
    int val;
    while (file >> val)
    {
        assert (cpt < size && "Too much values.");
        assert (val <= maxValue && "Current value is higher than expected.");

        data[cpt++] = (uint8_t)val;
    }

    std::map<std::pair<unsigned int, unsigned int>,unsigned int> facesMap;
    std::vector<Face *> facesId;
    unsigned int cptV(0);
    for (unsigned int i = 0; i < size; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            Face * f = createFace(i,j,sizes, &facesMap, &facesId, &cptV);
            //Checker if the two are different here
            //If identical then not change color
            std::pair<unsigned int, unsigned int> kl = getCoVoxel(i,j,sizes);
            if (kl.second != 6 && data[i] != data[kl.first])
            {
                uint8_t mC = data[i];
                if (data[kl.first] > mC)
                    mC = data[kl.first];
                f->setColor(mC);
            }
            else
            {
                f->setColor(0);
            }
        }
    }

    Face ** faces = (Face **)malloc(size*6*sizeof(Face*));
    for (unsigned int i = 0; i < facesId.size(); ++i)
    {
        faces[i] = facesId[i];
    }

    Object * obj = new Object(faces,facesId.size(), 3);
    obj->setVEF(true);

    return obj;
}

void fill(Polyhedron * P)
{
    // Incrementally fill the holes
    unsigned int nb_holes = 0;
    Halfedge_handle he = P->halfedges_begin();
    Halfedge_iterator he_b= he;
    do
    {
        if(he_b->is_border())
        {
            std::vector<Facet_handle>  patch_facets;
            std::vector<Vertex_handle> patch_vertices;
            CGAL::Polygon_mesh_processing::triangulate_hole(
                        *P,
                        he_b,
                        std::back_inserter(patch_facets),//std::back_inserter(patch_vertices),
                        CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, *P)).geom_traits(Kernel())
                        );
            std::cout << " Number of facets in constructed patch: " << patch_facets.size() << std::endl;
            std::cout << " Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
            ++nb_holes;
        }
        he_b++;
    }while(he_b !=   P->halfedges_end());
    std::cout << std::endl;
    std::cout << nb_holes << " holes have been filled" << std::endl;
}

void offset(Polyhedron *P, float distance, int maxValue)
{
    Halfedge* tab_edge = (Halfedge*) malloc(sizeof(Halfedge)*P->size_of_halfedges()); //taille des halfedges ?
    Vertex* tab_vertex = (Vertex*) malloc(sizeof(Vertex)*P->size_of_vertices());

    //sphere
    for (auto it = P->facets_begin(); it != P->facets_end(); ++it, ++it) {
        auto hfc = it->facet_begin();
        for (int j = 0; j < maxValue; ++j, ++hfc) {
            auto hfc = it->facet_begin();
            Point_3 p = Point_3(hfc->vertex()->point().x(),
                                hfc->vertex()->point().y(),
                                hfc->vertex()->point().z());
            Sphere_3 s1 = Sphere_3(p, distance);
            /* Analytique */
        }

    }
    //face
    for (auto it = P->halfedges_begin(); it != P->halfedges_end(); ++it) {
       Halfedge* h1 = P->Halfedge(*it, 0);




    }
}

/* Calcul approximation
Polyhedron approxi(Polyhedron *P, int epsilon )
{

}
*/
/*typedef boost::graph_traits<Polyhedron>::face_descriptor face_descriptor;

void offset(Polyhedron *P)
{
    Facet_handle ft = P->facets_begin();
    Facet_iterator ft_b= ft;
    do{
        std::vector<face_descriptor>  patch_facets;
        CGAL::Polygon_mesh_processing::connected_component(
                    ft_b,
                    *P,
                    std::back_inserter(patch_facets), //!!!!!!!!!!!
                    CGAL::Polygon_mesh_processing::parameters::all_default()
        );
        ft_b++;
    }while(ft_b != P->facets_end());
}*/


/*
void fill(Polyhedron *P)
{
    Halfedge_handle he = P->halfedges_begin();
    Halfedge_iterator he_b = he;
    size_t i_th_hole = 0;
    do{
        if (he_b->is_border()){
            i_th_hole++;

            CGAL::Vector_3<Kernel> vec(0.0,0.0,0.0);
            std::size_t order = 0;
            Halfedge_iterator h = he_b;
            do{
                vec=vec + (h->vertex()->point() - CGAL::ORIGIN);
                ++order;
                h=h->next();
            }while(h != he_b);

            CGAL_assertion(order >= 3);
            CGAL::Point_3<Kernel> center = CGAL::ORIGIN + (vec / static_cast<double>(order));
            P->fill_hole(he_b);
            Halfedge_handle new_center=P->create_center_vertex(he_b);
            new_center->vertex()->point() = center;
        }
        he_b++;
    }while(he_b != P->halfedges_end());
}
*/

double
stringToDouble(std::string s)
{
    double pre = 1;
    double res = 0;
    bool minus = false;
    bool inf = true;
    int cptInf = -1;
    int iC = 0;
    for(char& c : s) {
        iC = c - '0';
        if(iC < 0){
            if(iC == -3)
                minus = true;
            if(iC == -2) // If there are more than one digits before the decimal point
                inf = false;
        } else {
            res += iC * pre;
            pre /= 10;
            if(inf)
                cptInf++;
        }
    }
    if(minus)
        res = -res;
    if(cptInf > 0)
        res = res * (10 * cptInf);
    return res;
}


Polyhedron
FileManager::loadOBJ(const char *path, int *maxValue)
{
    std::ifstream f_in;
    std::ifstream* p_f_in;

    f_in.open(path);
    p_f_in = &f_in;
    if(!*p_f_in) {
        std::cerr << "error: cannot open file '" << path
                  << "' for reading." << std::endl;
        exit(1);
    }
    std::istream& ref_f_in = f_in;

    Polyhedron P;
    BuildCgalPolyhedronFromObj<HalfedgeDS> P_scanned (ref_f_in);

    P.delegate(P_scanned);
    f_in.close();
    offset(&P, 3, 2);
    return P;
}


void
FileManager::saveOBJ(const char *path, Polyhedron P)
{
    std::ofstream fout(path);
    CGAL::print_polyhedron_wavefront(fout, P);
}

#define CGAL_EIGEN3_DISABLE


