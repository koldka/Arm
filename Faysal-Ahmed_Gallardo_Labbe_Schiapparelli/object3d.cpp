#include "object3d.h"

#include <fstream>
#include <string>
#include <iostream>

Object3D::Object3D(char *filename) {
    //Read file
    std::ifstream file(filename);
    std::cout << "Read file:" << filename << std::endl;

    //First line == "PGM3D"
    std::string format;
    std::getline(file,format);
    std::cout << format << std::endl;
    if (format.compare("PGM3D") == 0)
    {
        std::cout << "Ok compare." << std::endl;
        //Get sizes
        file >> this->sizes[0] >> this->sizes[1] >> this->sizes[2];

        //Get max value
        file >> this->maxValue;

        long size = this->sizes[0] * this->sizes[1] * this->sizes[2];
        this->data = (uint8*)malloc(size* sizeof(uint8));

        long cpt(0);
        int val;
        while (file >> val)
        {
            if (cpt > size)
            {
                std::cout << "Too much values." << std::endl;
                break;
            }
            if (val > this->maxValue)
            {
                std::cout << "Current value is higher than expected." << std::endl;
                break;
            }

            data[cpt++] = (uint8)val;
        }
    }

}

void
Object3D::computeFaces()
{
    faces = (face *)malloc(this->sizes[0]*this->sizes[1]*this->sizes[2]*6*sizeof(face));
    //std::cout << this->sizes[0]*this->sizes[1]*this->sizes[2]*6*sizeof(face) << std::endl;

    long size = this->sizes[0]*this->sizes[1]*this->sizes[2];
    for (long i = 0; i < size; ++i) {
        int z = i / (this->sizes[0]*this->sizes[1]);
        int y = (i-(z*this->sizes[0]*this->sizes[1]))/this->sizes[0];
        int x = i-(z*this->sizes[0]*this->sizes[1])-(y*this->sizes[0]);

        //Right face
        if (x != this->sizes[0]-1)
        {
            if (this->data[i] != this->data[i+1])
            {
                face f = getFace(i,0);
                f.c = std::max(this->data[i],this->data[i+1]);
                faces[nbFaces] = f;
                nbFaces++;
            }
        }
        //Bottom face
        if (y != this->sizes[1]-1)
        {
            if (this->data[i] != this->data[i+this->sizes[0]])
            {
                face f = getFace(i,1);
                f.c = std::max(this->data[i],this->data[i+this->sizes[0]]);
                faces[nbFaces] = f;
                nbFaces++;
            }
        }
        //Rear face
        if (z != this->sizes[2]-1)
        {
            if (this->data[i] != this->data[i+this->sizes[0]*this->sizes[1]])
            {
                face f = getFace(i,2);
                f.c = std::max(this->data[i],this->data[i+this->sizes[0]*this->sizes[1]]);
                faces[nbFaces] = f;
                nbFaces++;
            }
        }
    }
    std::cout << nbFaces << std::endl;
}

Object3D::face
Object3D::getFace(long i, int f)
{
    int z = i / (this->sizes[0]*this->sizes[1]);
    int y = (i-(z*this->sizes[0]*this->sizes[1]))/this->sizes[0];
    int x = i-(z*this->sizes[0]*this->sizes[1])-(y*this->sizes[0]);

    x -= this->sizes[0] / 2;
    y -= this->sizes[1] / 2;
    z -= this->sizes[2] / 2;

    //0 : Right face
    //1 : Bottom face
    //2 : Rear face

    vertex v0, v1, v2, v3;
    v0.x = x + .5f;
    v0.y = y + .5f;
    v0.z = z + .5f;

    if (f == 0 || f == 1)
    {
        v1.x = x + .5f;
        v1.y = y + .5f;
        v1.z = z - .5f;
    }

    if (f == 0 || f == 2)
    {
        v3.x = x + .5f;
        v3.y = y - .5f;
        v3.z = z + .5f;
    }

    if (f == 0)
    {
        v2.x = x + .5f;
        v2.y = y - .5f;
        v2.z = z - .5f;
    }

    if (f == 1)
    {
        v2.x = x - .5f;
        v2.y = y + .5f;
        v2.z = z - .5f;

        v3.x = x - .5f;
        v3.y = y + .5f;
        v3.z = z + .5f;
    }

    if (f == 2)
    {
        v1.x = x - .5f;
        v1.y = y + .5f;
        v1.z = z + .5f;

        v2.x = x - .5f;
        v2.y = y - .5f;
        v2.z = z + .5f;
    }

    Object3D::face res;
    res.v[0] = v0;
    res.v[1] = v1;
    res.v[2] = v2;
    res.v[3] = v3;
    return res;
}

void
Object3D::getVoxel(long i) {
    int z = i / (this->sizes[0]*this->sizes[1]);
    int y = (i-(z*this->sizes[0]*this->sizes[1]))/this->sizes[0];
    int x = i-(z*this->sizes[0]*this->sizes[1])-(y*this->sizes[0]);

    std::cout << x << " ; " << y << " ; " << z << std::endl;

    vertex vertices[8];
    int cpt(0);
    //Create vertices
    for (int d_x = 0; d_x < 2; ++d_x) {
        for (int d_y = 0; d_y < 2; ++d_y) {
            for (int d_z = 0; d_z < 2; ++d_z) {
                vertex v;
                v.x = x + (d_x - .5f);
                v.y = y + (d_y - .5f);
                v.z = z + (d_z - .5f);
                vertices[cpt++] = v;
                std::cout << cpt-1 << ":" << v.x << " " << v.y << " " << v.z << '\n';
            }
        }
    }

    /*
    const char edges[12][2] = {
        {0, 1}, //0
      {0, 2}, //1
      {0, 4}, //2
      {5, 1}, //3
      {5, 4}, //4
      {5, 7}, //5
      {3, 1}, //6
      {3, 2}, //7
      {3, 7}, //8
      {6, 2}, //9
      {6, 4}, //10
      {6, 7} //11
    };
    */

    const char facesIndex[6][4] = {
        {0, 2, 6, 4},
        {4, 6, 7, 5},
        {5, 1, 0, 4},
        {5, 7, 3, 1},
        {1, 3, 2, 0},
        {6, 2, 3, 7},
    };

    face faces[6];

    for (size_t i = 0; i < 6; i++) {
        face f;
      for (size_t j = 0; j < 4; j++) {
        f.v[j] = vertices[facesIndex[i][j]];
      }
      faces[i] = f;
    }

}

Object3D::face *
Object3D::getFaces()
{
    return this->faces;
}

long
Object3D::getNbFaces()
{
    return this->nbFaces;
}
