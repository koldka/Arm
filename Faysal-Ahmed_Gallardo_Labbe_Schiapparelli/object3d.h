#ifndef ARM_PROJECT_OBJECT3D_H
#define ARM_PROJECT_OBJECT3D_H

typedef unsigned char uint8;


class Object3D {

public:
    Object3D(char* filename);

    struct vertex {
        float x;
        float y;
        float z;
    };

    struct edge {
        vertex v[2];
    };

    struct face {
        vertex v[4];
        uint8 c = 0;
    };

    struct voxel {
        face f[6];
    };

    void getVoxel(long i);
    void computeFaces();
    face getFace(long i, int f);
    face * getFaces();
    long getNbFaces();

private:
    int sizes[3];
    int maxValue;
    uint8* data;
    face * faces;
    long nbFaces = 0;

};


#endif //ARM_PROJECT_OBJECT3D_H
