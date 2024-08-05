#include "geometricworld.h"
#include <GL/freeglut.h>


extern const string projectPath;

// ----- public ----- //
// Getters

GeometricWorld& GeometricWorld::getInstance() {
    static GeometricWorld instance;
    return instance;
}

bool GeometricWorld::meshIsDelaunay() const {
    return mesh.isDelaunay();
}

MeshConstInterface* GeometricWorld::getMeshConstInterface() {
    return &mesh;
}

// Setters

void GeometricWorld::meshAddVertex(const Point& p, bool delaunize){
    mesh.vertexAdd(p);
    if (delaunize) {
        mesh.applyLawson(mesh.getNumberOfVertices() -1);
    }
}


// Input - Output

void GeometricWorld::loadMesh(const string& path){
    mesh = Mesh(path);
}

void GeometricWorld::saveOFF(const string& path) const {
    mesh.saveOFF(path);
}

//void GeometricWorld::loadPointSet(const string& path){
//    mesh.loadPointSet(path);
//}

// Utilities

void GeometricWorld::enforceDelaunay(){
    mesh.applyLawson(UINT_MAX);
}

void GeometricWorld::enforceRuppert() {
    mesh.applyRuppert();
}

// ----- protected ----- //
// Constructors

GeometricWorld::GeometricWorld()
{
    mesh = Mesh(projectPath + "data/core2DMesh.off");
}



