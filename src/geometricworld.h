#ifndef GEOMETRICWORLD_H
#define GEOMETRICWORLD_H

#include "mesh.h"
#include "meshinterface.h"
#include <array>

class GeometricWorld {
public :
    GeometricWorld(const GeometricWorld&) = delete;
    GeometricWorld& operator=(const GeometricWorld&) = delete;

    // Getters
    static GeometricWorld& getInstance();
    bool meshIsDelaunay() const;
    MeshConstInterface* getMeshConstInterface();

    // Setters
    void meshAddVertex(const Point& p, bool delaunize = false);

    // Input - Output
    void loadMesh(const string& path);
    void saveOFF(const string& path) const;
//    void loadPointSet(const string& path);

    // Utilities
    void enforceDelaunay();
    void enforceRuppert();

protected :
    // Constructors
    GeometricWorld();

    // data
    Mesh mesh;
};


#endif // GEOMETRICWORLD_H
