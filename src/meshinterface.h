#ifndef MESHINTERFACE_H
#define MESHINTERFACE_H

#include "geometryComponents/vertex.h"
#include "geometryComponents/vector.h"
#include <array>
#include <vector>

class MeshConstInterface {
public:
    virtual unsigned getNumberOfVertices() const = 0;
    virtual unsigned getNumberOfFaces() const = 0;
    virtual std::array<Vertex, 3> getFaceVertices(unsigned faceIndex) const = 0;
};

#endif // MESHINTERFACE_H
