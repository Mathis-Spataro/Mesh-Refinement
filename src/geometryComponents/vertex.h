#ifndef VERTEX_H
#define VERTEX_H

#include "point.h"
#include <climits>

class Vertex : public Point
{
public:
// constructors
    Vertex() : Point(), triangleIndex(UINT_MAX) {}
    Vertex(const Point& point) : Point(point), triangleIndex(UINT_MAX) {}
    Vertex(const Vertex& vertex) : Point(vertex), triangleIndex(vertex.triangleIndex) {}

    Vertex(double x, double y, double z) : Point(x, y, z), triangleIndex(UINT_MAX) {}
    Vertex(double x, double y) : Point(x, y), triangleIndex(UINT_MAX) {}
    Vertex(double n) : Point(n), triangleIndex(UINT_MAX) {}

    Vertex(const Point& point, const unsigned& triangleId) : Point(point), triangleIndex(triangleId) {}
    Vertex(double x, double y, double z, const unsigned& triangleId) : Point(x, y, z), triangleIndex(triangleId) {}
    Vertex(double x, double y, const unsigned& triangleId) : Point(x, y), triangleIndex(triangleId) {}
    Vertex(double n, const unsigned& triangleId) : Point(n), triangleIndex(triangleId) {}

//getters
    Point position() const {
        return Point(x, y, z);
    }

//operators
    // Assignment
    Vertex& operator=(const Vertex& other) {
        if (this != &other) {
            Point::operator=(other);
            this->triangleIndex = other.triangleIndex;
        }
        return *this;
    }

    // Comparison
    bool operator==(const Vertex& other) const {
        return static_cast<const Point&>(*this) == static_cast<const Point&>(other) &&
               triangleIndex == other.triangleIndex;
    }

    bool operator!=(const Vertex& other) const {
        return static_cast<const Point&>(*this) != static_cast<const Point&>(other) ||
               triangleIndex != other.triangleIndex;
    }

//data
    unsigned triangleIndex;         // index of one of the incident triangles in the mesh.

};

#endif // VERTEX_H
