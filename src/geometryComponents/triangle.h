#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <climits>
#include <array>
#include <iostream>


class Triangle
{
public:
// Constructors
    Triangle() : verticesIds({UINT_MAX, UINT_MAX, UINT_MAX}), adjacentTrianglesIds(verticesIds) {}

    Triangle(const std::array<unsigned, 3>& vertices)
        : verticesIds(vertices), adjacentTrianglesIds({UINT_MAX, UINT_MAX, UINT_MAX}) {}

    Triangle(const std::array<unsigned, 3>& vertices, const std::array<unsigned, 3>& adjacentTriangles)
        : verticesIds(vertices), adjacentTrianglesIds(adjacentTriangles) {}

    Triangle(const Triangle& other): verticesIds(other.verticesIds), adjacentTrianglesIds(other.adjacentTrianglesIds){}


// Getters
    unsigned getVertexIndex(int localIndex) const {
        if (localIndex < 0) {
            localIndex = 3 - ((-localIndex) % 3);
        }
        return verticesIds[localIndex % 3];
    }

    unsigned getAdjacentTriangleIndex(int localIndex) const {
        if (localIndex < 0) {
            localIndex = 3 - ((-localIndex) % 3);
        }
        return adjacentTrianglesIds[localIndex % 3];
    }

    unsigned getVertexLocalIndex(unsigned vertexIndex) const {
        for(unsigned i=0; i < 3; i++){
            if (verticesIds[i] == vertexIndex) return i;
        }
        return UINT_MAX;
    }

    unsigned getTriangleLocalIndex(unsigned triangleIndex) const {
        for(unsigned i=0; i < 3; i++){
            if (adjacentTrianglesIds[i] == triangleIndex) return i;
        }
        return UINT_MAX;
    }

    unsigned getNeighborTriangleIndex(unsigned vertexIndex1, unsigned vertexIndex2) const {
        unsigned short i=0;
        while(verticesIds[i] == vertexIndex1 || verticesIds[i] == vertexIndex2) ++i;
        return adjacentTrianglesIds[i];
    }

    unsigned getOppositeVertexLocalIndex(unsigned vertexIndex1, unsigned vertexIndex2) const {
        for(int i=0; i<3; i++){
            if ((verticesIds[i] != vertexIndex1) && (verticesIds[i] != vertexIndex2)){
                return i;
            }
        }
        return UINT_MAX;
    }

    // Setters
    void replaceAdjTriangleIndex(unsigned oldIndex, unsigned newIndex){
        for(unsigned short i=0; i<3; i++){
            if (this->adjacentTrianglesIds[i] == oldIndex){
                this->adjacentTrianglesIds[i] = newIndex;
                break;
            }
        }
    }

// Operators
    // Input and Output
    friend std::istream& operator>>(std::istream& in, Triangle& triangle) {
        for (int i = 0; i < 3; i++) {
            in >> triangle.verticesIds[i];
        }
        return in;
    }
    friend std::ostream& operator<<(std::ostream& out, const Triangle& triangle) {
        out <<triangle.verticesIds[0] <<' ' <<triangle.verticesIds[1] <<' ' <<triangle.verticesIds[2];
        return out;
    }

// Data
    // vertex i facing triangle i
    std::array<unsigned, 3> verticesIds;
    std::array<unsigned, 3> adjacentTrianglesIds;
};
#endif // TRIANGLE_H
