#ifndef MESH_H
#define MESH_H

#include "geometry.h"
#include "meshinterface.h"
#include <vector>
#include <fstream>
#include <map>
#include <queue>

using namespace std;

class Mesh : public MeshConstInterface {
public:
// Constructors
    Mesh();
    Mesh(const std::vector<Vertex>& vs, const std::vector<Triangle>& fs);
    Mesh(const std::string& path);

// Getters
    const Triangle& face(unsigned i) const;
    const Vertex& vertex(unsigned i) const;
    unsigned getNumberOfVertices() const;
    unsigned getNumberOfFaces() const;
    const std::vector<Vector>& getLaplacians() const;
    bool isDelaunay() const;

    array<Vertex, 3> getFaceVertices(unsigned faceIndex) const;
    const array<unsigned, 3>& getFaceVerticesIndices(unsigned faceIndex) const;
    array<Triangle, 3> getFaceNeighbors(unsigned faceIndex) const;
    const array<unsigned, 3>& getFaceNeighborsIndices(unsigned faceIndex) const;
    Point getFaceCenter(unsigned faceIndex);

// Setters
    void vertexAdd(const Point& p);

// Input - Output : meshio.cpp
    void loadOFF(const string& path);
    void saveOFF(const string& path) const;
    void loadPointSet(const string& path);

// Iterators
    auto facesBegin();
    auto facesEnd();
    auto verticesBegin();
    auto verticesEnd();

// Circulators : meshiterators.cpp
    class FacesCirculator;
    inline FacesCirculator getFacesCirculator(unsigned centerVertexIndex);
    class VerticesCirculator;
    inline VerticesCirculator getVerticesCirculator(unsigned centerVertexIndex);

// Utilities
    void applyLawson(unsigned vertexIndex);
    void applyRuppert();


protected:
// Getters
    std::array<unsigned, 2> getClosestVertices(unsigned triangleIndex, const Point& p);
    array<array<unsigned, 2>, 4> getEdgeNeighboringEdges(array<unsigned, 2> edge) const;
        /* Returns edges defined as 2 triangles indices */

// Setters
    unsigned addTriangle(const Triangle& triangle);
    unsigned addVertex(const Point& vertexPosition);

// Utilities
    // Triangulation utilities
    void buildTopology();
    void computeLaplacians();
    void computeVertexLaplacian(unsigned vertexIndex);
    Vector computeLaplacianComponent(const Point& p1, const Point& p2, const Point& p3);
    void updateConvexEnveloppe(unsigned insertedPointIndex, unsigned newTriangleIndex);
    unsigned findClosestTriangleIndex(const Point& p);
    std::array<unsigned, 2> findFacingEdge(unsigned currentTriangleIndex, const Point& p);

    // 2D meshes operations
    void splitTriangle(unsigned triangleIndex,const Point& newVertexPosition);
    unsigned splitLongestEdge(unsigned triangleIndex);
    /* splits an edge by inserting a point in its middle */
    unsigned splitEdge(unsigned triangleIndex, unsigned oppositeVertexIndex);
    void flipEdge(array<unsigned, 2> edge);
    void flipEdge(unsigned firstTriangleIndex, unsigned secondTriangleIndex);

    // Delaunay utilities
    bool violatesDelaunay(array<unsigned, 2> edge);
    bool violatesDelaunay(unsigned triangleIndex1, unsigned triangleIndex2);
    unsigned violatesRuppert(unsigned triangleIndex);
    bool insideCircumcirle(const Triangle& t, const Point& p);
    void getAllNonDelaunayEdges(queue<array<unsigned, 2>>& nonDelaunayEdges);
    void getNonDelaunayEdgesAroundVertex(queue<array<unsigned, 2>>& nonDelaunayEdges, unsigned vertexIndex);

// Data
    std::vector<Vertex> vertices;
    std::vector<Triangle> faces;
    std::vector<Vector> laplacians;
    unsigned maxLaplacianIndex;
    unsigned minLaplacianIndex;
    bool _isDelaunay;
};




class Mesh::FacesCirculator {
    friend class Mesh;  // needed to allow TriangulationMesh to create this iterator
public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = Triangle;
    using difference_type = std::ptrdiff_t;
    using pointer = Triangle*;
    using reference = Triangle&;

// Constructors
    FacesCirculator();

// Getters
    unsigned id() const;
    Triangle& operator*();
    FacesCirculator begin();
    FacesCirculator end();

// Setters
    FacesCirculator& operator++();
    FacesCirculator operator++(int);
    FacesCirculator& operator--();
    FacesCirculator operator--(int);
    void goToLastFace();

// Operators
    bool operator!=(const FacesCirculator& other);
    bool operator==(const FacesCirculator& other);

protected:
// Constructors
    FacesCirculator(Mesh* tmesh,  unsigned centerVertexIdx);

// Utilities
    unsigned getNextFaceIndex();
    unsigned getPreviousFaceIndex();

// Data
    Mesh* mesh;
    unsigned centerVertexIndex, currentFaceIndex, lastFaceIndex;
};

Mesh::FacesCirculator Mesh::getFacesCirculator(unsigned centerVertexIndex){
    return FacesCirculator(this, centerVertexIndex);
}




class Mesh::VerticesCirculator : public FacesCirculator {
    friend class Mesh;  // needed to allow TriangulationMesh to create this iterator
public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = Vertex;
    using difference_type = std::ptrdiff_t;
    using pointer = Vertex*;
    using reference = Vertex&;

// Constructors
    VerticesCirculator();

// Getters
    unsigned id() const;
    Vertex operator*();
    VerticesCirculator begin();
    VerticesCirculator end();

// Setters
    VerticesCirculator& operator++();
    VerticesCirculator operator++(int);
    VerticesCirculator& operator--();
    VerticesCirculator operator--(int);

// Operators
    bool operator!=(const VerticesCirculator& other);
    bool operator==(const VerticesCirculator& other);

// Utilities
    void goToLastFace();
    void goToLastVertex();

protected:
// Constructors
    VerticesCirculator(Mesh* mesh, unsigned centerVertexIndex);

// Utilities
    void updateCurrentVertexIndex();
// Data
    unsigned currentVertexIndex;
    bool incrementLimit= false;
};

Mesh::VerticesCirculator Mesh::getVerticesCirculator(unsigned centerVertexIndex){
    return VerticesCirculator(this, centerVertexIndex);
}

#endif // MESH_H


















