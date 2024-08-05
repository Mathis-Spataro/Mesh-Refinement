#include "mesh.h"


// ----- public ----- //


// Constructors

Mesh::Mesh() : vertices(), faces(), _isDelaunay(false) {}

Mesh::Mesh(const std::vector<Vertex>& vs, const std::vector<Triangle>& fs)
    : vertices(vs), faces(fs), _isDelaunay(false) {}

Mesh::Mesh(const std::string& path) : Mesh(){
    if (path.substr(path.length() - 4) == ".off") {
        loadOFF(path);
    } else {
        loadPointSet(path);
    }
}


// Getters

const Triangle& Mesh::face(unsigned i) const {
    return faces[i];
}

const Vertex& Mesh::vertex(unsigned i) const {
    return vertices[i];
}

unsigned Mesh::getNumberOfVertices() const {
    return vertices.size();
}

unsigned Mesh::getNumberOfFaces() const {
    return faces.size();
}

bool Mesh::isDelaunay() const {
    return _isDelaunay;
}


array<Vertex, 3> Mesh::getFaceVertices(unsigned faceIndex) const {
    const Triangle& face = faces[faceIndex];
    array<Vertex, 3> faceVertices = {
        vertices[face.verticesIds[0]],
        vertices[face.verticesIds[1]],
        vertices[face.verticesIds[2]]
    };
    return faceVertices;
}

const array<unsigned, 3>& Mesh::getFaceVerticesIndices(unsigned faceIndex) const {
    return faces[faceIndex].verticesIds;
}

array<Triangle, 3> Mesh::getFaceNeighbors(unsigned faceIndex) const {
    const Triangle& face = faces[faceIndex];
    array<Triangle, 3> faceVertices = {
        faces[face.adjacentTrianglesIds[0]],
        faces[face.adjacentTrianglesIds[1]],
        faces[face.adjacentTrianglesIds[2]]
    };
    return faceVertices;
}

const array<unsigned, 3>& Mesh::getFaceNeighborsIndices(unsigned faceIndex) const {
    return faces[faceIndex].adjacentTrianglesIds;
}

Point Mesh::getFaceCenter(unsigned triangleIndex){
    return (vertices[faces[triangleIndex].verticesIds[0]].position() +
            vertices[faces[triangleIndex].verticesIds[1]].position() +
            vertices[faces[triangleIndex].verticesIds[2]].position()) / 3.0;
}

// Setters

/*note : not handling p exactly on an edge (considered inside triangle)*/
void Mesh::vertexAdd(const Point& p){
    _isDelaunay = false;
    unsigned closestTriangleIndex = findClosestTriangleIndex(p);
    array<unsigned, 2> closestVerticesId = getClosestVertices(closestTriangleIndex, p);

    if(closestVerticesId[0] == UINT_MAX){
        // p inside the triangle
        splitTriangle(closestTriangleIndex, p);
    } else {
        // p outside the triangle
        unsigned newVertexIndex = addVertex(p),
                 newTriangleIndex;
        double sta = signedTriangleArea(vertices[closestVerticesId[0]], vertices[closestVerticesId[1]], vertices[newVertexIndex]);
        if(sta > 0){ // orientation topologicaly correct
            newTriangleIndex = addTriangle(Triangle({closestVerticesId[0], closestVerticesId[1], newVertexIndex}));
        } else {     // reorder for topology.
            newTriangleIndex = addTriangle(Triangle({closestVerticesId[1], closestVerticesId[0], newVertexIndex}));
        }

        // set up topology
        unsigned oppositeVertexLocalIndex = faces[closestTriangleIndex].getOppositeVertexLocalIndex(
                                                                        closestVerticesId[0], closestVerticesId[1]
                                                                        );
        faces[closestTriangleIndex].adjacentTrianglesIds[oppositeVertexLocalIndex] = newTriangleIndex;
        faces[newTriangleIndex].adjacentTrianglesIds[2] = closestTriangleIndex;

        vertices[newVertexIndex].triangleIndex = newTriangleIndex;

        // keep convex Enveloppe
        updateConvexEnveloppe(newVertexIndex, newTriangleIndex);

    }
}


// Iterators

auto Mesh::facesBegin() {
    return faces.begin();
}

auto Mesh::facesEnd() {
    return faces.end();
}

auto Mesh::verticesBegin() {
    return vertices.begin();
}

auto Mesh::verticesEnd() {
    return vertices.end();
}


// Utilities

// enforce delaunay using lawson's algorithm
void Mesh::applyLawson(unsigned vertexIndex) {
    queue<array<unsigned, 2>> edges;
    if (vertexIndex != UINT_MAX)  getNonDelaunayEdgesAroundVertex(edges, vertexIndex);
    else getAllNonDelaunayEdges(edges);
    array<unsigned, 2> currentEdge;
    while(!edges.empty()){
        currentEdge = edges.front();

        // fliping some edges can make others delaunay or non-delaunay
        if(violatesDelaunay(currentEdge)){
            flipEdge(currentEdge);
            // add neighboring edges for inspection
            array<array<unsigned, 2>, 4> impactedEdges = getEdgeNeighboringEdges(currentEdge);
            for(int i=0; i<4; i++) edges.push(impactedEdges[i]);
        }
        edges.pop();
    }
    _isDelaunay = true;
    computeLaplacians();
}

void Mesh::applyRuppert(){
    for(unsigned i=0; i < faces.size(); i++){
        unsigned VertexIndexOppositeToLongestEdge = UINT_MAX;
        VertexIndexOppositeToLongestEdge = violatesRuppert(i);
        if (VertexIndexOppositeToLongestEdge != UINT_MAX) {
            unsigned newVertex = splitEdge(i, VertexIndexOppositeToLongestEdge);
            applyLawson(newVertex);
        }
    }
    computeLaplacians();
}

// ----- protected ----- //


// Getters

array<unsigned, 2> Mesh::getClosestVertices(unsigned closestTriangleIndex, const Point& p) {
    array<unsigned, 2> closestEdge = findFacingEdge(closestTriangleIndex, p);
    if(closestEdge[0] != UINT_MAX){
        if(p.distanceTo(vertices[closestEdge[0]]) > p.distanceTo(vertices[closestEdge[1]])){
            //reorder for index 0 to be the closest
            closestEdge = {closestEdge[1], closestEdge[0]};
        }
    }
    return closestEdge;
}

/* Returns edges defined as 2 triangles indices */
array<array<unsigned, 2>, 4> Mesh::getEdgeNeighboringEdges(array<unsigned, 2> edge) const {
     unsigned indexT2inT1 = faces[edge[0]].getTriangleLocalIndex(edge[1]);
     unsigned indexT1inT2 = faces[edge[1]].getTriangleLocalIndex(edge[0]);
     array<unsigned, 2> edge1 = { edge[0], faces[edge[0]].getAdjacentTriangleIndex(indexT2inT1 + 1) };
     array<unsigned, 2> edge2 = { edge[0], faces[edge[0]].getAdjacentTriangleIndex(indexT2inT1 - 1) };
     array<unsigned, 2> edge3 = { edge[1], faces[edge[1]].getAdjacentTriangleIndex(indexT1inT2 + 1) };
     array<unsigned, 2> edge4 = { edge[1], faces[edge[1]].getAdjacentTriangleIndex(indexT1inT2 - 1) };
     return {edge1, edge2, edge3, edge4};
}


// Setters

unsigned Mesh::addTriangle(const Triangle& triangle){
    faces.push_back(triangle);
    return faces.size()-1;
}

unsigned Mesh::addVertex(const Point& vertexPosition){
    vertices.push_back(Vertex(vertexPosition));
    return vertices.size()-1;
}


// Utilities

    // Triangulation Utilities

void Mesh::buildTopology() {
    // <paire id sommet formant l'arrete, paire (face incident à l'arrete, id sommet opposé)>
    std::map<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> topology;
    for(vector<Triangle>::iterator it = facesBegin(); it != facesEnd(); ++it){
        Triangle &f = *it;
        Vertex &v0 = vertices[f.verticesIds[0]];
        Vertex &v1 = vertices[f.verticesIds[1]];
        Vertex &v2 = vertices[f.verticesIds[2]];
        v0.triangleIndex = v1.triangleIndex = v2.triangleIndex = distance(facesBegin(), it);
        for(int j = 0; j < 3; ++j){
            int jNext = (j +1) % 3;
            int op = (j +2) % 3;
            auto edge = std::make_pair(f.verticesIds[j], f.verticesIds[jNext]);
            auto location_it = topology.find(edge);
            if(location_it == topology.end()){
                edge = std::make_pair(f.verticesIds[jNext], f.verticesIds[j]);
                location_it = topology.find(edge);
            }
            if( location_it == topology.end()){
                topology.insert(
                    std::make_pair(
                        edge,
                        std::make_pair(distance(facesBegin(), it), op)
                    )
                );
                vertices[f.verticesIds[j]].triangleIndex = distance(facesBegin(), it);
            }else{
                // ajoute la face adjacente à f à l'indice du sommet opposée à l'arete
                f.adjacentTrianglesIds[op] = location_it->second.first;
                faces[location_it->second.first].adjacentTrianglesIds[location_it->second.second] = distance(facesBegin(), it);
            }
        }
    }
    computeLaplacians();
}

void Mesh::computeLaplacians(){
    laplacians.resize(vertices.size());
    maxLaplacianIndex = 0;
    double maxLaplacianNorm = 0.0;
    for (unsigned i=0; i<laplacians.size(); i++){
        computeVertexLaplacian(i);
        if (laplacians[i].norm() > maxLaplacianNorm) {
            maxLaplacianIndex = i;
            maxLaplacianNorm = laplacians[maxLaplacianIndex].norm();
        }
    }
}

void Mesh::computeVertexLaplacian(unsigned vertexIndex){
    bool looped = false;
    VerticesCirculator vc(this, vertexIndex);
    unsigned lastVertexIndex;
    laplacians[vertexIndex] = Vector(0.0, 0.0, 0.0);
    while (vc != vc.end()) {
        lastVertexIndex = vc.id(); ++vc;
        laplacians[vertexIndex] += computeLaplacianComponent(vertices[lastVertexIndex],
                                                             vertices[vertexIndex],
                                                             (*vc));
        if (lastVertexIndex == vc.begin().id()){ // edge of mesh case handling
            looped = true;
            break;
        }
    } // end while
    if (!looped){ // edge of mesh case handling
        vc = vc.begin();
        while (vc != vc.end()) {
            lastVertexIndex = vc.id(); --vc;
            laplacians[vertexIndex] += computeLaplacianComponent(vertices[lastVertexIndex],
                                                                 vertices[vertexIndex],
                                                                 (*vc));
        }
    }
}

Vector Mesh::computeLaplacianComponent(const Point& p1, const Point& p2, const Point& p3){
    // Laplacian component based on cotangent and edge vectors
    double angle = angleRadians(p1, p2, p3);
    double cotangent = 1.0 / tan(angle);
    Vector vector1 = p1 - p2;
    Vector vector2 = p3 - p2;
    return (vector1 + vector2) * cotangent;
}

void Mesh::updateConvexEnveloppe(unsigned newVertexIndex, unsigned newTriangleIndex){
    VerticesCirculator vc;
    unsigned currentCenterVertexIndex, currentTriangleIndex;
    double area;

    // --- update left side of convex enveloppe when looking from the added vertex. --- //
    // (new Vertex local index always = 2)
    currentCenterVertexIndex = faces[newTriangleIndex].verticesIds[1];
    currentTriangleIndex = newTriangleIndex;
    area = 1;
    while(area > 0){ // > 0 means a triangle was added in the previous iterations = need to check convexity again.
        // find next vertex on the edge.
        vc = getVerticesCirculator(currentCenterVertexIndex);
        while(vc != vc.end()) ++vc;
        vc.goToLastVertex();

        // check convexity
        area = signedTriangleArea(vertices[newVertexIndex], vertices[vc.centerVertexIndex], *vc);
        if (area > 0){  // on the left side trigo order = concave = add triangle.
            // Triangle :
            array<unsigned, 3> verticesIndices = {newVertexIndex, vc.centerVertexIndex, vc.id()};
            array<unsigned, 3> adjTrianglesIndices = {vc.lastFaceIndex, UINT_MAX, currentTriangleIndex};
            unsigned addedTriangleIndex = addTriangle(Triangle(verticesIndices, adjTrianglesIndices));

            // topology :
            Triangle& lastface = faces[vc.lastFaceIndex];
            unsigned oppositeVertexLocalIndex = (lastface.getVertexLocalIndex(vc.id()) + 2) % 3;
            lastface.adjacentTrianglesIds[oppositeVertexLocalIndex] = addedTriangleIndex;

            Triangle& currentface = faces[currentTriangleIndex];
            oppositeVertexLocalIndex = (currentface.getVertexLocalIndex(vc.centerVertexIndex) + 2) % 3;
            currentface.adjacentTrianglesIds[oppositeVertexLocalIndex] = addedTriangleIndex;

            // update variables for next iteration.
            currentTriangleIndex = addedTriangleIndex;
            currentCenterVertexIndex = vc.id();
        }
    }
    // --- end left side --- //

    // --- update right side of convex enveloppe when looking from the added vertex. --- //
    // (new Vertex local index always = 2)
    currentCenterVertexIndex = faces[newTriangleIndex].verticesIds[0];
    currentTriangleIndex = newTriangleIndex;
    area = -1;
    while(area < 0){// < 0 means a triangle was added in the previous iterations = need to check convexity again.
        // find next vertex on the edge.
        vc = getVerticesCirculator(currentCenterVertexIndex);
        while(vc != vc.end()) --vc;
        vc.goToLastVertex();

        // check convexity
        area = signedTriangleArea(vertices[newVertexIndex], vertices[vc.centerVertexIndex], *vc);
        if (area < 0){  // on the right side clockwise order = concave = add triangle.
            // triangle :
            array<unsigned, 3> verticesIndices = {newVertexIndex, vc.id(), vc.centerVertexIndex};
            array<unsigned, 3> adjTrianglesIndices = {vc.lastFaceIndex, currentTriangleIndex, UINT_MAX};
            unsigned addedTriangleIndex = addTriangle(Triangle(verticesIndices, adjTrianglesIndices));

            // topology :
            Triangle& lastface = faces[vc.lastFaceIndex];
            unsigned oppositeVertexLocalIndex = (lastface.getVertexLocalIndex(vc.id()) + 1) % 3;
            lastface.adjacentTrianglesIds[oppositeVertexLocalIndex] = addedTriangleIndex;

            Triangle& currentface = faces[currentTriangleIndex];
            oppositeVertexLocalIndex = (currentface.getVertexLocalIndex(vc.centerVertexIndex) + 1) % 3;
            currentface.adjacentTrianglesIds[oppositeVertexLocalIndex] = addedTriangleIndex;

            // update variables for next iteration.
            currentTriangleIndex = addedTriangleIndex;
            currentCenterVertexIndex = vc.id();
        }
    }
    // --- end right side --- //
}

unsigned Mesh::findClosestTriangleIndex(const Point& p) {
    unsigned currentTriangleIndex = UINT_MAX, nextTriangleIndex =  rand() % faces.size();

    while(nextTriangleIndex != UINT_MAX){
        currentTriangleIndex = nextTriangleIndex;
        array<unsigned, 2> edgeToCross = findFacingEdge(currentTriangleIndex, p);
        if(edgeToCross[0] == UINT_MAX){
            // point inside the triangle (no intersection found).
            nextTriangleIndex = UINT_MAX;
        } else {
            // outside : find index of the next triangle.
            for(unsigned short i=0; i<3; i++){
                unsigned index = faces[currentTriangleIndex].verticesIds[i];
                if(index != edgeToCross[0] && index != edgeToCross[1]){
                    nextTriangleIndex = faces[currentTriangleIndex].adjacentTrianglesIds[i];
                    break;
                }
            }
        } // if no triangle on the other side of the edge, next triangle = UINT_MAX (topology)
    }

    return currentTriangleIndex;
}

array<unsigned, 2> Mesh::findFacingEdge(unsigned triangleIndex, const Point& p){
    Point center = getFaceCenter(triangleIndex);

    // Check intersection between edges and center-to-point segment.
    for(unsigned short i = 0; i<3; i++) {
        unsigned v1Index = faces[triangleIndex].verticesIds[i];
        unsigned v2Index = faces[triangleIndex].verticesIds[(i+1)%3];
        double area1 = signedTriangleArea(vertices[v1Index], vertices[v2Index], p) *
                       signedTriangleArea(vertices[v1Index], vertices[v2Index], center);
        double area2 = signedTriangleArea(center, p, vertices[v1Index]) *
                       signedTriangleArea(center, p, vertices[v2Index]);

        if(area1 <= 0 && area2 <= 0){ //intersection
            return {v1Index, v2Index};
        }
    }
    return {UINT_MAX, UINT_MAX};    // p is in the triangle.
}

    // 2D meshes operations

/* Splits a triangle into 3, following topology
   Postcondition : the old triangle is one of the resulting triangles
   Postcondition : the triangle index of the added vertex is the old triangle */
void Mesh::splitTriangle(unsigned triangleIndex,const Point& newVertexPosition){
    // Adding vertex
    unsigned newVertexIndex = addVertex(newVertexPosition);
    vertices[newVertexIndex].triangleIndex = triangleIndex;

    // Creating two copies of the surrounding triangle.
    unsigned newTriangleIndex1 = addTriangle(Triangle(faces[triangleIndex]));
    unsigned newTriangleIndex2 = addTriangle(Triangle(faces[triangleIndex]));

    // Updating neighbors
    array<unsigned, 3>& neighbors = faces[triangleIndex].adjacentTrianglesIds;
    if (neighbors[0] != UINT_MAX)
        faces[neighbors[0]].replaceAdjTriangleIndex(triangleIndex, newTriangleIndex1);
    if (neighbors[1] != UINT_MAX)
        faces[neighbors[1]].replaceAdjTriangleIndex(triangleIndex, newTriangleIndex2);

    // Adjusting the 3 triangles to perform spliting.
    faces[newTriangleIndex1].verticesIds[0] = newVertexIndex;
    faces[newTriangleIndex2].verticesIds[1] = newVertexIndex;
    faces[triangleIndex].verticesIds[2] = newVertexIndex;

    faces[newTriangleIndex1].adjacentTrianglesIds[1] = newTriangleIndex2;
    faces[newTriangleIndex1].adjacentTrianglesIds[2] = triangleIndex;

    faces[newTriangleIndex2].adjacentTrianglesIds[0] = newTriangleIndex1;
    faces[newTriangleIndex2].adjacentTrianglesIds[2] = triangleIndex;

    faces[triangleIndex].adjacentTrianglesIds[0] = newTriangleIndex1;
    faces[triangleIndex].adjacentTrianglesIds[1] = newTriangleIndex2;
}

/* splits an edge by inserting a point in its middle, following topology */
unsigned Mesh::splitEdge(unsigned triangleIndex, unsigned oppositeVertexIndex) {
    unsigned oppositeVertexLocalIndex = faces[triangleIndex].getVertexLocalIndex(oppositeVertexIndex);
    unsigned oppositeTriangleIndex = faces[triangleIndex].adjacentTrianglesIds[oppositeVertexLocalIndex];

    // Add a vertex in the middle of the edge
    unsigned edgeP1Index = faces[triangleIndex].verticesIds[(oppositeVertexLocalIndex + 1) % 3];
    unsigned edgeP2Index = faces[triangleIndex].verticesIds[(oppositeVertexLocalIndex + 2) % 3];
    Point edgeMiddle = (vertices[edgeP1Index] + vertices[edgeP2Index]) / 2;
    unsigned newVertexIndex = addVertex(edgeMiddle);
    vertices[newVertexIndex].triangleIndex = triangleIndex;

    // create a copy of the current triangle
    unsigned newTriangleIndex = addTriangle(Triangle(faces[triangleIndex]));

    // modify the two triangles to put them in their place
    Triangle& newTriangle = faces[newTriangleIndex];
    Triangle& currentTriangle = faces[triangleIndex];
    unsigned newVertexLocalIndexInCurrentTriangle = (oppositeVertexLocalIndex + 1) % 3;
    unsigned newVertexLocalIndexInNewTriangle = (oppositeVertexLocalIndex + 2) % 3;
    currentTriangle.verticesIds[newVertexLocalIndexInCurrentTriangle] = newVertexIndex;
    newTriangle.verticesIds[newVertexLocalIndexInNewTriangle] = newVertexIndex;

    // link the resulting triangles together
    currentTriangle.adjacentTrianglesIds[(oppositeVertexLocalIndex + 2) % 3] = newTriangleIndex;
    newTriangle.adjacentTrianglesIds[(oppositeVertexLocalIndex + 1) % 3] = triangleIndex;

    // adjust neighbor next to new triangle
    unsigned indexOfNeighborOfNewTriangleToUpdate
            = faces[newTriangleIndex].adjacentTrianglesIds[newVertexLocalIndexInNewTriangle];
    faces[indexOfNeighborOfNewTriangleToUpdate].replaceAdjTriangleIndex(triangleIndex, newTriangleIndex);

    // handle when there is a triangle on the other side of the edge
    if (oppositeTriangleIndex != UINT_MAX) {
        // repeat on the other side to keep topology
        // copy
        Triangle& currentOppositeTriangle = faces[oppositeTriangleIndex];
        unsigned newOppositeTriangleIndex = addTriangle(Triangle(currentOppositeTriangle));
        Triangle& newOppositeTriangle = faces[newOppositeTriangleIndex];
        unsigned otherOppositeVertexLocalIndex = currentOppositeTriangle.getOppositeVertexLocalIndex(edgeP1Index, edgeP2Index);
        currentOppositeTriangle.verticesIds[(otherOppositeVertexLocalIndex + 2) % 3] = newVertexIndex;
        newOppositeTriangle.verticesIds[(otherOppositeVertexLocalIndex + 1) % 3] = newVertexIndex;

        // link
        currentOppositeTriangle.adjacentTrianglesIds[(otherOppositeVertexLocalIndex + 1) % 3] = newOppositeTriangleIndex;
        newOppositeTriangle.adjacentTrianglesIds[(otherOppositeVertexLocalIndex + 2) % 3] = oppositeTriangleIndex;
        newTriangle.adjacentTrianglesIds[oppositeVertexLocalIndex] = newOppositeTriangleIndex;
        newOppositeTriangle.adjacentTrianglesIds[otherOppositeVertexLocalIndex] = newTriangleIndex;

        // adjust neighbor next to new triangle
        indexOfNeighborOfNewTriangleToUpdate
                = faces[newOppositeTriangleIndex].adjacentTrianglesIds[newVertexLocalIndexInNewTriangle];
        faces[indexOfNeighborOfNewTriangleToUpdate].replaceAdjTriangleIndex(oppositeTriangleIndex, newOppositeTriangleIndex);

    }
    return newVertexIndex;
}

void Mesh::flipEdge(array<unsigned, 2> edge){
    flipEdge(edge[0], edge[1]);
}

void Mesh::flipEdge(unsigned T1Index, unsigned T2Index){
    // local indices of each triangle in the other.
    // these indices are those of the vertex opposite to the edge in each triangle due to topology
    unsigned T2IndexInT1 = faces[T1Index].getTriangleLocalIndex(T2Index);
    unsigned T1IndexInT2 = faces[T2Index].getTriangleLocalIndex(T1Index);

    // vertices modification
    faces[T1Index].verticesIds[(T2IndexInT1 + 1) % 3] = faces[T2Index].verticesIds[T1IndexInT2];
    faces[T2Index].verticesIds[(T1IndexInT2 + 1) % 3] = faces[T1Index].verticesIds[T2IndexInT1];

    // adjacent faces update of their neighbors.
    unsigned T1neighborToUpdate = faces[T1Index].getAdjacentTriangleIndex((T2IndexInT1 + 2) % 3);
    if (T1neighborToUpdate != UINT_MAX){
        faces[T1neighborToUpdate].replaceAdjTriangleIndex(T1Index, T2Index);
    }
    unsigned T2neighborToUpdate = faces[T2Index].getAdjacentTriangleIndex((T1IndexInT2 + 2) % 3);
    if (T2neighborToUpdate != UINT_MAX){
        faces[T2neighborToUpdate].replaceAdjTriangleIndex(T2Index, T1Index);
    }

    // adjacent faces update in T1 and T2
    unsigned tmp = faces[T1Index].adjacentTrianglesIds[T2IndexInT1];
    faces[T1Index].adjacentTrianglesIds[T2IndexInT1] = faces[T2Index].adjacentTrianglesIds[(T1IndexInT2 + 2) % 3];
    faces[T2Index].adjacentTrianglesIds[(T1IndexInT2 + 2) % 3] = faces[T2Index].adjacentTrianglesIds[T1IndexInT2];
    faces[T2Index].adjacentTrianglesIds[T1IndexInT2] = faces[T1Index].adjacentTrianglesIds[(T2IndexInT1 + 2) % 3];
    faces[T1Index].adjacentTrianglesIds[(T2IndexInT1 + 2) % 3] = tmp;
}

    // Delaunay utilities

bool Mesh::violatesDelaunay(array<unsigned, 2> edge){
    return violatesDelaunay(edge[0], edge[1]);
}

/* check whether an edge defined by two triangles is locally delaunay */
bool Mesh::violatesDelaunay(unsigned triangleIndex1, unsigned triangleIndex2){
    if (triangleIndex1 == UINT_MAX || triangleIndex2 == UINT_MAX ||
            faces[triangleIndex1].getTriangleLocalIndex(triangleIndex2) == UINT_MAX)
        //edge is on the border or triangles are not adjacent.
        return false;
    unsigned indexT2inT1 = faces[triangleIndex1].getTriangleLocalIndex(triangleIndex2);
    unsigned indexT1inT2 = faces[triangleIndex2].getTriangleLocalIndex(triangleIndex1);

    unsigned t1oppositeVertexIndex = faces[triangleIndex2].verticesIds[indexT1inT2];
    unsigned t2oppositeVertexIndex = faces[triangleIndex1].verticesIds[indexT2inT1];

    return insideCircumcirle(faces[triangleIndex1], vertices[t1oppositeVertexIndex]) ||
           insideCircumcirle(faces[triangleIndex2], vertices[t2oppositeVertexIndex]);
}

unsigned Mesh::violatesRuppert(unsigned triangleIndex){
    Point& p1 = vertices[faces[triangleIndex].verticesIds[0]];
    Point& p2 = vertices[faces[triangleIndex].verticesIds[1]];
    Point& p3 = vertices[faces[triangleIndex].verticesIds[2]];
    double edge12Length = Vector(p2 - p1).norm();
    double edge23Length = Vector(p3 - p2).norm();
    double edge13Length = Vector(p1 - p3).norm();

    double longestLength = max(edge12Length, max(edge23Length, edge13Length));
    double shortestLength = min(edge12Length, min(edge23Length, edge13Length));

    if ((longestLength / shortestLength) > 4){
        if(longestLength == edge12Length){
            return faces[triangleIndex].verticesIds[2];
        } else if (longestLength == edge13Length) {
            return faces[triangleIndex].verticesIds[1];
        } else {
            return faces[triangleIndex].verticesIds[0];
        }
    }
    return UINT_MAX;
}

/* checks whether point p is inside or on the circumcircle of the three points from triangle t */
bool Mesh::insideCircumcirle(const Triangle& t, const Point& p){
    Point& p1 = vertices[t.verticesIds[0]];
    Point& p2 = vertices[t.verticesIds[1]];
    Point& p3 = vertices[t.verticesIds[2]];
    double ax = p1.x - p.x;
    double ay = p1.y - p.y;
    double bx = p2.x - p.x;
    double by = p2.y - p.y;
    double cx = p3.x - p.x;
    double cy = p3.y - p.y;

    return ((ax * ax + ay * ay) * (bx * cy - cx * by) -
            (bx * bx + by * by) * (ax * cy - cx * ay) +
            (cx * cx + cy * cy) * (ax * by - bx * ay)) >= 0;
}

/* /!\ complexity O(n squared) */
void Mesh::getAllNonDelaunayEdges(queue<array<unsigned, 2>>& nonDelaunayEdges){
    for(unsigned i=0; i<faces.size(); i++){
        for(unsigned j=i+1; j<faces.size(); j++){
            if(violatesDelaunay(i, j)){
                nonDelaunayEdges.push({i, j});
            }
        }
    }
}

void Mesh::getNonDelaunayEdgesAroundVertex(queue<array<unsigned, 2>>& nonDelaunayEdges, unsigned vertexIndex){
    FacesCirculator fc = getFacesCirculator(vertexIndex);
    bool looped = false;
    unsigned face, oppositeFace;
    while(fc != fc.end()){
        face = fc.id();
        oppositeFace = (*fc).adjacentTrianglesIds[(*fc).getVertexLocalIndex(vertexIndex)];
        if(oppositeFace != UINT_MAX && violatesDelaunay(face, oppositeFace)) {
            nonDelaunayEdges.push({face, oppositeFace});
        }
        ++fc;
        if(fc != fc.end() && violatesDelaunay(face, fc.id())){
            nonDelaunayEdges.push({face, fc.id()});
        }
        if(fc == fc.begin()) {
            looped = true;
            break;
        }
    }
    if (!looped){
        fc = fc.begin();
        // handling first iteration separately to avoid duplicate of edge with opposite face.
        face = fc.id();
        --fc;
        if(fc != fc.end() && violatesDelaunay(face, fc.id())){
            nonDelaunayEdges.push({face, fc.id()});
        }
        while(fc != fc.end()){
            face = fc.id();
            oppositeFace = (*fc).adjacentTrianglesIds[(*fc).getVertexLocalIndex(vertexIndex)];
            if(oppositeFace != UINT_MAX && violatesDelaunay(face, oppositeFace)) {
                nonDelaunayEdges.push({face, oppositeFace});
            }
            --fc;
            if(fc != fc.end() && violatesDelaunay(face, fc.id())){
                nonDelaunayEdges.push({face, fc.id()});
            }
        }
    }
}
