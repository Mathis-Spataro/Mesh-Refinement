#include "mesh.h"


// =========================================== //
//              FacesCirculator                //
// =========================================== //

// Constructors

Mesh::FacesCirculator::FacesCirculator()
    : mesh(nullptr),
      centerVertexIndex(UINT_MAX),
      currentFaceIndex(UINT_MAX),
      lastFaceIndex(UINT_MAX)
{}


// Getters

unsigned Mesh::FacesCirculator::id() const {
    return currentFaceIndex;
}

Triangle& Mesh::FacesCirculator::operator*(){
    return mesh->faces[currentFaceIndex];
}

Mesh::FacesCirculator Mesh::FacesCirculator::begin(){
    return FacesCirculator(mesh, centerVertexIndex);
}

Mesh::FacesCirculator Mesh::FacesCirculator::end(){
        FacesCirculator tmp = FacesCirculator(mesh, centerVertexIndex);
        tmp.currentFaceIndex = UINT_MAX;
        return tmp;
    }


// Setters

Mesh::FacesCirculator& Mesh::FacesCirculator::operator++(){
    lastFaceIndex = this->id();
    currentFaceIndex = getNextFaceIndex();
    return *this;
}

Mesh::FacesCirculator Mesh::FacesCirculator::operator++(int) {
    FacesCirculator temp(*this);
    ++(*this);
    return temp;
}

Mesh::FacesCirculator& Mesh::FacesCirculator::operator--(){
    lastFaceIndex = this->id();
    currentFaceIndex = getPreviousFaceIndex();
    return *this;
}

Mesh::FacesCirculator Mesh::FacesCirculator::operator--(int) {
    FacesCirculator temp(*this);
    --(*this);
    return temp;
}

void Mesh::FacesCirculator::goToLastFace(){
    currentFaceIndex = lastFaceIndex;
}


// Operators

bool Mesh::FacesCirculator::operator!=(const FacesCirculator& other){
    return centerVertexIndex != other.centerVertexIndex ||
           currentFaceIndex != other.currentFaceIndex;
}

bool Mesh::FacesCirculator::operator==(const FacesCirculator& other){
    return centerVertexIndex == other.centerVertexIndex &&
           currentFaceIndex == other.currentFaceIndex;
}


// Constructors

Mesh::FacesCirculator::FacesCirculator(Mesh* tmesh,  unsigned centerVertexIdx)
    : mesh(tmesh),
      centerVertexIndex(centerVertexIdx),
      currentFaceIndex(mesh->vertices[centerVertexIndex].triangleIndex),
      lastFaceIndex(currentFaceIndex){}


// Utilities

unsigned Mesh::FacesCirculator::getNextFaceIndex(){
    /* topology ensures triangle i is opposite to vertex i in Triangle data storage */
    Triangle& currentFace = mesh->faces[currentFaceIndex];
    unsigned centerVertexLocalIndex = currentFace.getVertexLocalIndex(centerVertexIndex);
    return currentFace.adjacentTrianglesIds[(centerVertexLocalIndex +1) % 3];
}

unsigned Mesh::FacesCirculator::getPreviousFaceIndex(){
    /* topology ensures triangle i is opposite to vertex i in Triangle data storage */
    Triangle& currentFace = mesh->faces[currentFaceIndex];
    unsigned centerVertexLocalIndex = currentFace.getVertexLocalIndex(centerVertexIndex);
    return currentFace.adjacentTrianglesIds[(centerVertexLocalIndex+2) % 3];
}




// =========================================== //
//              VerticesCirculator             //
// =========================================== //


// Constructors

Mesh::VerticesCirculator::VerticesCirculator() : FacesCirculator(), currentVertexIndex(USHRT_MAX){}


// Getters

unsigned Mesh::VerticesCirculator::id() const {return currentVertexIndex;}

Vertex Mesh::VerticesCirculator::operator*(){return mesh->vertices[currentVertexIndex];}

Mesh::VerticesCirculator Mesh::VerticesCirculator::begin(){
        return VerticesCirculator(mesh, centerVertexIndex);
    }

Mesh::VerticesCirculator Mesh::VerticesCirculator::end(){
        VerticesCirculator tmp = VerticesCirculator(mesh, centerVertexIndex);
        tmp.currentVertexIndex = UINT_MAX;
        return tmp;
    }


// Setters

Mesh::VerticesCirculator& Mesh::VerticesCirculator::operator++(){
        if(currentFaceIndex == UINT_MAX){
            currentVertexIndex = UINT_MAX;
            incrementLimit = true;
        } else {
            FacesCirculator::operator++();
            if(currentFaceIndex == UINT_MAX){
                unsigned centerVertexLocalIndex = mesh->faces[lastFaceIndex].getVertexLocalIndex(centerVertexIndex);
                currentVertexIndex = mesh->faces[lastFaceIndex].verticesIds[(centerVertexLocalIndex + 2) % 3];
            } else {
                updateCurrentVertexIndex();
            }
        }
        return *this;
    }

Mesh::VerticesCirculator Mesh::VerticesCirculator::operator++(int){
        VerticesCirculator temp(*this);
        ++(*this);
        return temp;
    }

Mesh::VerticesCirculator& Mesh::VerticesCirculator::operator--(){
        FacesCirculator::operator--();
        updateCurrentVertexIndex();
        return *this;
    }

Mesh::VerticesCirculator Mesh::VerticesCirculator::operator--(int){
        VerticesCirculator temp(*this);
        --(*this);
        return temp;
    }

// Operators
    bool Mesh::VerticesCirculator::operator!=(const VerticesCirculator& other){
        return centerVertexIndex != other.centerVertexIndex ||
               currentVertexIndex != other.currentVertexIndex;
    }

    bool Mesh::VerticesCirculator::operator==(const VerticesCirculator& other){
        return centerVertexIndex == other.centerVertexIndex &&
               currentVertexIndex == other.currentVertexIndex;
    }

// Utilities
    void Mesh::VerticesCirculator::goToLastFace(){
        FacesCirculator::goToLastFace();
        updateCurrentVertexIndex();
    }

    void Mesh::VerticesCirculator::goToLastVertex(){
        if(incrementLimit){
            // face stays at uint max
            unsigned centerVertexLocalIndex = mesh->faces[lastFaceIndex].getVertexLocalIndex(centerVertexIndex);
            currentVertexIndex = mesh->faces[lastFaceIndex].verticesIds[(centerVertexLocalIndex + 2) % 3];
            incrementLimit = false;
        } else {
            currentFaceIndex = lastFaceIndex;
            updateCurrentVertexIndex();
        }
    }

// Constructors
    Mesh::VerticesCirculator::VerticesCirculator(Mesh* mesh, unsigned centerVertexIndex) : FacesCirculator(mesh, centerVertexIndex) {
        updateCurrentVertexIndex();
    }

// Utilities
    void Mesh::VerticesCirculator::updateCurrentVertexIndex(){
        if(currentFaceIndex == UINT_MAX){   // reached the edge of the mesh
            currentVertexIndex = UINT_MAX;
        } else {
            unsigned centerVertexLocalIndex = mesh->faces[currentFaceIndex].getVertexLocalIndex(centerVertexIndex);
            currentVertexIndex = mesh->faces[currentFaceIndex].verticesIds[(centerVertexLocalIndex + 1) % 3];
        }
    }


