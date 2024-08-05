#include "mesh.h"

void Mesh::loadOFF(const string &path) {
    cout<<"loading off : " + path <<endl;
    ifstream in(path, ios_base::in);
    if(in.is_open()){
        vertices.clear();
        faces.clear();

        // first line is "OFF\n", skipping it.
        string firstline;
        getline(in, firstline);

        int nbVertices, nbFaces, dummy;         // useless 0 at the end of first line.
        in >> nbVertices >> nbFaces >>dummy;
        vertices.resize(nbVertices);
        faces.resize(nbFaces);

        for(int i=0; i < nbVertices; i++){
            in >> vertices[i];
        }

        for(int i=0; i < nbFaces; i++){
            in >> dummy;        // skip the first value indicating a triangle
            in >> faces[i];
        }

        buildTopology();

    } else {
        cout<<"unable to read or find the file"<<endl;
    }
    in.close();
}

void Mesh::saveOFF(const string &path) const {
    cout<<"saving off : " + path <<endl;
    ofstream out(path);
    if(out.is_open()){
        out <<"OFF\n";
        out <<vertices.size() <<' ' <<faces.size() <<" 0\n";
        for(unsigned i=0; i<vertices.size(); i++){
            out <<vertices[i] <<'\n';
        }
        for(unsigned i=0; i<faces.size(); i++){
            out <<"3 " <<faces[i] <<'\n';
        }
        out.close();
    } else {
        cout<<"unable to open file"<<endl;
    }
}

void Mesh::loadPointSet(const string& path) {
    cout<<"loading point set : " + path <<endl;
    ifstream in(path, ios_base::in);
    if(in.is_open()){
        vertices.clear();
        faces.clear();

        int nbVertices;
        in >> nbVertices;
        vertices.resize(3);

        in >> vertices[0];
        in >> vertices[1];
        in >> vertices[2];
        // ensure trigo order of first triangle
        if (signedTriangleArea(vertices[0], vertices[1], vertices[2]) > 0){
            addTriangle( Triangle(std::array<unsigned, 3>({0, 1, 2}) ));
        } else {
            addTriangle( Triangle(std::array<unsigned, 3>({0, 2, 1}) ));
        }
        buildTopology();

        // add all other vertices
        for(int i=3; i < nbVertices; i++){
            Point p;
            in >> p;
            vertexAdd(p);
        }


    } else {
        cout<<"unable to read or find the file"<<endl;
    }
    in.close();
}

