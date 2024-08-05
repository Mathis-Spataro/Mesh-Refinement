#ifndef POINT_H
#define POINT_H

#include<iostream>
#include<cmath>

class Point
{
public:
//Constructors
    Point() : x(0.0), y(0.0), z(0.0) {}
    Point(const Point& other) : x(other.x), y(other.y), z(other.z) {} // Copy constructor
    Point(double coord_x, double coord_y, double coord_z) : x(coord_x), y(coord_y), z(coord_z) {}
    Point(double coord_x, double coord_y) : x(coord_x), y(coord_y), z(0.0) {} // 2D constructor
    Point(double scalar) : x(scalar), y(scalar), z(scalar) {}

//Getters
    double operator[](int i) const{
        return i == 2 ? z : i == 1 ? y : x;
    }

//Setters
    double &operator[](int i){
        return i == 2 ? z : i == 1 ? y : x;
    }

//Operators
    // Assignment
    Point& operator=(const Point& other) {
        if (this != &other) {
            this->x = other.x;
            this->y = other.y;
            this->z = other.z;
        }
        return *this;
    }

    // Comparison
    bool operator==(const Point& other) const{
        return this->x == other.x &&
               this->y == other.y &&
               this->z == other.z;
    }
    bool operator!=(const Point& other) const{
        return this->x != other.x ||
               this->y != other.y ||
               this->z != other.z;
    }

    double distanceTo(const Point& other) const{
        return sqrt(pow(other.x - x, 2) + pow(other.y - y, 2) + pow(other.z - z, 2));
    }

    // Arithmetic
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y, z + other.z);
    }
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y, z - other.z);
    }
    Point operator-() const {
        return Point(Point(0,0,0) -(*this));
    }
    Point operator*(double scalar) const {
        return Point(x * scalar, y * scalar, z * scalar);
    }
    Point operator/(double scalar) const {
        if (scalar == 0.0) {
            throw std::invalid_argument("Division by zero.");
        }
        return Point(x/scalar, y/scalar, z/scalar);
    }

    // Inplace arithmetic
    Point& operator+=(const Point& other) {
        (*this) = (*this) + other;
        return *this;
    }
    Point& operator-=(const Point& other) {
        (*this) = (*this) - other;
        return *this;
    }
    Point& operator*=(double scalar) {
        (*this) = (*this) * scalar;
        return *this;
    }
    Point& operator/=(double scalar) {
        (*this) = (*this) / scalar;
        return *this;
    }

    // Input and Output
    friend std::istream& operator>>(std::istream& in, Point& point) {
        in >> point.x >> point.y >> point.z;
        return in;
    }
    friend std::ostream& operator<<(std::ostream& out, const Point& point) {
        out <<point.x <<' ' <<point.y <<' ' <<point.z;
        return out;
    }

//Data
    double x;
    double y;
    double z;
};


#endif // POINT_H


