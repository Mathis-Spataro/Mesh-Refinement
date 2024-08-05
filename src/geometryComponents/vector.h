#ifndef VECTOR_H
#define VECTOR_H

#include "point.h"

class Vector : public Point
{
public:
//Constructors
    Vector() : Point() {}
    Vector(const Point& coordinates) : Point(coordinates) {}
    Vector(double x, double y, double z) : Point(x, y, z) {}
    Vector(double x, double y) : Point(x, y) {}
    Vector(double n) : Point(n) {}

//Vector maths
    double norm() const {
        return sqrt(x * x + y * y + z * z);
    }

    Vector normalization() const {
        double magnitude = norm();
        if (magnitude == 0.0) {
            // Avoid division by zero, return a copy of the zero vector.
            return Vector(0.0, 0.0, 0.0);
        }
        return *this / magnitude;
    }
};
#endif // VECTOR_H
