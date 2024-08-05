#ifndef GEOMETRY_H
#define GEOMETRY_H

// this is an umbrella header for all the core geometrical components.
#include "geometryComponents/point.h"
#include "geometryComponents/vertex.h"
#include "geometryComponents/vector.h"
#include "geometryComponents/triangle.h"

inline double dotProduct(const Vector& u, const Vector& v) {
    return u.x * v.x +
           u.y * v.y +
           u.z * v.z;
}

// 3D vectors only
inline Vector crossProduct(const Vector& u, const Vector& v){
    return Vector(u.y * v.z - u.z * v.y,
                  u.z * v.x - u.x * v.z,
                  u.x * v.y - u.y * v.x);
}

// > 0 = trigo order, 0 = aligned, < 0 = clockwise order. 2D only
inline double signedTriangleArea(const Point& p1, const Point& p2, const Point& p3){
    Vector v1(p2 - p1);
    v1.z = 0;
    Vector v2(p3 - p2);
    v2.z = 0;
    return 0.5 * crossProduct(v1, v2).z;
}

// Function to check if a point is inside the circumcircle of a triangle. 2D only
inline bool pointInsideCircumcircle(const Point& point, const std::array<Point, 3>& triangleVertices) {
    double ax = triangleVertices[0].x - point.x;
    double ay = triangleVertices[0].y - point.y;
    double bx = triangleVertices[1].x - point.x;
    double by = triangleVertices[1].y - point.y;
    double cx = triangleVertices[2].x - point.x;
    double cy = triangleVertices[2].y - point.y;

    double det = ax * (by * cx - bx * cy) - ay * (bx * cx - by * cy) + (ax * by - ay * bx) * cx;
    return det > 0; // Return true if the point is inside the circumcircle
}

inline double angleRadians(const Point& p1, const Point& p2, const Point& p3){
    // Calculate vectors between points
    double dx1 = p1.x - p2.x;
    double dy1 = p1.y - p2.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;

    // Calculate dot product and magnitudes of vectors
    double dotProduct = dx1 * dx2 + dy1 * dy2;
    double magnitude1 = sqrt(dx1 * dx1 + dy1 * dy1);
    double magnitude2 = sqrt(dx2 * dx2 + dy2 * dy2);

    // Calculate cosine of the angle using dot product and magnitudes
    double cosineAngle = dotProduct / (magnitude1 * magnitude2);
    double angleInRadians = acos(cosineAngle);
    return angleInRadians;
}


inline double angleDegrees(const Point& p1, const Point& p2, const Point& p3){
    double angleInDegrees = angleRadians(p1, p2, p3) * 180.0 / M_PI;
    return angleInDegrees;
}


#endif // GEOMETRY_H
