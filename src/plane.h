#ifndef PLANE_H
#define PLANE_H

#include "vector.h"

class Plane
{
public:
    Plane(double3 origin, double3 normal);
    Plane() {}
    double3 origin;
    double3 normal;
};

#endif // PLANE_H
