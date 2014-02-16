#include <QDebug>
#include "catch.hpp"

#include <CGAL/Gmpz.h>
#include <CGAL/Extended_homogeneous.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Nef_nary_union_3.h>

typedef CGAL::Homogeneous<CGAL::Gmpz>   Kernel;
typedef CGAL::Nef_polyhedron_3<Kernel>  Nef_polyhedron;
typedef Nef_polyhedron::Plane_3  Plane3;
typedef Nef_polyhedron::Point_3  Point3;
typedef Nef_polyhedron::Vector_3 Vector3;

TEST_CASE("", "[current]")
{
    CGAL::Nef_nary_union_3<Nef_polyhedron> nary_union;

    // Top
    std::vector<Point3> v1 = {Point3(0,0,0), Point3(1,0,0), Point3(1,1,0), Point3(0,1,0)};
    Nef_polyhedron N1(v1.begin(), v1.end(), Vector3( 0,  0, -1), true);
    nary_union.add_polyhedron(N1);

    // Bottom
    std::vector<Point3> v2 = {Point3(0,0,1), Point3(1,0,1), Point3(1,1,1), Point3(0,1,1)};
    Nef_polyhedron N2(v2.begin(), v2.end(), Vector3( 0,  0,  1), true);
    nary_union.add_polyhedron(N2);

    // Front
    std::vector<Point3> v3 = {Point3(0,0,0), Point3(1,0,0), Point3(1,0,1), Point3(0,0,1)};
    Nef_polyhedron N3(v3.begin(), v3.end(), Vector3(-1,  0,  0), true);
    nary_union.add_polyhedron(N3);

    // Left
    std::vector<Point3> v4 = {Point3(0,0,0), Point3(0,1,0), Point3(0,1,1), Point3(0,0,1)};
    Nef_polyhedron N4(v4.begin(), v4.end(), Vector3( 0, -1,  0), true);
    nary_union.add_polyhedron(N4);

    // Back
    std::vector<Point3> v5 = {Point3(1,0,0), Point3(1,1,0), Point3(1,1,1), Point3(1,0,1)};
    Nef_polyhedron N5(v5.begin(), v5.end(), Vector3( 1,  0,  0), true);
    nary_union.add_polyhedron(N5);

    // Right
    std::vector<Point3> v6 = {Point3(0,1,0), Point3(1,1,0), Point3(1,1,1), Point3(0,1,1)};
    Nef_polyhedron N6(v6.begin(), v6.end(), Vector3( 0,  1,  0), true);
    nary_union.add_polyhedron(N6);



    Nef_polyhedron N = nary_union.get_union();
}
