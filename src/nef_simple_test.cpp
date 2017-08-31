#include <iostream>
#include <sstream>

#include <QDebug>
#include "catch.hpp"

#include <CGAL/Gmpz.h>
#include <CGAL/Extended_homogeneous.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Nef_nary_union_3.h>
#include <CGAL/convex_decomposition_3.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/OFF_to_nef_3.h>

#include <CGAL/IO/Geomview_stream.h>
#include <CGAL/IO/Polyhedron_geomview_ostream.h>

typedef CGAL::Homogeneous<CGAL::Gmpz>   Kernel;
typedef CGAL::Nef_polyhedron_3<Kernel>  Nef_polyhedron_3;
typedef Nef_polyhedron_3 Nef_polyhedron;
typedef CGAL::Polyhedron_3<Kernel>  Polyhedron_3;
typedef Nef_polyhedron_3::Plane_3  Plane_3;
typedef Nef_polyhedron_3::Point_3  Point_3;
typedef Nef_polyhedron_3::Vector_3 Vector_3;

CGAL::Geomview_stream& operator<< (CGAL::Geomview_stream &gv, const Nef_polyhedron_3 &P) {
    CHECK(P.is_simple());
    Polyhedron_3 P2;
    P.convert_to_polyhedron(P2);
    return gv << P2;
}

TEST_CASE("test simple Nef_polyhedron_3", "[.]")
{
    Nef_polyhedron N1(Plane_3( 1, 0, 0,-1));
    Nef_polyhedron N2(Plane_3(-1, 0, 0,-1));
    Nef_polyhedron N3(Plane_3( 0, 1, 0,-1));
    Nef_polyhedron N4(Plane_3( 0,-1, 0,-1));
    Nef_polyhedron N5(Plane_3( 0, 0, 1,-1));
    Nef_polyhedron N6(Plane_3( 0, 0,-1,-1));
    Nef_polyhedron I1(!N1 + !N2); // open slice in yz-plane
    Nef_polyhedron I2(N3 - !N4); // closed slice in xz-plane
    Nef_polyhedron I3(N5 ^ N6); // open slice in yz-plane
    Nef_polyhedron Cube1(I2 * !I1);
    Cube1 *= !I3;
    Nef_polyhedron Cube2 = N1 * N2 * N3 * N4 * N5 * N6;

    CHECK(Cube1.is_simple());
    CHECK(Cube2.is_simple());

    Polyhedron_3 P2;
    Cube1.convert_to_polyhedron(P2);

    std::cout << P2 << std::endl;

    Polyhedron_3 P3;
    Cube2.convert_to_polyhedron(P3);

    std::cout << P3 << std::endl;
}

TEST_CASE("test simple Nef_polyhedron_3 on geomview", "[.]")
{
    CGAL::Geomview_stream gv(CGAL::Bbox_3(-100, -100, -100, 600, 600, 600));
    gv.set_line_width(4);
    gv.set_bg_color(CGAL::Color(0, 200, 200));

    Nef_polyhedron N1(Plane_3( 1, 0, 0,-1));
    Nef_polyhedron N2(Plane_3(-1, 0, 0,-1));
    Nef_polyhedron N3(Plane_3( 0, 1, 0,-1));
    Nef_polyhedron N4(Plane_3( 0,-1, 0,-1));
    Nef_polyhedron N5(Plane_3( 0, 0, 1,-1));
    Nef_polyhedron N6(Plane_3( 0, 0,-1,-1));
    Nef_polyhedron I1(!N1 + !N2); // open slice in yz-plane
    Nef_polyhedron I2(N3 - !N4); // closed slice in xz-plane
    Nef_polyhedron I3(N5 ^ N6); // open slice in yz-plane
    Nef_polyhedron Cube1(I2 * !I1);
    Cube1 *= !I3;
    Nef_polyhedron Cube2 = N1 * N2 * N3 * N4 * N5 * N6;

    gv << Cube2;

    Polyhedron_3 P2;
    Cube2.convert_to_polyhedron(P2);

    std::cout << "Enter a key to finish" << std::endl;
    char ch;
    std::cin >> ch;
}
