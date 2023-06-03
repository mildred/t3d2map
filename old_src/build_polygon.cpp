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
typedef CGAL::Nef_polyhedron_3<Kernel>  Nef_polyhedron;
typedef CGAL::Polyhedron_3<Kernel>  Polyhedron;
typedef Nef_polyhedron::Plane_3  Plane3;
typedef Nef_polyhedron::Point_3  Point3;
typedef Nef_polyhedron::Vector_3 Vector3;

template < class Traits,
           class Items,
           class Mark>
CGAL::Geomview_stream&
operator<<( CGAL::Geomview_stream &gv,
            const CGAL::Nef_polyhedron_3<Traits,Items,Mark> &P) {
    CGAL::Polyhedron_3<Kernel> P2;
    P.convert_to_polyhedron(P2);
    return gv << P;
}

TEST_CASE("Try to build a concave Nef_polyhedron_3", "[.]")
{
    CGAL::Geomview_stream gv(CGAL::Bbox_3(-100, -100, -100, 600, 600, 600));
    gv.set_line_width(4);
    gv.set_bg_color(CGAL::Color(0, 200, 200));

    CGAL::Nef_nary_union_3<Nef_polyhedron> nary_union;

    qDebug() << "Top z=2";
    std::vector<Point3> v1 = {Point3(0,0,0), Point3(2,0,0), Point3(2,2,0), Point3(0,2,0)};
    Nef_polyhedron N1(v1.begin(), v1.end(), Vector3( 0,  0, -1), true);
    nary_union.add_polyhedron(N1);

    gv << N1;

    qDebug() << "Bottom z=0";
    std::vector<Point3> v2 = {Point3(0,0,2), Point3(1,0,2), Point3(1,2,2), Point3(0,2,2)};
    Nef_polyhedron N2(v2.begin(), v2.end(), Vector3( 0,  0,  1), true);
    nary_union.add_polyhedron(N2);

    gv << N2;

    qDebug() << "Front y=0";
    std::vector<Point3> v3 = {Point3(0,0,0), Point3(1,0,0), Point3(1,0,1), Point3(2,0,1), Point3(2,0,2), Point3(0,0,2)};
    Nef_polyhedron N3(v3.begin(), v3.end(), Vector3( 0, -1,  0), true);
    nary_union.add_polyhedron(N3);

    qDebug() << "Left x=0";
    std::vector<Point3> v4 = {Point3(0,0,0), Point3(0,2,0), Point3(0,2,2), Point3(0,0,2)};
    Nef_polyhedron N4(v4.begin(), v4.end(), Vector3(-1,  0,  0), true);
    nary_union.add_polyhedron(N4);

    qDebug() << "Right Up x=2";
    std::vector<Point3> v5 = {Point3(2,0,1), Point3(2,2,1), Point3(2,2,2), Point3(2,0,2)};
    Nef_polyhedron N5(v5.begin(), v5.end(), Vector3( 1,  0,  0), true);
    nary_union.add_polyhedron(N5);

    {
        qDebug() << "Middle z=1";
        std::vector<Point3> v = {Point3(1,0,1), Point3(1,2,1), Point3(2,2,1), Point3(2,0,1)};
        Nef_polyhedron N(v.begin(), v.end(), Vector3( 0,  0, -1), true);
        nary_union.add_polyhedron(N);
    }

    {
        qDebug() << "Right Down x=1";
        std::vector<Point3> v = {Point3(1,0,0), Point3(1,2,0), Point3(1,2,1), Point3(1,0,1)};
        Nef_polyhedron N(v.begin(), v.end(), Vector3( 1,  0,  0), true);
        nary_union.add_polyhedron(N);
    }

    qDebug() << "Back y=2";
    std::vector<Point3> v6 = {Point3(0,2,0), Point3(1,2,0), Point3(1,2,1), Point3(2,2,1), Point3(2,2,2), Point3(0,2,2)};
    Nef_polyhedron N6(v6.begin(), v6.end(), Vector3( 0,  1,  0), true);
    nary_union.add_polyhedron(N6);

    qDebug() << "Union to N";
    Nef_polyhedron N = nary_union.get_union();

    CHECK(N.is_valid());
    CHECK(N.number_of_volumes() == 1);
    CHECK(N.is_simple());

    qDebug() << "N:";
    std::cout << N << std::endl;

    qDebug() << "Convex Decomposition";
    CGAL::convex_decomposition_3(N);

    qDebug() << "N:";
    std::cout << N << std::endl;

    CHECK(N.is_valid());
    CHECK(N.number_of_volumes() >= 2);
    CHECK(N.is_simple());

    std::cout << "Enter a key to finish" << std::endl;
    char ch;
    std::cin >> ch;
}

TEST_CASE("Difference between two cubes", "[current]")
{
    std::stringstream cube1_off("OFF\n"
    "#  cube.off\n"
    "#  A cube\n"
    "\n"
    "8 6 12\n"
    " 1.0   0.0   1.0\n"
    " 0.0   1.0   1.0\n"
    "-1.0   0.0   1.0\n"
    " 0.0  -1.0   1.0\n"
    " 1.0   0.0  -1.0\n"
    " 0.0   1.0  -1.0\n"
    "-1.0   0.0  -1.0\n"
    " 0.0  -1.0  -1.0\n"
    " 4  0 1 2 3  \n"
    " 4  7 4 0 3  \n"
    " 4  4 5 1 0  \n"
    " 4  5 6 2 1  \n"
    " 4  3 2 6 7  \n"
    " 4  6 5 4 7\n");

    std::stringstream cube2_off("OFF\n"
    "#  cube.off\n"
    "#  A cube\n"
    "\n"
    "8 6 12\n"
    " 2.0   0.0   2.0\n"
    " 0.0   2.0  2.0\n"
    "-2.0   0.0   2.0\n"
    " 0.0  -2.0   2.0\n"
    " 2.0   0.0  -2.0\n"
    " 0.0   2.0  -2.0\n"
    "-2.0   0.0  -2.0\n"
    " 0.0  -2.0  -2.0\n"
    " 4  0 1 2 3  \n"
    " 4  7 4 0 3  \n"
    " 4  4 5 1 0  \n"
    " 4  5 6 2 1  \n"
    " 4  3 2 6 7  \n"
    " 4  6 5 4 7\n");

    Nef_polyhedron Cube1, Cube2;
    std::size_t discarded = CGAL::OFF_to_nef_3 (cube1_off, Cube1, true);

    CHECK(Cube1.is_valid());
    CHECK(Cube1.is_simple());
    CHECK(discarded == 0);

    CHECK(CGAL::OFF_to_nef_3 (cube2_off, Cube2, true) == 0);

    Nef_polyhedron HollowCube = Cube2.difference(Cube1);

    CHECK(Cube1.is_valid());
    CHECK(HollowCube.is_simple());

    Polyhedron P;
    HollowCube.convert_to_polyhedron(P);
    std::cout << P;
}
