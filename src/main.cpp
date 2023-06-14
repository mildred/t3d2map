#include <fstream>
#include <iostream>
#include <regex>
#include <format>
#include <map>
#include <cmath>
#include <algorithm>

#include <boost/algorithm/string.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/property_map.h>
#include <CGAL/convex_decomposition_3.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/boost/graph/IO/OBJ.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/Nef_2/debug.h>
#ifdef CONFIG_VIEWER
#include <CGAL/draw_surface_mesh.h>
#endif

#include "backtrace.hpp"

// using Kernel = Simple_cartesian<double>
// using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
// using Kernel = CGAL::Exact_predicates_exact_constructions_with_sqrt_kernel;
using K = Kernel;

using Nef = CGAL::Nef_polyhedron_3<Kernel>;

typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;
typedef Mesh::Edge_index edge_descriptor;

typedef enum {
  CSG_None,
  CSG_Subtract,
  CSG_Add
} CsgOper; 

using namespace std;

bool parse_line(ifstream &f, std::string &line, bool trim_left = true) {
  if (!std::getline(f, line)) return false;
  boost::trim_right(line);
  if (trim_left) boost::trim_left(line);
  return true;
}

std::regex reg_keyvals("^\\s*(Begin\\s+\\S+\\s+)?((\\S+)=(\\S+)\\s*)*$");
std::regex reg_polygon_vertex("^Vertex\\s+([^,]+),([^,]+),([^,]+)$");
std::regex reg_polygon_vector("^(Vertex|Origin|TextureU|TextureV)\\s+([^,]+),([^,]+),([^,]+)$");
std::regex reg_polygon_pan("^Pan\\s+U=(\\S+)\\s+V=(\\S+)$");
std::regex reg_polygon_attr("^(\\S+)\\s+([^,]+),([^,]+),([^,]+)$");
std::regex reg_keyval("^([^=]+)=(.*)$");
std::regex reg_location("^Location=\\((.*)\\)$");
std::regex reg_xyz_vector_var("([XYZ])=([^,\\)]+)");

struct CsgNode {
  size_t index;
  CsgOper oper;

  CsgNode(size_t index, CsgOper oper) : index(index), oper(oper) {}
};

struct Transform {
  double tx, ty, tz;

  void transform_import(Mesh &mesh) {
    using FT       = typename K::FT;
    using Point_3  = typename K::Point_3;
    using Vector_3 = typename K::Vector_3;

    using Affine_transformation_3 = CGAL::Aff_transformation_3<K>;
    using namespace CGAL::Polygon_mesh_processing;
    using namespace CGAL;

    /* FOR each vertex of each polygon of parsed brush DO:
     *   do MainScale ... x *= MainScale[x], y *= MainScale[y], z *= MainScale[z]
     *   do translation (-PrePivot[x], -PrePivot[y], -PrePivot[z])
     *   do rotation Yaw, Pitch, Roll
     *   do PostScale ... x *= PostScale[x], y *= PostScale[y], z *= PostScale[z]
     *   do TempScale ... x *= TempScale[x], y *= TempScale[y], z *= TempScale[z]
     *   do translation (Location[x], Location[y], Location[z])
     * ENDFOR
     */

    // translate
    auto translation = Affine_transformation_3(Translation(), Vector_3(FT(tx), FT(ty), FT(tz)));
    transform(translation, mesh);
  }
};

struct UVMap {
  std::string texture;
  K::Point_3 origin;
  K::Vector_3 u;
  K::Vector_3 v;
  double upan, vpan;

  UVMap() : upan(0), vpan(0) {
  }
};

using UVPropertyMap = Mesh::Property_map<face_descriptor,UVMap>; 

void parse_xyz_vector(std::string vector, double &x, double &y, double &z) {
  std::smatch m;

  x = 0.0;
  y = 0.0;
  z = 0.0;
  while(regex_search(vector, m, reg_xyz_vector_var)) {
    //cerr << "vector found " << m[0] << " in " << vector << endl;
    if (m[1] == "X") x = stod(m[2]);
    else if (m[1] == "Y") y = stod(m[2]);
    else if (m[1] == "Z") z = stod(m[2]);
    vector = m.suffix();
  }
}

struct Map {

  std::vector<Mesh> meshes;
  std::list<CsgNode> csg_tree;
  std::list<Mesh> worldspan;

  double xmin, xmax, ymin, ymax, zmin, zmax;

  bool debug_meshes;

  Map() {
    xmin = 0.0;
    xmax = 0.0;
    ymin = 0.0;
    ymax = 0.0;
    zmin = 0.0;
    zmax = 0.0;
    debug_meshes = false;
  }

  void compute_bounds(double x, double y, double z) {
    if (x < xmin) xmin = x;
    if (y < ymin) ymin = y;
    if (z < zmin) zmin = z;
    if (x > xmax) xmax = x;
    if (y > ymax) ymax = y;
    if (z > zmax) zmax = z;
  }

  bool parse_keyval(std::string line, std::map<std::string,std::string> res) {
    std::smatch m;
    if (!regex_match(line, m, reg_keyvals)) return false;
    for(int i = 2; i+2 < m.size(); i+=3) {
      res[m[i+1]] = m[i+2];
      cerr << "keyval " << m[i+1] << " : " << m[i+2] << endl;
    }
    return true;
  }

  bool parse_polygon(ifstream &f, std::string polygon_line, Mesh &mesh, UVPropertyMap &uvmap) {
    std::map<std::string, std::string> keyval;
    UVMap texture_map;
    std::list<vertex_descriptor> vertices;
    std::smatch m;
    std::string line;

    if (!parse_keyval(polygon_line, keyval)) {
      cerr << "Failed to parse polygon line key-values: " << polygon_line << endl;
      return false;
    }

    texture_map.texture = keyval["Texture"];

    while (parse_line(f, line)) {
      if (line.starts_with("End Polygon")) {
        const face_descriptor f = mesh.add_face(vertices);
        // TODO: flip normal if needed
        uvmap[f] = texture_map;
        return true;
      } else if (regex_match(line, m, reg_polygon_vertex)) {
        double x = stod(m[1]);
        double y = stod(m[2]);
        double z = stod(m[3]);
        compute_bounds(x, y, z);
        vertices.push_back(mesh.add_vertex(K::Point_3(x, y, z)));
      } else if (regex_match(line, m, reg_polygon_vector)) {
        std::string kind = m[1];
        double x = stod(m[2]);
        double y = stod(m[3]);
        double z = stod(m[4]);
        if (kind == "Vertex") {
          compute_bounds(x, y, z);
          vertices.push_back(mesh.add_vertex(K::Point_3(x, y, z)));
        } else if (kind == "Origin") {
          texture_map.origin = K::Point_3(x, y, z);
        } else if (kind == "TextureU") {
          texture_map.u = K::Vector_3(x, y, z);
        } else if (kind == "TextureV") {
          texture_map.v = K::Vector_3(x, y, z);
        }
      } else if (regex_match(line, m, reg_polygon_pan)) {
        texture_map.upan = stod(m[1]);
        texture_map.vpan = stod(m[2]);
      } else if (regex_match(line, m, reg_polygon_attr)) {
        // cerr << "Polygon line " << m[1] << ": " << m[2] << m[3] << m[4] << endl;
      } else {
        cerr << "Unexpected Polygon line: " << line << endl;
        return false;
      }
    }
    return false;
  }

  bool parse_polylist(ifstream &f, std::string actor_line, Mesh &mesh, UVPropertyMap &uvmap) {
    std::string line;
    while (parse_line(f, line)) {
      if (line.starts_with("End PolyList")) {
        return true;
      } else if (line.starts_with("Begin Polygon")) {
        if (!parse_polygon(f, line, mesh, uvmap)) return false;
      } else {
        cerr << "PolyList line: " << line << endl;
      }
    }
    return false;
  }

  bool parse_brush(ifstream &f, std::string actor_line, CsgOper csg_oper, Transform t) {
    using namespace CGAL::Polygon_mesh_processing;

    Mesh mesh;
    std::string line;

    bool created;
    UVPropertyMap uvmap;

    // https://doc.cgal.org/latest/Surface_mesh/index.html#title7
    boost::tie(uvmap, created) = mesh.add_property_map<face_descriptor,UVMap>("f:uv", UVMap());

    while (parse_line(f, line)) {
      if (line.starts_with("End Brush")) {
        stitch_borders(mesh);
        // Transform mesh
        t.transform_import(mesh);
        // Add mesh
        size_t index = meshes.size();
        meshes.push_back(mesh);
        csg_tree.push_back(CsgNode(index, csg_oper));
        return true;
      } else if (line.starts_with("Begin PolyList")) {
        if (!parse_polylist(f, line, mesh, uvmap)) return false;
      } else {
        cerr << "Brush line: " << line << endl;
      }
    }
    return false;
  }

  bool parse_actor(ifstream &f, std::string actor_line) {
    Transform t;
    std::string line;
    std::smatch m;
    CsgOper csg_oper = CSG_None;
    while (parse_line(f, line)) {
      if (line.starts_with("End Actor")) {
        return true;
      } else if (line.starts_with("Begin Brush")) {
        if (!parse_brush(f, line, csg_oper, t)) return false;
      } else if (regex_match(line, m, reg_location)) {
        parse_xyz_vector(m[1], t.tx, t.ty, t.tz);
      } else if (regex_match(line, m, reg_keyval)) {
        std::string key = m[1];
        std::string val = m[2];
        cerr << "Actor keyval: " << key << " = " << val << endl;
        if (key == "CsgOper") {
          if (val == "CSG_Subtract") csg_oper = CSG_Subtract;
          else if (val == "CSG_Add") csg_oper = CSG_Add;
        }
      } else {
        cerr << "Actor line: " << line << endl;
      }
    }
    return false;
  }

  bool parse_map(ifstream &f) {
    std::string line;
    if (!parse_line(f, line)) return false;
    if (line != "Begin Map") {
      cerr << "While parsing map, unexpected start line: " << line << endl;
      return false;
    }

    while (parse_line(f, line)) {
      if (line.starts_with("End Map")) {
        return true;
      } else if (line.starts_with("Begin Actor")) {
        if (!parse_actor(f, line)) return false;
      } else {
        cerr << "Unexpected line: " << line << endl;
        return false;
      }
    }
    return false;
  }

  bool get_bounding_box(Mesh &bounding, bool triangulate = true, double pad = 0.0) {
    using namespace CGAL::Polygon_mesh_processing;

    if (pad == 0.0) pad = (zmax - zmin) / 10;
    if (pad == 0.0) pad = 10.0;

    vertex_descriptor v1 = bounding.add_vertex(K::Point_3(xmax+pad, ymax+pad, zmax+pad));
    vertex_descriptor v2 = bounding.add_vertex(K::Point_3(xmax+pad, ymin-pad, zmax+pad));
    vertex_descriptor v3 = bounding.add_vertex(K::Point_3(xmin-pad, ymax+pad, zmax+pad));
    vertex_descriptor v4 = bounding.add_vertex(K::Point_3(xmin-pad, ymin-pad, zmax+pad));
    vertex_descriptor v5 = bounding.add_vertex(K::Point_3(xmax+pad, ymax+pad, zmin-pad));
    vertex_descriptor v6 = bounding.add_vertex(K::Point_3(xmax+pad, ymin-pad, zmin-pad));
    vertex_descriptor v7 = bounding.add_vertex(K::Point_3(xmin-pad, ymax+pad, zmin-pad));
    vertex_descriptor v8 = bounding.add_vertex(K::Point_3(xmin-pad, ymin-pad, zmin-pad));
    /*
     * these have normals inwards
    bounding.add_face(v7, v8, v6, v5); // zmin bottom
    bounding.add_face(v1, v2, v4, v3); // zmax top
    bounding.add_face(v3, v4, v8, v7); // xmin
    bounding.add_face(v5, v6, v2, v1); // xmax
    bounding.add_face(v6, v8, v4, v2); // ymin
    bounding.add_face(v1, v3, v7, v5); // ymax
     */
    bounding.add_face(v5, v6, v8, v7); // zmin bottom
    bounding.add_face(v3, v4, v2, v1); // zmax top
    bounding.add_face(v7, v8, v4, v3); // xmin
    bounding.add_face(v1, v2, v6, v5); // xmax
    bounding.add_face(v2, v4, v8, v6); // ymin
    bounding.add_face(v5, v7, v3, v1); // ymax

    stitch_borders(bounding);
    remove_isolated_vertices(bounding); // probably not needed

    if (triangulate && !triangulate_faces(bounding)) {
      cerr << "Cannot triangulate bounding box" << endl;
      return false;
    }
    return true;
  }

  bool get_clipped_bounding_box(Mesh &res, Kernel::Plane_3 plane) {
    using namespace CGAL::Polygon_mesh_processing;
    using namespace CGAL::parameters;
    const bool use_nef = false;

    Mesh bounding;
    if (!get_bounding_box(bounding)) return false;

    if (use_nef) {
      Nef bounding_nef(bounding);
      Nef result;

      const bool use_halfspace = false; // not allowed by kernel
      if (use_halfspace) {
        Nef halfspace(plane, Nef::EXCLUDED);
        result = bounding_nef.intersection(halfspace);
      } else {
        result = bounding_nef.intersection(plane, Nef::CLOSED_HALFSPACE);
      }

      CGAL::convert_nef_polyhedron_to_polygon_mesh(result, res);
    } else {
      cerr << "Clip bounding box" << endl;
      clip(bounding, plane, clip_volume(true));
      res = bounding;
    }

    return true;
  }

  bool clip_mesh(Mesh &current, std::list<Mesh> &queue, std::list<Mesh> &res) {
    using namespace CGAL::Polygon_mesh_processing;
    using namespace CGAL::parameters;
    using namespace CGAL;
    using Vector_3 = Kernel::Vector_3;
    using Plane_3 = Kernel::Plane_3;

    const bool debug = false;

    if (!is_triangle_mesh(current)) {
      cerr << "Mesh is not triangulated" << endl;
      return false;
    }
    if (!is_valid_polygon_mesh(current)) {
      cerr << "Mesh is not valid polygon" << endl;
      return false;
    }
    if (does_self_intersect(current)) {
      cerr << "Mesh self intersects!" << endl;
      return false;
    }
    if (!does_bound_a_volume(current)) {
      cerr << "Mesh does not bound a volume!" << endl;
      return false;
    }

    bool convex = true;

    // iterate over all faces
    for(edge_descriptor edge : current.edges()) {
      if (debug) cerr << "get first half-edge" << endl;
      // get the first half edge and 3 vertices around
      auto edge1 = current.halfedge(edge);
      auto v1p = current.target(current.prev(edge1));
      auto v1t = current.target(edge1);
      auto v1n = current.target(current.next(edge1));

      if (debug) cerr << "get second half-edge" << endl;
      // get the second half edge and 3 vertices around
      auto edge2 = current.opposite(edge1);
      auto v2p = current.target(current.prev(edge2));
      auto v2t = current.target(edge2);
      auto v2n = current.target(current.next(edge2));

      // get opposite points for each of the two faces
      // these points do not belongs to the edge
      auto p1 = current.point((v1p == v2t) ? v1n : v1p);
      auto p2 = current.point((v2p == v1t) ? v2n : v2p);
      if (debug) cerr << "get opposite points (" << p1 << "), (" << p2 << ")" << endl;

      // compute the normal for the reference face
      auto norm1 = compute_face_normal(current.face(edge1), current);
      if (debug) cerr << "get face normal " << norm1 << endl;

      // if the scalar product is positive, the angle is concave
      // <https://stackoverflow.com/a/40019587>
      auto product = scalar_product(Vector_3(p1, p2), norm1);
      if (debug) cerr << "dot product: " << product << endl;
      if( product > 0 ) {
        Plane_3 clip_plane(p1, norm1);
        Plane_3 rclip_plane(p1, -norm1);

        convex = false;
        Mesh complement;

        const bool custom_clip = false;
        const bool use_clip = false;
        if (use_clip && custom_clip) {
          cerr << "TODO: manual clip" << endl;
          cerr << "TODO: hole filling" << endl;
          // for all holes created:
          // triangulate_hole(mesh, an_edge_of_the_hole)
          return false;
        } else if (use_clip) {
          complement = current;
          cerr << "clip current mesh" << endl;
          clip(current, clip_plane, clip_volume(true));
          if (debug_meshes) CGAL::IO::write_OBJ("dbg_last_split.obj", current);
          cerr << "clip complement mesh" << endl;
          clip(complement, rclip_plane, clip_volume(true));
          if (debug_meshes) CGAL::IO::write_OBJ("dbg_last_split_complement.obj", complement);

          // Maybe try with clip_volume(false) and manually fill holes
          // (does not work)
          // triangulate_hole(current, )
          // triangulate_hole(complement)
        } else {
          // Alternative to clipping, use CSG difference instead of infinite
          // plane clipping which seems to have some issues.
          Mesh halfspace;
          Mesh current_res;
          if (!get_clipped_bounding_box(halfspace, clip_plane)) return false;
          if (debug_meshes) CGAL::IO::write_OBJ("dbg_last_split_tool.obj", halfspace);
          cerr << "clip/intersection current mesh" << endl;
          //corefine_and_compute_difference(current, halfspace, current_res);
          corefine_and_compute_intersection(current, halfspace, current_res);
          if (debug_meshes) CGAL::IO::write_OBJ("dbg_last_split.obj", current_res);
          if (!get_clipped_bounding_box(halfspace, clip_plane)) return false;
          cerr << "clip/intersection complement mesh" << endl;
          corefine_and_compute_difference(current, halfspace, complement);
          //corefine_and_compute_intersection(current, halfspace, complement);
          if (debug_meshes) CGAL::IO::write_OBJ("dbg_last_split_complement.obj", complement);
          current = current_res;
        }
        cerr << "push back 2 meshes" << endl;
        queue.push_back(complement);
        queue.push_back(current);
        if (does_self_intersect(current)) {
          cerr << "Mesh (dbg_last_split.obj) self intersects!" << endl;
          return false;
        }
        if (!does_bound_a_volume(current)) {
          cerr << "Mesh (dbg_last_split.obj) does not bound a volume!" << endl;
          return false;
        }
        if (does_self_intersect(complement)) {
          cerr << "Mesh (dbg_last_split_complement.obj) self intersects!" << endl;
          return false;
        }
        if (!does_bound_a_volume(complement)) {
          cerr << "Mesh (dbg_last_split_complement.obj) does not bound a volume!" << endl;
          return false;
        }
        // If the face iterator can work with a mesh changing, perhaps we can
        // continue to iterate over it but in the meantime put both meshes for
        // reprocess
        return true;
      }
    }

    if (convex) {
      // No concave surface got split, the mesh is convex
      // res.push_back(current);
      split_connected_components(current, res);
    }

    return true;
  }

  bool convex_decomposition(Mesh original, std::list<Mesh> &res) {
    using namespace CGAL::Polygon_mesh_processing;

    std::list<Mesh> queue;
    queue.push_back(original);
    int step = 0;
    while(queue.size()) {
      cerr << "Clip #" << step << " mesh " << queue.size() << " / " << (queue.size() + res.size()) << endl;
      Mesh working = queue.front();
      queue.pop_front();
      orient_to_bound_a_volume(working);
      if (debug_meshes) CGAL::IO::write_OBJ(format("dbg_convex_step_{}.obj", step), working);
      if(!clip_mesh(working, queue, res)) return false;
      step++;
    }
    return true;
  }

  bool construct_csg_mesh(bool draw = false){
    using namespace CGAL::Polygon_mesh_processing;

    Mesh result;
    if (!get_bounding_box(result)) return false;
    if (debug_meshes) CGAL::IO::write_OBJ("dbg_bounding.obj", result);

    int step = 0;
    for(CsgNode csg_node : csg_tree) {
      step++;
      cerr << "CSG(Mesh) Step #" << step << endl;
      Mesh step_res;
      Mesh step_brush = meshes[csg_node.index];
      std::string op = "noop";
      if (!triangulate_faces(step_brush)) {
        cerr << "Cannot triangulate mesh " << csg_node.index << endl;
        return false;
      }
      switch (csg_node.oper) {
        case CSG_Add:
          corefine_and_compute_union(result, step_brush, step_res);
          result = step_res;
          op = "csg-add";
          break;
        case CSG_Subtract:
          corefine_and_compute_difference(result, step_brush, step_res);
          result = step_res;
          op = "csg-sub";
          break;
        default:
          break;
      }
      if (debug_meshes) CGAL::IO::write_OBJ(format("dbg_step_{}_{}_brush.obj", step, op), step_brush);
      if (debug_meshes) CGAL::IO::write_OBJ(format("dbg_step_{}_{}_result.obj", step, op), result);
    }

#ifdef CONFIG_VIEWER
    if (draw) CGAL::draw(result);
#endif

    if (debug_meshes) CGAL::IO::write_OBJ("dbg_result_concave.obj", result);

    cerr << "CSG(Nef) Final convex decomposition" << endl;
    std::list<Mesh> convex_meshes;
    if (!convex_decomposition(result, convex_meshes)) return false;

    if (debug_meshes) {
      int i = 0;
      ofstream fres("dbg_result.obj");
      for(Mesh m : convex_meshes) {
        CGAL::IO::write_OBJ(format("dbg_result_{}.obj", i++), m);
        CGAL::IO::write_OBJ(fres, result);
      }
    }

    this->worldspan = convex_meshes;

    return true;
  }

  bool construct_csg_nef(bool draw = false){
    using namespace CGAL::Polygon_mesh_processing;

    Mesh bounding;
    if (!get_bounding_box(bounding)) return false;
    if (debug_meshes) CGAL::IO::write_OBJ("dbg_bounding.obj", bounding);

    Nef result(bounding);

    int step = 0;
    for(CsgNode csg_node : csg_tree) {
      step++;
      cerr << "CSG(Nef) Step #" << step << endl;
      Mesh step_brush = meshes[csg_node.index];
      std::string op = "noop";
      if (!triangulate_faces(step_brush)) {
        cerr << "Cannot triangulate mesh " << csg_node.index << endl;
        return false;
      }
      if (debug_meshes) CGAL::IO::write_OBJ(format("dbg_step_{}_brush.obj", step), step_brush);
      cerr << "CSG(Nef) convert mesh to nef" << endl;
      Nef step_nef(step_brush);
      switch (csg_node.oper) {
        case CSG_Add:
          cerr << "CSG(Nef) add " /*<< result << " ; " << step_nef*/ << endl;
          result = result.join(step_nef);
          op = "csg-add";
          break;
        case CSG_Subtract:
          cerr << "CSG(Nef) sub " /*<< result << " ; " << step_nef*/ << endl;
          result = result.difference(step_nef);
          op = "csg-sub";
          break;
        default:
          break;
      }
      Mesh mesh_result;
      CGAL::convert_nef_polyhedron_to_polygon_mesh(result, mesh_result);
      if (debug_meshes) CGAL::IO::write_OBJ(format("dbg_step_{}_{}_result.obj", step, op), mesh_result);
    }

    Mesh mesh_result;
    CGAL::convert_nef_polyhedron_to_polygon_mesh(result, mesh_result);
    if (debug_meshes) CGAL::IO::write_OBJ("dbg_result_concave.obj", mesh_result);

#ifdef CONFIG_VIEWER
    if (draw) CGAL::draw(result);
#endif

    cerr << "CSG(Nef) Final convex decomposition" << endl;
    CGAL::convex_decomposition_3(result);

    CGAL::convert_nef_polyhedron_to_polygon_mesh(result, mesh_result);
    if (debug_meshes) CGAL::IO::write_OBJ("dbg_result.obj", mesh_result);

    return true;
  }

#if 0
  Kernel::Vector_3 unit_vector(Kernel::Vector_3 vec) {
    Kernel::Vector_3 v(vec);
    // Approx sqrt, no need to get real unit vectors. We just need to get
    // vectors with non zero coordinates when rounded
    // double ulen = std::sqrt(to_double(v.squared_length()));
    auto len = approximate_sqrt(v.squared_length());
    // For very small units, avoid divisions by 0 and increase vector size
    // before normalizing it
    while (std::abs(to_double(len)) < 1e-6) {
      v = v * 1e9;
      len = approximate_sqrt(v.squared_length());
    }
    return v / len;
  }

#endif

  Kernel::Vector_3 unit_vector(Kernel::Vector_3 vec) {
    Kernel::Vector_3 v(vec);
    while (v.squared_length() < 1e-9) v = v * 1e9;

    auto len = approximate_sqrt(v.squared_length());
    return v / len;
  }

  bool generate_brush(ostream &map, int idx, Mesh &m) {
    using Plane_3 = Kernel::Plane_3;
    using Point_3 = Kernel::Point_3;
    using Vector_3 = Kernel::Vector_3;

    bool uvfound;
    UVPropertyMap uvmap;
    boost::tie(uvmap, uvfound) = m.property_map<face_descriptor,UVMap>("f:uv");

    map
      << "  // brush " << idx << " uvmap: " << uvfound << endl
      << "  {" << endl;

#if 0
    Nef poly(m);
    for(Plane_3 plane : poly.planes()){
      UVMap uv;
      uv.texture = "__TB_empty";
      Point_3 p0 = plane.point();
      Point_3 p1 = p1 + plane.base1();
      Point_3 p2 = p1 + plane.base2();
      map
        << "    "
        << "(" << p0.x() << " " << p0.y() << " " << p0.z() << ") "
        << "(" << p1.x() << " " << p1.y() << " " << p1.z() << ") "
        << "(" << p2.x() << " " << p2.y() << " " << p2.z() << ") "
        << uv.texture
        << endl;
    }
#endif

    std::list<Plane_3> planes;

    for(face_descriptor f : m.faces()) {
      // TODO: if face is too small in one direction, it can be imprecise and
      // produce very small vectors that are hard to reason with. In that case
      // there might be a better face on the same plane with better properties.
      auto edge = m.halfedge(f);
      Point_3 v0 = m.point(m.target(m.prev(edge)));
      Point_3 v1 = m.point(m.target(edge));
      Point_3 v2 = m.point(m.target(m.next(edge)));
      Plane_3 plane(v0, v1, v2);
      Vector_3 normal = plane.orthogonal_vector();
      Vector_3 b1 = unit_vector(plane.base1());
      Vector_3 b2 = unit_vector(plane.base2());

      Point_3 p0 = v0;
      Point_3 p1 = p0 + b1;
      Point_3 p2 = p0 + b2;

      UVMap uv;
      if (uvfound) uv = uvmap[f];
      if (uv.texture == "") {
        uv.texture = "__TB_empty";
        uv.u = b1;
        uv.v = b2;
      }

      if (std::find(planes.begin(), planes.end(), plane) != planes.end()) continue;
      planes.push_back(plane);

      if (uv.u * normal != 0) {
        cerr << "U vector is not included in face: u.n = " << (uv.u * normal) << endl;
        return false;
      }

      if (uv.v * normal != 0) {
        cerr << "V vector is not included in face: v.n = " << (uv.v * normal) << endl;
        return false;
      }

      map << "    ";
      // if (plane.is_degenerate()) map << "// DEGENERATE: ";
      map
        << "( " << p2.x() << " " << p2.y() << " " << p2.z() << " ) "
        << "( " << p1.x() << " " << p1.y() << " " << p1.z() << " ) "
        << "( " << p0.x() << " " << p0.y() << " " << p0.z() << " ) "
        << uv.texture << " "
        << "[ "
          << uv.u.x() << " " << uv.u.y() << " " << uv.u.z() << " "
          << uv.upan // TODO: pan from the world origin
        << " ] "
        << "[ "
          << uv.v.x() << " " << uv.v.y() << " " << uv.v.z() << " "
          << uv.vpan // TODO: pan from the world origin
        << " ] "
        << 0 << " " // rotation is 0, we have U and V for that
        << 1 << " " << 1 << " " // u and v scale  (vector contains the scale)
        << endl;
    }

    map << "  }" << endl;
    return true;
  }

  bool generate_brushes(ostream &map) {
    int idx = 0;
    for(Mesh m : this->worldspan) {
      if (!generate_brush(map, idx++, m)) {
        map << "// ERROR: stop export" << endl;
        return false;
      }
    }
    return true;
  }

  bool generate_map_file(ostream &map) {
    bool res = true;
    map
      << fixed
      << "// Game: Generic" << endl
      << "// Format: Valve" << endl
      << "// Generated: t3d2map" << endl
      << "{" << endl
      << "  \"mapversion\" \"220\"" << endl
      << "  \"classname\" \"worldspan\"" << endl;
    res = generate_brushes(map);
    map
      << "}" << endl;
    return res;
  }

};

int main(int argc, char **argv) {
  install_backtrace();

  ofstream output_file;
  ostream *mapfile = &cout;
  bool mesh = true;
  bool debug_meshes = false;
  int i = 1;
  for(; i < argc; i++) {
    string arg(argv[i]);
    if (arg == "--mesh") {
      mesh = true;
    } else if (arg == "--nef") {
      mesh = false;
    } else if (arg == "--debug-mesh") {
      debug_meshes = true;
#ifdef CGAL_USE_TRACE
    } else if (arg == "--cgal-debug") {
      debugthread = 0;
#endif
    } else if (i+1 < argc && (arg == "-o" || arg == "--output")) {
      arg = argv[++i];
      output_file = ofstream(arg);
      ostream *out = &output_file;
      mapfile = out;
    } else {
      break;
    }
  }

  if (i + 1 != argc) {
    cerr
      << "Usage: " << argv[0] << " [OPTIONS...] T3D_FILE > MAP_FILE" << endl
      << endl
      << "Options:" << endl
      << "    --nef, --mesh   Choose method to compute the results, default is nef." << endl
      << "    --debug-mesh    Generate in the durrent directoruy the debug meshes" << endl
      << "    -o OUTPUT       Generate map file to OUTPUT (or stdout if not defined)" << endl;
    return 1;
  }

  ifstream t3dfile(argv[i]);
  Map map;
  map.debug_meshes = debug_meshes;
  if(!t3dfile.is_open()) {
    perror("Error open");
    exit(1);
  }
  if (!map.parse_map(t3dfile)) return 1;
  if (mesh) {
    if (!map.construct_csg_mesh(false)) return 1;
  } else {
    if (!map.construct_csg_nef(false)) return 1;
  }
  if (!map.generate_map_file(*mapfile)) return 1;
  return 0;
}

