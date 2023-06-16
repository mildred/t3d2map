#include <fstream>
#include <iostream>
#include <regex>
#include <format>
#include <map>
#include <cmath>
#include <algorithm>

#include <boost/algorithm/string.hpp>

#include <CGAL/Named_function_parameters.h>
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
typedef Mesh::Halfedge_index halfedge_descriptor;

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

std::regex reg_conversion_line("^(\\S+)\\s+(\\S+)\\s+(\\S+)$");
std::regex reg_keyvals("^\\s*(Begin\\s+\\S+\\s+)?(((\\S+)=(\\S+)\\s*)*)$");
std::regex reg_keyval_spc("([^=]+)=(\\S*)\\s*");
std::regex reg_polygon_vertex("^Vertex\\s+([^,]+),([^,]+),([^,]+)$");
std::regex reg_polygon_vector("^(Vertex|Origin|TextureU|TextureV)\\s+([^,]+),([^,]+),([^,]+)$");
std::regex reg_polygon_pan("^Pan\\s+U=(\\S+)\\s+V=(\\S+)$");
std::regex reg_polygon_attr("^(\\S+)\\s+([^,]+),([^,]+),([^,]+)$");
std::regex reg_keyval("^([^=]+)=(.*)$");
std::regex reg_location("^Location=\\((.*)\\)$");
std::regex reg_xyz_vector_var("([XYZ])=([^,\\)]+)");

struct TextureConversion {
  TextureConversion() {}
  virtual std::string convert(std::string) = 0;
  virtual std::set<std::string> packages() = 0;
};

struct FileTextureConversionLine {
  FileTextureConversionLine() {}
  FileTextureConversionLine(std::string pkg, std::string tex) : package(pkg), texture(tex) {}
  std::string package;
  std::string texture;
};

class FileTextureConversion: public TextureConversion {
  std::map<std::string, FileTextureConversionLine> map;
  std::set<std::string> pkgs;

  public:
  FileTextureConversion() { }
  FileTextureConversion(std::string fname) {
    ifstream f(fname);
    std::string line;
    std::smatch m;

    while (std::getline(f, line)) {
      if (!regex_match(line, m, reg_conversion_line)) continue;
      FileTextureConversionLine convline(m[2], m[3]);
      map[m[1]] = convline;
    }
  }

  virtual std::string convert(std::string tex) {
    FileTextureConversionLine res = map[tex];
    if (res.texture == "") return tex;

    pkgs.insert(res.package);
    return res.texture;
  };

  virtual std::set<std::string> packages() {
    return pkgs;
  };
};

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

bool has_uvmap(Mesh &mesh) {
  bool found;
  UVPropertyMap uvmap;
  boost::tie(uvmap, found) = mesh.property_map<face_descriptor,UVMap>("f:uv");
  return found;
}

void add_uvmap(Mesh &mesh) {
  mesh.add_property_map<face_descriptor,UVMap>("f:uv", UVMap());
}

class FaceUVMapCopyVisitor {
  using Triangle_mesh = Mesh;
  using Boolean_operation_type = CGAL::Polygon_mesh_processing::Corefinement::Boolean_operation_type;
  face_descriptor last_f_split;

  UVPropertyMap uvmap;
  bool has_uvmap;
  bool debug;

  public:

  FaceUVMapCopyVisitor(Triangle_mesh &mesh) : debug(true) {
    boost::tie(uvmap, has_uvmap) = mesh.property_map<face_descriptor,UVMap>("f:uv");
  }

  FaceUVMapCopyVisitor() : has_uvmap(false), debug(false) {
  }

  void before_subface_creations (face_descriptor f_split, const Triangle_mesh &tm) {
    last_f_split = f_split;
    bool found;
    UVPropertyMap uv;
    boost::tie(uv, found) = tm.property_map<face_descriptor,UVMap>("f:uv");
    if (debug) cerr << "last_f_split = " << last_f_split << " (mesh given) " << uv[f_split].texture << endl;
  }

  void before_subface_creations (face_descriptor f_split) {
    last_f_split = f_split;
    if (debug) cerr << "last_f_split = " << last_f_split << " (visitor mesh) " << uvmap[f_split].texture << endl;
  }

  void after_subface_created (face_descriptor f_new, const Triangle_mesh &tm) {
    bool found;
    UVPropertyMap uvmap;
    boost::tie(uvmap, found) = tm.property_map<face_descriptor,UVMap>("f:uv");
    if (found) {
      if (debug) cerr << "uvmap[f_new] = uvmap[last_f_split] (from given mesh) " << f_new << " " << last_f_split << " " << uvmap[last_f_split].texture << endl;
      uvmap[f_new] = uvmap[last_f_split];
    }
  }

  void after_subface_created (face_descriptor f_new) {
    if (has_uvmap) {
      if (debug) cerr << "uvmap[f_new] = uvmap[last_f_split] (from visitor mesh) " << f_new << " " << last_f_split << " " << uvmap[last_f_split].texture << endl;
      uvmap[f_new] = uvmap[last_f_split];
    }
  }

  void after_face_copy (face_descriptor f_src, const Triangle_mesh &tm_src, face_descriptor f_tgt, const Triangle_mesh &tm_tgt) {
    bool found1, found2;
    UVPropertyMap uvmap1, uvmap2;
    boost::tie(uvmap1, found1) = tm_src.property_map<face_descriptor,UVMap>("f:uv");
    // boost::tie(uvmap2, created) = const_cast<Triangle_mesh&>(tm_tgt).add_property_map<face_descriptor,UVMap>("f:uv", UVMap());
    boost::tie(uvmap2, found2) = tm_tgt.property_map<face_descriptor,UVMap>("f:uv");
    if (found1 && found2) {
      if (debug) cerr << "uvmap2[f_tgt] = uvmap1[f_src] " << f_tgt << " " << f_src << " " << uvmap1[f_src].texture << endl;
      uvmap2[f_tgt] = uvmap1[f_src];
    } else if (!found2) {
      cerr << "Could not find property_map(f:uv) for destination mesh" << endl;
    } else if (!found1) {
      cerr << "Could not find property_map(f:uv) for source mesh " /*<< tm_src*/ << endl;
    }
  }

  // void before_subface_creations(face_descriptor f_split, const Triangle_mesh& tm){}
  void after_subface_creations(){}
  void before_subface_created(){}
  void after_subface_creations(const Triangle_mesh& tm){}
  void before_subface_created(const Triangle_mesh& tm){}
  // void after_subface_created(face_descriptor f_new, const Triangle_mesh& tm){}

  void before_edge_split(halfedge_descriptor h, const Triangle_mesh& tm){}
  void edge_split(halfedge_descriptor hnew, const Triangle_mesh& tm){}
  void after_edge_split(){}
  void add_retriangulation_edge(halfedge_descriptor h, const Triangle_mesh& tm){}
  void intersection_point_detected(std::size_t i_id,
                                   int sdim,
                                   halfedge_descriptor h_f,
                                   halfedge_descriptor h_e,
                                   const Triangle_mesh& tm_f,
                                   const Triangle_mesh& tm_e,
                                   bool is_target_coplanar,
                                   bool is_source_coplanar){}
  void new_vertex_added(std::size_t i_id, vertex_descriptor v, const Triangle_mesh& tm){}
  void before_face_copy(face_descriptor f_src, const Triangle_mesh& tm_src, const Triangle_mesh& tm_tgt){}
  // void after_face_copy(face_descriptor  f_src, const Triangle_mesh& tm_src,
  //                      face_descriptor  f_tgt, const Triangle_mesh& tm_tgt){}
  void before_edge_copy(halfedge_descriptor h_src, const Triangle_mesh& tm_src, const Triangle_mesh& tm_tgt){}
  void after_edge_copy(halfedge_descriptor h_src, const Triangle_mesh& tm_src,
                       halfedge_descriptor h_tgt, const Triangle_mesh& tm_tgt){}
  void before_edge_duplicated(halfedge_descriptor h, const Triangle_mesh& tm){}
  void after_edge_duplicated(halfedge_descriptor h_src,
                             halfedge_descriptor h_new, const Triangle_mesh& tm){}
  void intersection_edge_copy(halfedge_descriptor h_src1, const Triangle_mesh& tm_src1,
                              halfedge_descriptor h_src2, const Triangle_mesh& tm_src2,
                              halfedge_descriptor h_tgt,  const Triangle_mesh& tm_tgt){}
  void before_vertex_copy(vertex_descriptor v_src, const Triangle_mesh& tm_src, const Triangle_mesh& tm_tgt){}
  void after_vertex_copy(vertex_descriptor v_src, const Triangle_mesh& tm_src,
                         vertex_descriptor v_tgt, const Triangle_mesh& tm_tgt){}
  void start_filtering_intersections(){}
  void progress_filtering_intersections(double d){}
  void end_filtering_intersections(){}
  void start_handling_intersection_of_coplanar_faces(std::size_t n){}
  void intersection_of_coplanar_faces_step() const{}
  void end_handling_intersection_of_coplanar_faces() const{}
  void start_handling_edge_face_intersections(std::size_t n){}
  void edge_face_intersections_step(){}
  void end_handling_edge_face_intersections(){}
  void start_triangulating_faces(std::size_t n){}
  void triangulating_faces_step(){}
  void end_triangulating_faces(){}
  void start_building_output(){}
  void end_building_output(){}
  void filter_coplanar_edges(){}
  void detect_patches(){}
  void classify_patches(){}
  void classify_intersection_free_patches(const Triangle_mesh& tm){}
  void out_of_place_operation(Boolean_operation_type t){}
  void in_place_operation(Boolean_operation_type t){}
  void in_place_operations(Boolean_operation_type t1,Boolean_operation_type t2){}
};

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

  bool parse_keyval(std::string line, std::map<std::string,std::string> &res) {
    std::smatch m;
    if (!regex_match(line, m, reg_keyvals)) return false;
    std::string keyvals = m[2];
    cerr << "parse_keyval from: " << keyvals << endl;

    std::sregex_iterator rit( keyvals.begin(), keyvals.end(), reg_keyval_spc);
    while(rit != std::sregex_iterator()) {
      std::smatch m = *rit;
      cerr << "parse_keyval [" << m[1] << "]=" << m[2] << endl;
      res[m[1]] = m[2];
      ++rit;
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

    cerr << "parse_polygon: polygon_line: " << polygon_line << endl;
    cerr << "parse_polygon: Texture=" << keyval["Texture"] << endl;
    texture_map.texture = keyval["Texture"];

    while (parse_line(f, line)) {
      if (line.starts_with("End Polygon")) {
        const face_descriptor f = mesh.add_face(vertices);
        // TODO: flip normal if needed
        uvmap[f] = texture_map;
        cerr << "parse_polygon: uvmap[" << f << "]=" << texture_map.texture << endl;
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

      Mesh res2;
      CGAL::convert_nef_polyhedron_to_polygon_mesh(result, res2);
      res = res2;
    } else {
      cerr << "Clip bounding box" << endl;
      clip(bounding, plane, clip_volume(true));
      res = bounding;
    }

    // Add empty property map so each mesh in the process has it and we can fail
    // if we detect a mesh without a property map.
    add_uvmap(res);

    return true;
  }

  bool clip_mesh(Mesh &current, std::list<Mesh> &queue, std::list<Mesh> &res) {
    using namespace CGAL::Polygon_mesh_processing;
    using namespace CGAL::parameters;
    using namespace CGAL;
    using Vector_3 = Kernel::Vector_3;
    using Plane_3 = Kernel::Plane_3;


    if (!has_uvmap(current)) {
      cerr << "Missing UV Map in source mesh for clip_mesh()" << endl;
      return false;
    }

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

        FaceUVMapCopyVisitor vis;
        Mesh halfspace;
        Mesh current_res;
        if (!get_clipped_bounding_box(halfspace, clip_plane)) return false;
        if (debug_meshes) CGAL::IO::write_OBJ("dbg_last_split_tool.obj", halfspace);
        if (!has_uvmap(halfspace)) {
          cerr << "halfspace: missing uvmap" << endl;
          return false;
        }

        cerr << "clip/intersection current mesh" << endl;
        halfspace.add_property_map<face_descriptor,UVMap>("f:uv", UVMap());
        current_res.add_property_map<face_descriptor,UVMap>("f:uv", UVMap());
        corefine_and_compute_intersection(current, halfspace, current_res, CGAL::parameters::visitor(vis));
        if (debug_meshes) CGAL::IO::write_OBJ("dbg_last_split.obj", current_res);

        cerr << "clip/intersection complement mesh" << endl;
        complement.add_property_map<face_descriptor,UVMap>("f:uv", UVMap());
        if (!has_uvmap(current)) {
          cerr << "current: missing uvmap" << endl;
          return false;
        }
        if (!has_uvmap(halfspace)) {
          cerr << "halfspace: missing uvmap (bis)" << endl;
          return false;
        }
        if (!has_uvmap(complement)) {
          cerr << "complement: missing uvmap" << endl;
          return false;
        }
        corefine_and_compute_difference(current, halfspace, complement, CGAL::parameters::visitor(vis));
        if (debug_meshes) CGAL::IO::write_OBJ("dbg_last_split_complement.obj", complement);
        current = current_res;

        if (!has_uvmap(complement)) {
          cerr << "Missing UV Map in source mesh for clip_mesh complement" << endl;
          return false;
        }
        if (!has_uvmap(current)) {
          cerr << "Missing UV Map in source mesh for clip_mesh current (result)" << endl;
          return false;
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
      Mesh::Property_map<face_descriptor, std::size_t> fccmap = current.add_property_map<face_descriptor, std::size_t>("f:CC").first;
      std::size_t num_components = connected_components(current, fccmap);

      while (num_components > 1) {
        Mesh comp = current;
        face_descriptor fd = *faces(comp).first;

        // find first connected component
        std::vector<face_descriptor> cc;
        connected_component(fd, comp, std::back_inserter(cc));
        keep_connected_components(comp, cc);
        res.push_back(comp);

        std::vector<face_descriptor> cc2;
        connected_component(fd, current, std::back_inserter(cc2));
        remove_connected_components(current, cc2);
        num_components = connected_components(current, fccmap);
      }

      if (num_components == 1) {
        res.push_back(current);
      } else {
        // TODO: split disconnected components while keeping the propertry map
        split_connected_components(current, res); // this removes the uvmap
      }
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
    result.add_property_map<face_descriptor,UVMap>("f:uv", UVMap());
    if (debug_meshes) CGAL::IO::write_OBJ("dbg_bounding.obj", result);

    int step = 0;
    for(CsgNode csg_node : csg_tree) {
      step++;
      cerr << "CSG(Mesh) Step #" << step << endl;
      Mesh step_res;
      Mesh step_brush = meshes[csg_node.index];
      std::string op = "noop";
      FaceUVMapCopyVisitor vis(step_brush);
      if (!triangulate_faces(step_brush, CGAL::parameters::visitor(vis))) {
        cerr << "Cannot triangulate mesh " << csg_node.index << endl;
        return false;
      }
      switch (csg_node.oper) {
        case CSG_Add:
          add_uvmap(step_res);
          corefine_and_compute_union(result, step_brush, step_res, CGAL::parameters::visitor(vis));
          result = step_res;
          op = "csg-add";
          break;
        case CSG_Subtract:
          add_uvmap(step_res);
          corefine_and_compute_difference(result, step_brush, step_res, CGAL::parameters::visitor(vis));
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

  bool generate_brush(ostream &map, int idx, Mesh &m, TextureConversion &conv) {
    using Plane_3 = Kernel::Plane_3;
    using Point_3 = Kernel::Point_3;
    using Vector_3 = Kernel::Vector_3;

    if (!has_uvmap(m)) {
      cerr << "generate_brush: missing uvmap" << endl;
      return false;
    }

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

      if (std::find(planes.begin(), planes.end(), plane) != planes.end()) {
        continue;
      }
      planes.push_back(plane);

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
        << conv.convert(uv.texture) << " "
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

  bool generate_brushes(ostream &map, TextureConversion &conv) {
    int idx = 0;
    for(Mesh m : this->worldspan) {
      if (!generate_brush(map, idx++, m, conv)) {
        map << "// ERROR: stop export" << endl;
        return false;
      }
    }
    return true;
  }

  bool generate_map_file(ostream &map, std::string game, TextureConversion &conv) {
    bool res = true;

    // convert all textures beforehand to get the package list
    for(Mesh m : this->worldspan) {
      bool uvfound;
      UVPropertyMap uvmap;
      boost::tie(uvmap, uvfound) = m.property_map<face_descriptor,UVMap>("f:uv");
      if (!uvfound) continue;

      for(face_descriptor f : m.faces()) {
        conv.convert(uvmap[f].texture);
      }
    }
    std::string tex_packages;
    for(std::string pkg : conv.packages()) {
      if (tex_packages != "") tex_packages += ";";
      tex_packages += pkg;
    }

    map
      << fixed
      << "// Game: " << game << endl
      << "// Format: Valve" << endl
      << "// Generated: t3d2map" << endl
      << "{" << endl
      << "  \"mapversion\" \"220\"" << endl
      << "  \"classname\" \"worldspawn\"" << endl
      << "  \"_tb_textures\" \"" << tex_packages << "\"" << endl;
    res = generate_brushes(map, conv);
    map
      << "}" << endl;
    return res;
  }

};

int main(int argc, char **argv) {
  install_backtrace();

  std::string game = "Generic";
  FileTextureConversion file_conv;
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
    } else if (i+1 < argc && arg == "--game") {
      game = argv[++i];
    } else if (i+1 < argc && arg == "--convert") {
      arg = argv[++i];
      file_conv = FileTextureConversion(arg);
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
      << "    --convert FILE  use FILE for texture conversion" << endl
      << "    --game NAME     use this game" << endl
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
  if (!map.generate_map_file(*mapfile, game, file_conv)) return 1;
  return 0;
}

