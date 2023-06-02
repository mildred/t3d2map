# t3d2map-attempt
Convert WhelOfTime / Unreal1 maps in t3d to something else

Unfinished draft that tried in both C++ and ruby to make something but failed. See https://github.com/hogsy/t3d2map and my fork of it.

The issue with this conversion is that : 

- map files are composed of convex only brushes
- t3d represents a CSG tree buth substracts and adds

## CGAL algorithm

- start with a full Nef polyhedron as world or a Surface_mesh that spans the whole world plus some little bit
- for each brush
    - get the polygon soup to CGAL to a `Surface_mesh`
    - apply the texture https://doc.cgal.org/latest/Surface_mesh_parameterization/index.html
    - apply the CSG binary operation of the given object with the world (use corefine algorithms to keep texture?)
    - apply texture to vertices that did not change and detect new vertices (those without texture coordinates) https://sympa.inria.fr/sympa/arc/cgal-discuss/2011-09/msg00012.html
- copy the world to a polyhedron: https://github.com/CGAL/cgal/issues/5431 (if needed)
- create a convex decomposition of the world polyhedra
- export to .map

## resources

.map file format

- https://book.leveldesignbook.com/appendix/resources/formats/map
- https://quakewiki.org/wiki/Quake_Map_Format

T3D format:

- https://beyondunrealwiki.github.io/pages/t3d-file.html
- https://beyondunrealwiki.github.io/pages/brush.html
- https://beyondunrealwiki.github.io/pages/csg.html
- http://web.archive.org/web/20050913035722/http://users.skynet.be/fa550206/Slug-Production/Data/OpenGL-Documents/T3D-File-Format/T3D-File-Format.htm

Convex decomposition:

- https://gamedev.stackexchange.com/questions/53142/decomposing-a-concave-mesh-into-a-set-of-convex-meshes
- https://doc.cgal.org/latest/Convex_decomposition_3/index.html
- https://github.com/mjjq/ConvexDecomposition

tools:

- https://github.com/PyMesh/PyMesh
