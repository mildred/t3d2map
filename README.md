# t3d2map-attempt
Convert WhelOfTime / Unreal1 maps in t3d to something else

Unfinished draft that tried in both C++ and ruby to make something but failed. See https://github.com/hogsy/t3d2map and my fork of it.

The issue with this conversion is that : 

- map files are composed of convex only brushes
- t3d represents a CSG tree buth substracts and adds

## CGAL algorithm

- if the corefine boolean algorithms allows to work on a Nef_Polyhedra, use this data structure for boolean operation, else use Surface_mesh or a data structure that is compatible
- start with a world object that spans the world bounding box plus some padding, no texture (or transparent texture)
- for each brush in the t3d file
    - get the polygon soup to CGAL to a `Surface_mesh` (and copy it to a Nef_Polyhedra if that is the format we are using https://github.com/CGAL/cgal/issues/5431 )
    - add face properties corresponding to the UV texture mapping (origin, U, V, texture name) https://doc.cgal.org/latest/Surface_mesh_parameterization/index.html
    - triangulate mesh if needed using a visitor to copy source face properties to target faces
    - apply the CSG binary operation of the given object with the world using the corefine algorithms to keep texture information. Have the corefine visitor copy the texture information from the source faces to the resulting faces
    - the new mesh is now the new world, go on to the next T3D mesh
- copy the world to a polyhedron: https://github.com/CGAL/cgal/issues/5431 (if needed). Copy face properties.
- create a convex decomposition of the world polyhedra, make sure that the face properties are conserved
- new faces should be assigned the empty texture property (same as original world mesh)
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
