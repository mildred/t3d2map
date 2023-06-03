# t3d2map-attempt
Convert WhelOfTime / Unreal1 maps in t3d to something else

Unfinished draft that tried in both C++ and ruby to make something but failed. See https://github.com/hogsy/t3d2map and my fork of it.

The issue with this conversion is that : 

- map files are composed of convex only brushes
- t3d represents a CSG tree buth substracts and adds

## CGAL algorithm

Note: I believe there is no way to have face properties in a Nef_Polyhedra. Given this limitation there are two options:

- find a way to reconstruct the mapping of output face properties from the original input mesh face without keeping a track of it during boolean operation or decomposition. Pro: use the Nef_Polyhedra boolean algorithms that seems more powerful than mesh algorithms
   - store face properties (texture and all vertices) indexed by each of their vertices
   - in the output mesh, for each triangular face
       - get all the face properties for all their 3 vertices. Do not keep duplicate face properties
       - for each face property, check if the current face is a subset of the original face
       - assign the face property to the face
       - assign the face properties to the face vertices that did not reference it
   - keep doing the previous iteration step until no more faces are being assigned
   - assign the rest of faces with null properties
   - if a face is split in a non contiguous manner, the non contiguous parts will not receive face properties. Solution: face properties should be indexed by their normal and their distance from the origin.
   - this is possibly expansive
- Use custom algorithm for convex decomposition and perform everything with the Surface_mesh objects.

General algorithm:

- if the corefine boolean algorithms allows to work on a Nef_Polyhedra, use this data structure for boolean operation, else use Surface_mesh or a data structure that is compatible
- start with a world object that spans the world bounding box plus some padding, no texture (or transparent texture)
- for each brush in the t3d file
    - get the polygon soup to CGAL to a `Surface_mesh` (and copy it to a Nef_Polyhedra if that is the format we are using https://github.com/CGAL/cgal/issues/5431 )
    - add face properties corresponding to the UV texture mapping (origin, U, V, texture name) https://doc.cgal.org/latest/Surface_mesh_parameterization/index.html https://doc.cgal.org/latest/Surface_mesh/Surface_mesh_2sm_properties_8cpp-example.html https://doc.cgal.org/latest/Surface_mesh/index.html#title7
    - triangulate mesh if needed using a visitor to copy source face properties to target faces
    - apply the CSG binary operation of the given object with the world using the corefine algorithms to keep texture information. Have the corefine visitor copy the texture information from the source faces to the resulting faces
    - the new mesh is now the new world, go on to the next T3D mesh
- copy the world to a polyhedron: https://github.com/CGAL/cgal/issues/5431 (if needed). Copy face properties.
- create a convex decomposition of the world polyhedra, make sure that the face properties are conserved
- alternative: use hand written algorithm to decompose a Surface_mesh into convex meshes
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

Boolean operations:

- https://stackoverflow.com/questions/68860150/subtracting-multiple-polyhedra-continuously-from-a-polyhedron

tools:

- https://github.com/PyMesh/PyMesh
