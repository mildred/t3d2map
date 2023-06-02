# t3d2map-attempt
Convert WhelOfTime / Unreal1 maps in t3d to something else

Unfinished draft that tried in both C++ and ruby to make something but failed. See https://github.com/hogsy/t3d2map and my fork of it.

The issue with this conversion is that : 

- map files are composed of convex only brushes
- t3d represents a CSG tree buth substracts and adds

## resources

.map file format

- https://book.leveldesignbook.com/appendix/resources/formats/map
- https://quakewiki.org/wiki/Quake_Map_Format

T3D format:

- https://wiki.beyondunreal.com/Legacy:T3D_File

Convex decomposition:

- https://gamedev.stackexchange.com/questions/53142/decomposing-a-concave-mesh-into-a-set-of-convex-meshes
- https://doc.cgal.org/latest/Convex_decomposition_3/index.html
- https://github.com/mjjq/ConvexDecomposition

tools:

- https://github.com/PyMesh/PyMesh
