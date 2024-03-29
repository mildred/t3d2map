# t3d2map
Convert WhelOfTime / Unreal1 maps in t3d to Valve .map files

Work in progress... The issue with this conversion is that map files are only composed of convex brushes while Unreak/t3d starts with a filled world and use CSG operations to excavate from it resulting in mostly concave geometry. This converter makes use of CGAL algorithms to achieve this.

![map file created with Unreal 1 engine and opened in TrenchBroom](./screenshoot.png)

This is however a very heavy work even for very simple maps because CSG and
triangulation create many many vertices and there is no optimization. Perhaps a
better solution would be to parse Unreal compiled maps and extract the BSP tree
which already contains a stream of convex polygons ready to be imported in any
other engine without running costly CSG.

- CSG : constructive solid geometry : boolean operations resulting in a very
  concave mesh
- BSP : Binary Space Partitionning : a way to represent a concave mesh with a
  set of convex meshes by partitionning the concave mesh in various places until
  this makes a convex mesh.

This repository makes use of CGAL to perform the CSG to get the concave mesh.
This in itself is error prone because the concavity of Unreal Engine maps is
very concave. Also planes that splits zones cannot be represented. Then there is
an improvized BSP cutting algorithm (cut at random places until there are no
more concave meshes) that is very inefficient.

T3D file format stores the CSG tree but the compiled Unreal Engine maps have the
pre-computed BSP tree that might be exported. This would be a better strategy.
See <https://www.acordero.org/projects/unreal-tournament-package-tool/> which
can open Unreal Engine Packages and view polys.

## C++ Build

Conan build does not work because of conan v2, until it works, ensure CGAL is
installed system-wide. Once it works, perhaps this will be how to install the
CGAL dependency:

    conan profile detect --force
    conan install . --output-folder=build --build=missing

Build with CMake:

    cmake .
    make

## Run

Use:

    ./t3d2map ./examples/test.t3d

Or:

    make && rm -f dbg_*.obj && ./t3d2map --debug-mesh --convert ../WoT-conversion/textures/CONVERSION --game WoT -o examples/test.map examples/test.t3d

Or:

    make && for t3d in ~/Games/wineprefix/WoT/drive_c/*.T3D; do ./t3d2map --obj "${t3d%.*}.obj" "$t3d"; done

## Usage

The `--game` flag is only used to populate the `// Game: xxx` at the top of the file to ensure TrenchBroom pick up the correct game settings when opening the file.

The CONVERSION file via the `--convert` flag is a file containing a texture for each line. For each texture it has 3 fields separated by spaces:

- the texture name as it appears in t3d file
- the texture package name for the worldspawn `_tb_textures`
- the texture name for the .map file

example:

    BannWall1918 textures/AesSedaiT.utx AesSedaiT.utx/BannWall1918
    BannWall1919 textures/AesSedaiT.utx AesSedaiT.utx/BannWall1919
    BookBook1752 textures/AesSedaiT.utx AesSedaiT.utx/BookBook1752
    BookBook1755_m textures/AesSedaiT.utx AesSedaiT.utx/BookBook1755_m


## Roadmap:

- [x] Parse a list of T3D files representing (convex?) polygons
- [x] Generate .obj file for each .t3d file
- [ ] Check that this is really the BSP tree for a map... not sure anymore
- [ ] If the polygons are all convex, generate a map file
- [ ] Add code to parse Unreal Engine packages instead of t3d files

---

- [x] Parse t3d files
- [x] Parse brushes
- [x] Implement actor translation
- [ ] Implement actor rotation
- [ ] Implement actor scaling
- [x] Generate CSG concave mesh
    - [ ] Handle plane meshes (exclude from CSG tree). Currently it voids the
      entire scene.
- [x] Decompose in convex meshes
- [x] Split disjoined meshes while keeping the uvmap
- [x] Generate .map file
    - [x] Handle rounding errors producing vectors with all 3 coordinates at 0
- [x] Parse texture UV mapping
- [x] Generate a list of texture mappings and associate each face with a mapping
    - texture name
    - U vector
    - V vector
    - Texture origin (t3d.origin + uvector * (upan or 0) + vvector * (vpan or 0))
- [ ] Handle two coplanar faces in the same resulting mesh having different
  texture
    - coplanar polygons on a mesh must all share the same texture mapping, else
      those polygons must be split in separate meshes.
- [ ] Generate UV mapping to .map files
    - transform the U and V vectors to unit vectors and extract the scale factor
      for "X scale" and "Y scale"
    - set rotation to 0 (embedded within the U, V vectors)
    - transform the texture origin to U and V offsets discarding the component
      orthogonal to U and V as this is not necessary
    - try multiple texture and look to see if visually it matches

## CGAL algorithm

Note: I believe there is no way to have face properties in a Nef_Polyhedra. Given this limitation there are two options:

- find a way to reconstruct the mapping of output face properties from the original input mesh face without keeping a track of it during boolean operation or decomposition. Pro: use the Nef_Polyhedra boolean algorithms that seems more powerful than mesh algorithms
   - store face properties (texture and all vertices) indexed by each of their vertices
   - in the output mesh, for each triangular face
       - get all the face properties for all their 3 vertices. Do not keep duplicate face properties
       - for each face property, check if the current face is a subset of the original face
           - apply rotation and translation to both to eliminate z coordinate and work in 2D
           - use 2D boolean algorithm to check inclusion / intersection. Possibly full inclusion might not be met because of floating point errors.
       - if multiple faces matches, keep the face for which the (intersection surface - excluded surface) is the greatest compared to the original surface
       - assign the face property to the face
       - assign the face properties to the face vertices that did not reference it
   - keep doing the previous iteration step until no more faces are being assigned
   - assign the rest of faces with null properties
   - if a face is split in a non contiguous manner, the non contiguous parts will not receive face properties. Solution: face properties should be indexed by their normal and their distance from the origin. Because of floating point errors, it is possible that generated faces are not exactly coplanar. Comparaison must be performed with some epsilon, and probably hash based stucture is not going to fit.
   - this is possibly expansive
- Use custom algorithm for convex decomposition and perform everything with the Surface_mesh objects. Use clip https://doc.cgal.org/latest/Polygon_mesh_processing/index.html#title21

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
- https://book.leveldesignbook.com/appendix/resources/formats/map#texturing-uvs
- https://quakewiki.org/wiki/Quake_Map_Format
- https://3707026871-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-LtVT8pJjInrrHVCovzy%2Fuploads%2FEukkFYJLwfafFXUMpsI2%2FMAPFiles_2001_StefanHajnoczi.pdf?alt=media&token=51471685-bf69-42ae-a015-a474c0b95165
- https://developer.valvesoftware.com/wiki/UV_map
- https://www.gamers.org/dEngine/quake/QDP/QPrimer.html

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
- https://www.acordero.org/projects/unreal-tournament-package-tool/

---

- https://www.oldunreal.com/wiki/index.php?title=Main_Page

## UV Mapping

T3D format defines UV mapping for each face with 2 vectors (U, V) and 2 numbers
(UPan, VPan). Both U and V must be orthogonal to the face normal vector (hence
be "coplanar" with the face). The 2D point defined by U and V represents the
origin on the 2D texture coordinates (pixels). U=2 means that the texture starts
at the 3rd pixel on the left.

The texture is applied to the face such that on 3D space the U vector represents
the horizontal axis of the  texture image, and the V vector represents the
vertical axis. For a texture not to be deformed the U and V vector must be
orthogonal. The length of the vectors represents the length of the texture. A
unit vector represents the full width or height of the texture.

The .map format represents convex meshes with a set of intersecting planes, each
plane represented by 3 points (not necessarily on the surface mesh) organized so
the normal vector points out. Original texture information was subject to
incoherences so Valve updated the format to remove all ambiguity.

Quake standard format for UV mapping represents for each face after the texture
name real numbers for : X offset, Y offset, rotation, X scale, Y scale. Texture
are applied naturally on faces that are orthogonal to the X, Y or Z axis. If a
face is not orthogonal to the X, Y or Z axis, the texture will be stretched in
some way.

Valve texture format represents UV mapping after the texture name with:

- U 3d vector and U offset (from world origin 0,0,0 ?)
- V 3d vector and V offset (from world origin 0,0,0 ?)
- rotation
- X scale
- Y scale
