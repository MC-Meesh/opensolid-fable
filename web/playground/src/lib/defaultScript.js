export const DEFAULT_SCRIPT = `// Build a shape with the OpenSolid API and return it.
//
// Constructors (all centered at the origin, y is up):
//   Shape.sphere(r)
//   Shape.box3(hx, hy, hz)              half-extents
//   Shape.roundedBox(hx, hy, hz, r)     edge radius r
//   Shape.cylinder(r, halfHeight)       axis along y
//   Shape.torus(major, minor)           ring in the xz plane
//   Shape.capsule(x1,y1,z1, x2,y2,z2, r)
// Operations (each returns a new shape):
//   .translate(x, y, z)
//   .union(other)  .intersect(other)  .subtract(other)
//   .smoothUnion(other, radius?)

const body = Shape.roundedBox(1.0, 0.55, 0.8, 0.15);
const bump = Shape.sphere(0.55).translate(0, 0.65, 0);
const solid = body.smoothUnion(bump, 0.25);
const hole = Shape.cylinder(0.28, 2.0);
return solid.subtract(hole);
`;
