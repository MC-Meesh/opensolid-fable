export const DEFAULT_SCRIPT = `// Build a shape with the OpenSolid API and return it.
//
// Constructors (all centered at the origin, y is up):
//   Shape.sphere(r)
//   Shape.box3(hx, hy, hz)              half-extents
//   Shape.roundedBox(hx, hy, hz, r)     edge radius r
//   Shape.cylinder(r, halfHeight)       axis along y
//   Shape.torus(major, minor)           ring in the xz plane
//   Shape.capsule(x1,y1,z1, x2,y2,z2, r)
//   Shape.halfSpace(px,py,pz, nx,ny,nz)  solid side of a plane; intersect it
//                                       to terminate an extrude "up to face"
// Sweeps (build a closed 2D profile, then sweep it):
//   const p = new Profile(x, y);        start point
//   p.lineTo(x, y)  p.arcTo(x, y, bulge)  p.close()
//   Shape.extrude(p, height, draft?)    profile (x,y)->(x,z), swept along +y;
//                                       optional draft angle (deg) tapers it
//   Shape.revolve(p, angleDegrees)      around the y axis, x is the radius
// Transforms (each returns a new shape):
//   .translate(x, y, z)
//   .rotate(ax, ay, az, angleRad)       about axis (ax,ay,az) through origin
//   .scale(sx, sy, sz)                  per-axis, factors > 0
//   .uniformScale(factor)               factor > 0
// Booleans (each returns a new shape):
//   .union(other)  .intersect(other)  .subtract(other)
//   .smoothUnion(other, radius?)        blended union; radius defaults to
//                                       10% of the combined bounds

const body = Shape.roundedBox(1.0, 0.55, 0.8, 0.15);
const bump = Shape.sphere(0.55).translate(0, 0.65, 0);
const solid = body.smoothUnion(bump, 0.25);
const hole = Shape.cylinder(0.28, 2.0);
return solid.subtract(hole);
`;
