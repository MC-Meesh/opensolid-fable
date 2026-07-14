export const DEFAULT_SCRIPT = `// Build a shape with the OpenSolid API and return it.
//
// Constructors (all centered at the origin, y is up):
//   Shape.sphere(r)
//   Shape.box3(hx, hy, hz)              half-extents
//   Shape.roundedBox(hx, hy, hz, r)     edge radius r
//   Shape.cylinder(r, halfHeight)       axis along y
//   Shape.cone(rBottom, rTop, halfH)    axis along y; a radius may be 0
//   Shape.torus(major, minor)           ring in the xz plane
//   Shape.capsule(x1,y1,z1, x2,y2,z2, r)
//   Shape.halfSpace(px,py,pz, nx,ny,nz)  solid side of a plane; intersect it
//                                       to terminate an extrude "up to face"
// Sweeps (build a closed 2D profile, then sweep it):
//   const p = new Profile(x, y);        start point
//   p.lineTo(x, y)  p.arcTo(x, y, bulge)  p.close()
//   p.ellipseArcTo(x,y, cx,cy, rx,ry, rotation, ccw)   elliptical arc
//   p.cubicTo(c1x,c1y, c2x,c2y, x,y)                   cubic Bezier
//   Shape.extrude(p, height, draft?)    profile (x,y)->(x,z), swept along +y;
//                                       optional draft angle (deg) tapers it
//   Shape.revolve(p, angleDegrees)      around the y axis, x is the radius
//   const path = new Path(x, y, z);     3D polyline; path.lineTo(x, y, z)
//   Shape.sweep(p, path)                profile swept along path (no twist)
//   Shape.loft(bottom, top, height)     morph bottom (y=0) to top (y=height);
//                                       parallel planes, linear cross-section
// Transforms (each returns a new shape):
//   .translate(x, y, z)
//   .rotate(ax, ay, az, angleRad)       about axis (ax,ay,az) through origin
//   .scale(sx, sy, sz)                  per-axis, factors > 0
//   .uniformScale(factor)               factor > 0
//   .taper(px,py,pz, nx,ny,nz, deg)     draft: tilt walls about the neutral
//                                       plane through (nx,ny,nz) with pull
//                                       axis (px,py,pz), by deg degrees
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
