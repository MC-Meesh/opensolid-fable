//! Fluent builder API over the F-Rep path.
//!
//! [`shape`] holds the entry points; every constructor returns a [`Part`],
//! an immutable value combining a runtime-composable SDF ([`Shape`]) with a
//! conservative bounding box. Because a `Part` always knows a region that
//! contains its surface, [`Part::mesh`] needs only a resolution — no manual
//! bounding box bookkeeping.
//!
//! Design rules (see `spec/13-ai-api-design.md`):
//! - Zero hidden state: every operation consumes explicit inputs and
//!   returns a new `Part`; nothing depends on invisible context.
//! - All fallible operations return [`CoreResult`]; error messages name the
//!   offending argument and the violated constraint.
//! - Dimensions are radii and full sizes; angles are degrees.
//!
//! ```
//! use opensolid_kernel::builder::shape;
//!
//! let part = shape::sphere(1.0)?
//!     .at(0.0, 0.0, 0.0)?
//!     .union(shape::box3(1.6, 1.6, 1.6)?.at(0.9, 0.0, 0.0)?)
//!     .smooth(0.2)?
//!     .shell(0.1)?;
//! let mesh = part.mesh(64)?;
//! assert!(mesh.is_closed_manifold());
//! # Ok::<(), opensolid_kernel::core::error::CoreError>(())
//! ```

use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{BoundingBox3, Point3, Transform3, Vector3};
use opensolid_frep::Shape;
use opensolid_frep::mesh::{MeshOptions, mesh_sdf_indexed};
use opensolid_frep::ops::{Rounded, Shell};
use opensolid_frep::primitives::{Box3, Capsule, Cone, Cylinder, Sdf, Sphere, Torus};
use opensolid_frep::transform::SdfTransformExt;

/// Meshing needs the surface strictly inside the sampled region, clear of
/// the outermost cell layer. With a margin of `4 · extent / resolution` on
/// every side the surface stays at least two cells away from the boundary
/// for any `resolution >= MIN_MESH_RESOLUTION`.
const MIN_MESH_RESOLUTION: usize = 8;

fn positive_finite(argument: &'static str, value: f64) -> CoreResult<()> {
    if value > 0.0 && value.is_finite() {
        Ok(())
    } else {
        Err(CoreError::InvalidArgument {
            argument,
            reason: format!("must be positive and finite, got {value}"),
        })
    }
}

fn finite(argument: &'static str, value: f64) -> CoreResult<()> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(CoreError::InvalidArgument {
            argument,
            reason: format!("must be finite, got {value}"),
        })
    }
}

fn finite_point(argument: &'static str, p: &Point3) -> CoreResult<()> {
    if p.coords.iter().all(|c| c.is_finite()) {
        Ok(())
    } else {
        Err(CoreError::InvalidArgument {
            argument,
            reason: format!("must have finite coordinates, got {p:?}"),
        })
    }
}

fn corners(b: &BoundingBox3) -> [Point3; 8] {
    let (lo, hi) = (b.min, b.max);
    [
        Point3::new(lo.x, lo.y, lo.z),
        Point3::new(hi.x, lo.y, lo.z),
        Point3::new(lo.x, hi.y, lo.z),
        Point3::new(hi.x, hi.y, lo.z),
        Point3::new(lo.x, lo.y, hi.z),
        Point3::new(hi.x, lo.y, hi.z),
        Point3::new(lo.x, hi.y, hi.z),
        Point3::new(hi.x, hi.y, hi.z),
    ]
}

/// An immutable solid built on the F-Rep scene graph: a [`Shape`] plus a
/// conservative axis-aligned bounding box maintained through every
/// operation. Cloning is cheap (the SDF tree is shared).
///
/// Construct parts with the [`shape`] free functions and combine them with
/// the methods below; nothing here mutates in place.
#[derive(Clone)]
pub struct Part {
    shape: Shape,
    bounds: BoundingBox3,
}

/// The SDF tree has no useful `Debug` form; the bounds are the part's
/// observable summary.
impl std::fmt::Debug for Part {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Part")
            .field("bounds", &self.bounds)
            .finish_non_exhaustive()
    }
}

/// Entry points: primitive solids, each centered at the origin (except
/// [`capsule`], whose endpoints are explicit).
///
/// Rotational primitives ([`cylinder`], [`cone`], [`torus`]) have their
/// axis along **Y**, matching the underlying F-Rep primitives.
pub mod shape {
    use super::*;

    /// A sphere of the given `radius`, centered at the origin.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `radius` is not positive and finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let ball = shape::sphere(2.0)?;
    /// assert!(ball.distance(0.0, 0.0, 0.0) < 0.0); // inside
    /// assert!((ball.distance(2.0, 0.0, 0.0)).abs() < 1e-12); // on surface
    /// assert!(shape::sphere(-1.0).is_err());
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn sphere(radius: f64) -> CoreResult<Part> {
        positive_finite("radius", radius)?;
        let r = Vector3::new(radius, radius, radius);
        Ok(Part {
            shape: Shape::new(Sphere {
                center: Point3::origin(),
                radius,
            }),
            bounds: BoundingBox3::new(Point3::origin() - r, Point3::origin() + r),
        })
    }

    /// An axis-aligned box with the given full sizes along X, Y, and Z,
    /// centered at the origin. (`box` is a Rust keyword, hence `box3`.)
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if any size is not positive and finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let slab = shape::box3(4.0, 2.0, 1.0)?;
    /// assert!((slab.distance(2.0, 0.0, 0.0)).abs() < 1e-12); // +X face
    /// assert!((slab.distance(0.0, 0.0, 0.5)).abs() < 1e-12); // +Z face
    /// assert!(shape::box3(4.0, 0.0, 1.0).is_err());
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn box3(size_x: f64, size_y: f64, size_z: f64) -> CoreResult<Part> {
        positive_finite("size_x", size_x)?;
        positive_finite("size_y", size_y)?;
        positive_finite("size_z", size_z)?;
        let h = Vector3::new(size_x / 2.0, size_y / 2.0, size_z / 2.0);
        Ok(Part {
            shape: Shape::new(Box3 {
                center: Point3::origin(),
                half_extents: [h.x, h.y, h.z],
            }),
            bounds: BoundingBox3::new(Point3::origin() - h, Point3::origin() + h),
        })
    }

    /// A cylinder of the given `radius` and full `height`, centered at the
    /// origin with its axis along Y.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `radius` or `height` is not
    /// positive and finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let rod = shape::cylinder(0.5, 4.0)?;
    /// assert!((rod.distance(0.5, 0.0, 0.0)).abs() < 1e-12); // lateral surface
    /// assert!((rod.distance(0.0, 2.0, 0.0)).abs() < 1e-12); // top cap
    /// assert!(shape::cylinder(0.5, f64::NAN).is_err());
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn cylinder(radius: f64, height: f64) -> CoreResult<Part> {
        positive_finite("radius", radius)?;
        positive_finite("height", height)?;
        let h = Vector3::new(radius, height / 2.0, radius);
        Ok(Part {
            shape: Shape::new(Cylinder {
                center: Point3::origin(),
                radius,
                half_height: height / 2.0,
            }),
            bounds: BoundingBox3::new(Point3::origin() - h, Point3::origin() + h),
        })
    }

    /// A truncated cone (frustum), centered at the origin with its axis
    /// along Y: `radius_bottom` at `y = -height/2`, `radius_top` at
    /// `y = +height/2`. Either radius may be zero for a pointed tip, but
    /// not both.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `height` is not positive and
    /// finite, if either radius is negative or non-finite, or if both radii
    /// are zero.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let spike = shape::cone(1.0, 0.0, 2.0)?;
    /// assert!((spike.distance(0.0, 1.0, 0.0)).abs() < 1e-9); // tip
    /// assert!(spike.distance(0.0, 0.0, 0.0) < 0.0); // inside
    /// assert!(shape::cone(0.0, 0.0, 2.0).is_err());
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn cone(radius_bottom: f64, radius_top: f64, height: f64) -> CoreResult<Part> {
        for (name, r) in [("radius_bottom", radius_bottom), ("radius_top", radius_top)] {
            if !(r >= 0.0 && r.is_finite()) {
                return Err(CoreError::InvalidArgument {
                    argument: name,
                    reason: format!("must be non-negative and finite, got {r}"),
                });
            }
        }
        if radius_bottom == 0.0 && radius_top == 0.0 {
            return Err(CoreError::InvalidArgument {
                argument: "radius_bottom",
                reason: "radius_bottom and radius_top cannot both be zero; \
                         give at least one cap a positive radius"
                    .into(),
            });
        }
        positive_finite("height", height)?;
        let r = radius_bottom.max(radius_top);
        let h = Vector3::new(r, height / 2.0, r);
        Ok(Part {
            shape: Shape::new(Cone {
                center: Point3::origin(),
                half_height: height / 2.0,
                radius_bottom,
                radius_top,
            }),
            bounds: BoundingBox3::new(Point3::origin() - h, Point3::origin() + h),
        })
    }

    /// A torus centered at the origin, its ring in the XZ plane (axis along
    /// Y): `major_radius` from the center to the tube center, `minor_radius`
    /// of the tube itself.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if either radius is not positive and
    /// finite, or if `minor_radius >= major_radius` (self-intersecting).
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let ring = shape::torus(2.0, 0.5)?;
    /// assert!((ring.distance(2.5, 0.0, 0.0)).abs() < 1e-12); // outer equator
    /// assert!(ring.distance(0.0, 0.0, 0.0) > 0.0); // hole
    /// assert!(shape::torus(1.0, 1.0).is_err());
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn torus(major_radius: f64, minor_radius: f64) -> CoreResult<Part> {
        positive_finite("major_radius", major_radius)?;
        positive_finite("minor_radius", minor_radius)?;
        if minor_radius >= major_radius {
            return Err(CoreError::InvalidArgument {
                argument: "minor_radius",
                reason: format!(
                    "must be less than major_radius ({major_radius}) for a \
                     non-self-intersecting torus, got {minor_radius}"
                ),
            });
        }
        let reach = major_radius + minor_radius;
        let h = Vector3::new(reach, minor_radius, reach);
        Ok(Part {
            shape: Shape::new(Torus {
                center: Point3::origin(),
                major_radius,
                minor_radius,
            }),
            bounds: BoundingBox3::new(Point3::origin() - h, Point3::origin() + h),
        })
    }

    /// A capsule: a cylinder of the given `radius` from `start` to `end`
    /// with hemispherical caps. With `start == end` it degenerates to a
    /// sphere.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `radius` is not positive and
    /// finite or an endpoint has a non-finite coordinate.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    /// use opensolid_kernel::core::types::Point3;
    ///
    /// let pill = shape::capsule(
    ///     Point3::new(0.0, -1.0, 0.0),
    ///     Point3::new(0.0, 1.0, 0.0),
    ///     0.5,
    /// )?;
    /// assert!((pill.distance(0.5, 0.0, 0.0)).abs() < 1e-12); // side
    /// assert!((pill.distance(0.0, 1.5, 0.0)).abs() < 1e-12); // cap
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn capsule(start: Point3, end: Point3, radius: f64) -> CoreResult<Part> {
        finite_point("start", &start)?;
        finite_point("end", &end)?;
        positive_finite("radius", radius)?;
        let r = Vector3::new(radius, radius, radius);
        let ends = BoundingBox3::from_points([start, end]);
        Ok(Part {
            shape: Shape::new(Capsule { start, end, radius }),
            bounds: BoundingBox3::new(ends.min - r, ends.max + r),
        })
    }
}

impl Part {
    /// Translate by `(x, y, z)`. Primitives are created centered at the
    /// origin, so on a fresh primitive this places its center at that point.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if any coordinate is not finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let ball = shape::sphere(1.0)?.at(5.0, 0.0, 0.0)?;
    /// assert!((ball.distance(6.0, 0.0, 0.0)).abs() < 1e-12);
    /// assert!(ball.distance(0.0, 0.0, 0.0) > 0.0); // origin is now outside
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn at(self, x: f64, y: f64, z: f64) -> CoreResult<Part> {
        finite("x", x)?;
        finite("y", y)?;
        finite("z", z)?;
        let v = Vector3::new(x, y, z);
        Ok(Part {
            shape: Shape::new(self.shape.translated(v)),
            bounds: BoundingBox3::new(self.bounds.min + v, self.bounds.max + v),
        })
    }

    /// Rotate about the origin: `degrees` around `axis` (right-hand rule).
    /// The axis need not be unit length.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `axis` is zero or non-finite, or
    /// `degrees` is not finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    /// use opensolid_kernel::core::types::Vector3;
    ///
    /// // A slab reaching to x = ±2; after +90° about Z it reaches y = ±2.
    /// let slab = shape::box3(4.0, 2.0, 2.0)?.rotate(Vector3::z(), 90.0)?;
    /// assert!((slab.distance(0.0, 2.0, 0.0)).abs() < 1e-12);
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn rotate(self, axis: Vector3, degrees: f64) -> CoreResult<Part> {
        let norm = axis.norm();
        if !(norm > 0.0 && norm.is_finite()) {
            return Err(CoreError::InvalidArgument {
                argument: "axis",
                reason: format!("must be a nonzero finite vector, got {axis:?}"),
            });
        }
        finite("degrees", degrees)?;
        let axis_angle = axis / norm * degrees.to_radians();
        let rot = Transform3::rotation(axis_angle);
        Ok(Part {
            shape: Shape::new(self.shape.rotated(axis_angle)),
            bounds: BoundingBox3::from_points(corners(&self.bounds).map(|c| rot * c)),
        })
    }

    /// Rotate about the origin around the X axis. See [`Part::rotate`].
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `degrees` is not finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// // Y-axis cylinder tipped to lie along Z.
    /// let rod = shape::cylinder(0.5, 4.0)?.rotate_x(90.0)?;
    /// assert!((rod.distance(0.0, 0.0, 2.0)).abs() < 1e-12);
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn rotate_x(self, degrees: f64) -> CoreResult<Part> {
        self.rotate(Vector3::x(), degrees)
    }

    /// Rotate about the origin around the Y axis. See [`Part::rotate`].
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `degrees` is not finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// // A slab reaching to x = ±2; after +90° about Y it reaches z = ±2.
    /// let slab = shape::box3(4.0, 2.0, 2.0)?.rotate_y(90.0)?;
    /// assert!((slab.distance(0.0, 0.0, 2.0)).abs() < 1e-12);
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn rotate_y(self, degrees: f64) -> CoreResult<Part> {
        self.rotate(Vector3::y(), degrees)
    }

    /// Rotate about the origin around the Z axis. See [`Part::rotate`].
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `degrees` is not finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// // Y-axis cylinder laid down along X.
    /// let rod = shape::cylinder(0.5, 4.0)?.rotate_z(90.0)?;
    /// assert!((rod.distance(2.0, 0.0, 0.0)).abs() < 1e-12);
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn rotate_z(self, degrees: f64) -> CoreResult<Part> {
        self.rotate(Vector3::z(), degrees)
    }

    /// Scale uniformly about the origin by `factor`.
    ///
    /// Only uniform scaling is offered: non-uniform scale breaks the metric
    /// property of the distance field (see `opensolid_frep::UniformScale`).
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `factor` is not positive and finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let big = shape::sphere(1.0)?.scale(3.0)?;
    /// assert!((big.distance(3.0, 0.0, 0.0)).abs() < 1e-12);
    /// assert!(shape::sphere(1.0)?.scale(0.0).is_err());
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn scale(self, factor: f64) -> CoreResult<Part> {
        let scaled = self.shape.scaled(factor)?;
        Ok(Part {
            shape: Shape::new(scaled),
            bounds: BoundingBox3::new(
                Point3::from(self.bounds.min.coords * factor),
                Point3::from(self.bounds.max.coords * factor),
            ),
        })
    }

    /// Boolean union: material of either part.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let dumbbell = shape::sphere(1.0)?
    ///     .union(shape::sphere(1.0)?.at(3.0, 0.0, 0.0)?);
    /// assert!(dumbbell.distance(0.0, 0.0, 0.0) < 0.0);
    /// assert!(dumbbell.distance(3.0, 0.0, 0.0) < 0.0);
    /// assert!(dumbbell.distance(1.5, 0.0, 0.0) > 0.0); // gap between them
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn union(self, other: Part) -> Part {
        Part {
            bounds: self.bounds.union(&other.bounds),
            shape: self.shape.union(other.shape),
        }
    }

    /// Boolean intersection: material common to both parts.
    ///
    /// Intersecting disjoint parts yields an empty part; [`Part::mesh`]
    /// reports that as an error rather than returning an empty mesh.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let lens = shape::sphere(1.0)?
    ///     .intersect(shape::sphere(1.0)?.at(1.2, 0.0, 0.0)?);
    /// assert!(lens.distance(0.6, 0.0, 0.0) < 0.0); // shared middle
    /// assert!(lens.distance(-0.5, 0.0, 0.0) > 0.0); // only in the first
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn intersect(self, other: Part) -> Part {
        Part {
            bounds: self.bounds.intersection(&other.bounds),
            shape: self.shape.intersect(other.shape),
        }
    }

    /// Boolean subtraction: material of `self` not inside `tool`.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let block = shape::box3(4.0, 4.0, 4.0)?;
    /// let bored = block.subtract(shape::cylinder(1.0, 5.0)?);
    /// assert!(bored.distance(0.0, 0.0, 0.0) > 0.0); // inside the bore
    /// assert!(bored.distance(1.5, 0.0, 0.0) < 0.0); // remaining material
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn subtract(self, tool: Part) -> Part {
        Part {
            bounds: self.bounds,
            shape: self.shape.subtract(tool.shape),
        }
    }

    /// Smooth (blended) union: like [`Part::union`], but the junction is
    /// filleted over the given blend `radius` instead of leaving a crease.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `radius` is not positive and finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let ball = || shape::sphere(1.0);
    /// let sharp = ball()?.union(ball()?.at(1.6, 0.0, 0.0)?);
    /// let blended = ball()?.smooth_union(ball()?.at(1.6, 0.0, 0.0)?, 0.4)?;
    /// // The blend adds material at the junction beyond the sharp union.
    /// assert!(sharp.distance(0.8, 0.7, 0.0) > 0.0);
    /// assert!(blended.distance(0.8, 0.7, 0.0) < 0.0);
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn smooth_union(self, other: Part, radius: f64) -> CoreResult<Part> {
        positive_finite("radius", radius)?;
        Ok(Part {
            // The polynomial smooth min bulges outward at most radius/4
            // beyond the sharp union; dilate by radius/2 for headroom.
            bounds: self.bounds.union(&other.bounds).dilate(radius / 2.0),
            shape: self.shape.smooth_union(other.shape, radius),
        })
    }

    /// Round all edges and corners with fillets of the given `radius` by
    /// inflating the part (Minkowski sum with a sphere). Note this grows
    /// the part by `radius` in every direction; build the part undersized
    /// if the final dimensions matter.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `radius` is not positive and finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let rounded = shape::box3(2.0, 2.0, 2.0)?.smooth(0.2)?;
    /// // Faces moved out by the radius...
    /// assert!((rounded.distance(1.2, 0.0, 0.0)).abs() < 1e-12);
    /// // ...but the inflated sharp corner is outside: it got rounded off.
    /// assert!(rounded.distance(1.2, 1.2, 1.2) > 0.0);
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn smooth(self, radius: f64) -> CoreResult<Part> {
        let rounded = Rounded::new(self.shape, radius)?;
        Ok(Part {
            bounds: self.bounds.dilate(radius),
            shape: Shape::new(rounded),
        })
    }

    /// Hollow the part into a shell of the given wall `thickness`, centered
    /// on the original surface (extending `thickness / 2` to each side).
    ///
    /// Thin walls need a meshing grid fine enough to resolve them — see
    /// [`Part::mesh`] for the resolution rule of thumb.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `thickness` is not positive and
    /// finite.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let hollow = shape::sphere(1.0)?.shell(0.2)?;
    /// assert!(hollow.distance(0.0, 0.0, 0.0) > 0.0); // core removed
    /// assert!(hollow.distance(1.0, 0.0, 0.0) < 0.0); // wall material
    /// assert!((hollow.distance(1.1, 0.0, 0.0)).abs() < 1e-12); // outer skin
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn shell(self, thickness: f64) -> CoreResult<Part> {
        let shelled = Shell::new(self.shape, thickness)?;
        Ok(Part {
            bounds: self.bounds.dilate(thickness / 2.0),
            shape: Shape::new(shelled),
        })
    }

    /// Mesh the part into an indexed triangle mesh by dual contouring,
    /// with `resolution` grid cells along each axis of the part's bounding
    /// region (higher is finer; 32–128 is typical). The region is derived
    /// from the tracked bounds automatically.
    ///
    /// The grid must resolve the part's thinnest feature or the mesh will
    /// have holes: keep the cell size (roughly `1.2 × extent / resolution`)
    /// under half the thinnest wall. For example a [`Part::shell`] of
    /// thickness `t` on a part of extent `e` needs
    /// `resolution >= 2.4 * e / t` or so; verify with
    /// [`TriangleMesh::is_closed_manifold`] when in doubt.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `resolution` is below the minimum
    /// of 8; [`CoreError::Degenerate`] if the part is empty (for example an
    /// intersection of disjoint parts), since there is no surface to mesh.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let mesh = shape::sphere(1.0)?.mesh(24)?;
    /// assert!(mesh.is_closed_manifold());
    /// let area = mesh.total_area();
    /// let exact = 4.0 * std::f64::consts::PI;
    /// assert!((area - exact).abs() / exact < 0.1);
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn mesh(&self, resolution: usize) -> CoreResult<TriangleMesh> {
        if resolution < MIN_MESH_RESOLUTION {
            return Err(CoreError::InvalidArgument {
                argument: "resolution",
                reason: format!(
                    "must be at least {MIN_MESH_RESOLUTION} grid cells per \
                     axis, got {resolution}"
                ),
            });
        }
        if self.bounds.is_empty() {
            return Err(CoreError::Degenerate {
                context: "Part::mesh",
                reason: "the part is empty (for example an intersection of \
                         disjoint parts); there is no surface to mesh"
                    .into(),
            });
        }
        // Sample a cube around the part, not the tight box: dual contouring
        // stitches unreliably on strongly anisotropic cells (of-torus bead),
        // and cubic cells keep `resolution` meaning "cells across the
        // largest dimension".
        let e = self.bounds.extents();
        let extent = e.x.max(e.y).max(e.z);
        let margin = extent * 4.0 / resolution as f64;
        let half = Vector3::repeat(extent / 2.0 + margin);
        let center = self.bounds.center();
        Ok(mesh_sdf_indexed(
            &self.shape,
            &MeshOptions {
                bounds: BoundingBox3::new(center - half, center + half),
                resolution,
            },
        ))
    }

    /// Conservative axis-aligned bounds: guaranteed to contain the part,
    /// possibly with slack (booleans and blends are bounded, not measured).
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let b = shape::sphere(1.0)?.at(5.0, 0.0, 0.0)?.bounds();
    /// assert_eq!(b.min.x, 4.0);
    /// assert_eq!(b.max.x, 6.0);
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn bounds(&self) -> BoundingBox3 {
        self.bounds
    }

    /// Signed distance from the point `(x, y, z)` to the part's surface:
    /// negative inside, zero on the surface, positive outside. (Blended and
    /// combined parts may return a conservative approximation rather than
    /// the exact distance, but the sign is always meaningful.)
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    ///
    /// let ball = shape::sphere(1.0)?;
    /// assert!((ball.distance(0.0, 0.0, 0.0) + 1.0).abs() < 1e-12);
    /// assert!((ball.distance(2.0, 0.0, 0.0) - 1.0).abs() < 1e-12);
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn distance(&self, x: f64, y: f64, z: f64) -> f64 {
        self.shape.eval(&Point3::new(x, y, z))
    }

    /// Borrow the underlying F-Rep scene graph handle.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    /// use opensolid_kernel::frep::primitives::Sdf;
    /// use opensolid_kernel::core::types::Point3;
    ///
    /// let part = shape::sphere(1.0)?;
    /// assert!(part.shape().eval(&Point3::origin()) < 0.0);
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    /// Consume the part, returning the underlying [`Shape`] — the handoff
    /// point to lower-level APIs like `Session` model registration.
    ///
    /// ```
    /// use opensolid_kernel::builder::shape;
    /// use opensolid_kernel::{Model, Session};
    ///
    /// let mut session = Session::new();
    /// session.create(Model {
    ///     name: "ball".into(),
    ///     shape: shape::sphere(1.0)?.into_shape(),
    /// });
    /// # Ok::<(), opensolid_kernel::core::error::CoreError>(())
    /// ```
    pub fn into_shape(self) -> Shape {
        self.shape
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::massprops::mass_properties;

    fn assert_manifold(part: &Part, resolution: usize) -> TriangleMesh {
        let mesh = part.mesh(resolution).expect("meshing should succeed");
        assert!(!mesh.is_empty(), "mesh is empty");
        assert!(mesh.is_closed_manifold(), "mesh is not a closed manifold");
        mesh
    }

    #[test]
    fn every_primitive_meshes_to_a_closed_manifold() {
        let parts = [
            shape::sphere(1.0).unwrap(),
            shape::box3(2.0, 1.0, 1.5).unwrap(),
            shape::cylinder(0.7, 2.0).unwrap(),
            shape::cone(1.0, 0.4, 2.0).unwrap(),
            shape::torus(2.0, 0.5).unwrap(),
            shape::capsule(Point3::new(0.0, -1.0, 0.0), Point3::new(0.0, 1.0, 0.0), 0.5).unwrap(),
        ];
        for part in &parts {
            assert_manifold(part, 32);
        }
    }

    #[test]
    fn bead_example_chain_meshes_to_a_closed_manifold() {
        // The chain from the issue: sphere placed, unioned with a box,
        // smoothed, shelled, meshed.
        let part = shape::sphere(1.0)
            .unwrap()
            .at(0.0, 0.0, 0.0)
            .unwrap()
            .union(
                shape::box3(1.6, 1.6, 1.6)
                    .unwrap()
                    .at(0.9, 0.0, 0.0)
                    .unwrap(),
            )
            .smooth(0.2)
            .unwrap()
            .shell(0.1)
            .unwrap();
        // The 0.1 wall needs ~2 cells across it: extent ≈ 3.2, so res 64.
        assert_manifold(&part, 64);
    }

    #[test]
    fn booleans_mesh_to_closed_manifolds() {
        let block = || shape::box3(3.0, 3.0, 3.0).unwrap();
        let rod = || shape::cylinder(0.8, 4.0).unwrap();
        assert_manifold(&block().union(rod().at(1.0, 0.0, 0.0).unwrap()), 40);
        assert_manifold(&block().subtract(rod()), 40);
        assert_manifold(
            &shape::sphere(1.0)
                .unwrap()
                .intersect(shape::sphere(1.0).unwrap().at(1.0, 0.0, 0.0).unwrap()),
            40,
        );
        assert_manifold(
            &shape::sphere(1.0)
                .unwrap()
                .smooth_union(shape::sphere(1.0).unwrap().at(1.5, 0.0, 0.0).unwrap(), 0.4)
                .unwrap(),
            40,
        );
    }

    #[test]
    fn transformed_parts_mesh_within_tracked_bounds() {
        let part = shape::box3(4.0, 1.0, 1.0)
            .unwrap()
            .rotate_z(90.0)
            .unwrap()
            .at(10.0, 0.0, 0.0)
            .unwrap()
            .scale(0.5)
            .unwrap();
        let mesh = assert_manifold(&part, 32);
        let bbox = mesh.bounding_box().expect("non-empty mesh");
        let bounds = part.bounds();
        // The mesh must lie inside the tracked bounds (plus interpolation slop).
        for axis in 0..3 {
            assert!(bbox.min[axis] >= bounds.min[axis] - 1e-6, "axis {axis}");
            assert!(bbox.max[axis] <= bounds.max[axis] + 1e-6, "axis {axis}");
        }
        // Rotation + translation + scale land the box around x = 5.
        assert!((bounds.center().x - 5.0).abs() < 1e-12);
        // 90° about Z swaps the long axis from X to Y; scale halves it.
        assert!((bounds.extents().y - 2.0).abs() < 1e-12);
    }

    #[test]
    fn shelled_sphere_encloses_near_zero_volume() {
        // A thin shell's enclosed volume is the wall itself: far less than
        // the solid ball's. Volume via divergence theorem on the mesh.
        let solid = mass_properties(&shape::sphere(1.0).unwrap().mesh(48).unwrap())
            .expect("solid mesh props");
        let shelled_part = shape::sphere(1.0).unwrap().shell(0.1).unwrap();
        let shell_mesh = assert_manifold(&shelled_part, 48);
        let shell = mass_properties(&shell_mesh).expect("shell mesh props");
        assert!(
            shell.volume < solid.volume * 0.5,
            "shell volume {} vs solid {}",
            shell.volume,
            solid.volume
        );
    }

    #[test]
    fn constructors_reject_bad_arguments() {
        assert!(shape::sphere(0.0).is_err());
        assert!(shape::sphere(f64::NAN).is_err());
        assert!(shape::box3(1.0, -1.0, 1.0).is_err());
        assert!(shape::cylinder(1.0, f64::INFINITY).is_err());
        assert!(shape::cone(-1.0, 0.5, 1.0).is_err());
        assert!(shape::cone(0.0, 0.0, 1.0).is_err());
        assert!(shape::torus(1.0, 2.0).is_err());
        assert!(shape::capsule(Point3::new(f64::NAN, 0.0, 0.0), Point3::origin(), 1.0).is_err());

        let err = shape::sphere(-2.0).unwrap_err();
        assert!(
            matches!(
                &err,
                CoreError::InvalidArgument {
                    argument: "radius",
                    ..
                }
            ),
            "unexpected error: {err}"
        );
        assert!(err.to_string().contains("-2"), "missing value: {err}");
    }

    #[test]
    fn operations_reject_bad_arguments() {
        let part = || shape::sphere(1.0).unwrap();
        assert!(part().at(f64::NAN, 0.0, 0.0).is_err());
        assert!(part().rotate(Vector3::zeros(), 45.0).is_err());
        assert!(part().rotate(Vector3::x(), f64::NAN).is_err());
        assert!(part().rotate_z(f64::INFINITY).is_err());
        assert!(part().scale(-1.0).is_err());
        assert!(part().smooth(0.0).is_err());
        assert!(part().shell(f64::NAN).is_err());
        assert!(part().smooth_union(part(), -0.1).is_err());
    }

    #[test]
    fn mesh_rejects_low_resolution_and_empty_parts() {
        let part = shape::sphere(1.0).unwrap();
        let err = part.mesh(4).unwrap_err();
        assert!(
            matches!(
                &err,
                CoreError::InvalidArgument {
                    argument: "resolution",
                    ..
                }
            ),
            "unexpected error: {err}"
        );

        let empty = shape::sphere(1.0)
            .unwrap()
            .intersect(shape::sphere(1.0).unwrap().at(10.0, 0.0, 0.0).unwrap());
        let err = empty.mesh(32).unwrap_err();
        assert!(
            matches!(&err, CoreError::Degenerate { context, .. } if *context == "Part::mesh"),
            "unexpected error: {err}"
        );
        assert!(err.to_string().contains("empty"), "unteachable: {err}");
    }

    #[test]
    fn parts_are_cheaply_cloneable_and_send_sync() {
        fn assert_send_sync<T: Send + Sync>(_: &T) {}
        let a = shape::sphere(1.0).unwrap();
        let both = a.clone().union(a.clone());
        assert_send_sync(&both);
        assert_eq!(both.distance(0.0, 0.0, 0.0), a.distance(0.0, 0.0, 0.0));
    }
}
