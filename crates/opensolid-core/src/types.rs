use nalgebra as na;

pub type Point3 = na::Point3<f64>;
pub type Vector3 = na::Vector3<f64>;
pub type Transform3 = na::Isometry3<f64>;

/// A ray in 3D: all points `origin + t * direction` for `t >= 0`.
///
/// `direction` need not be unit length; intersection parameters are then in
/// units of `|direction|` rather than distance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray3 {
    pub origin: Point3,
    pub direction: Vector3,
}

impl Ray3 {
    pub fn new(origin: Point3, direction: Vector3) -> Self {
        Self { origin, direction }
    }

    /// The point at parameter `t`.
    pub fn at(&self, t: f64) -> Point3 {
        self.origin + self.direction * t
    }
}

/// Axis-aligned bounding box.
///
/// A box is *empty* when `min > max` on any axis; [`BoundingBox3::EMPTY`]
/// (min = +∞, max = −∞) is the canonical empty box and the identity for
/// [`union`](BoundingBox3::union). Queries treat boundaries as inclusive.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBox3 {
    pub min: Point3,
    pub max: Point3,
}

impl BoundingBox3 {
    /// The canonical empty box: unions as an identity, intersects to empty.
    pub const EMPTY: Self = Self {
        min: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        max: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
    };

    pub fn new(min: Point3, max: Point3) -> Self {
        Self { min, max }
    }

    /// The smallest box containing all `points`; [`EMPTY`](Self::EMPTY) for
    /// an empty iterator. A single point yields a degenerate (zero-volume,
    /// non-empty) box.
    pub fn from_points<I: IntoIterator<Item = Point3>>(points: I) -> Self {
        points.into_iter().fold(Self::EMPTY, |bbox, p| Self {
            min: Point3::new(
                bbox.min.x.min(p.x),
                bbox.min.y.min(p.y),
                bbox.min.z.min(p.z),
            ),
            max: Point3::new(
                bbox.max.x.max(p.x),
                bbox.max.y.max(p.y),
                bbox.max.z.max(p.z),
            ),
        })
    }

    /// True if the box contains no points (inverted on some axis).
    pub fn is_empty(&self) -> bool {
        !(self.min.x <= self.max.x && self.min.y <= self.max.y && self.min.z <= self.max.z)
    }

    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: Point3::new(
                self.min.x.min(other.min.x),
                self.min.y.min(other.min.y),
                self.min.z.min(other.min.z),
            ),
            max: Point3::new(
                self.max.x.max(other.max.x),
                self.max.y.max(other.max.y),
                self.max.z.max(other.max.z),
            ),
        }
    }

    /// The overlap of two boxes; empty (check [`is_empty`](Self::is_empty))
    /// when they are disjoint.
    pub fn intersection(&self, other: &Self) -> Self {
        Self {
            min: Point3::new(
                self.min.x.max(other.min.x),
                self.min.y.max(other.min.y),
                self.min.z.max(other.min.z),
            ),
            max: Point3::new(
                self.max.x.min(other.max.x),
                self.max.y.min(other.max.y),
                self.max.z.min(other.max.z),
            ),
        }
    }

    /// Expand (or shrink, for negative `margin`) the box by `margin` on every
    /// side. Shrinking may produce an empty box.
    pub fn dilate(&self, margin: f64) -> Self {
        let m = Vector3::new(margin, margin, margin);
        Self {
            min: self.min - m,
            max: self.max + m,
        }
    }

    /// Center of the box. Meaningless for empty boxes.
    pub fn center(&self) -> Point3 {
        na::center(&self.min, &self.max)
    }

    /// Full size of the box on each axis (`max - min`). Meaningless for
    /// empty boxes.
    pub fn extents(&self) -> Vector3 {
        self.max - self.min
    }

    /// Total surface area (`2·(dx·dy + dy·dz + dz·dx)`); 0 for empty boxes.
    /// The BVH surface-area heuristic uses this.
    pub fn surface_area(&self) -> f64 {
        if self.is_empty() {
            return 0.0;
        }
        let e = self.extents();
        2.0 * (e.x * e.y + e.y * e.z + e.z * e.x)
    }

    pub fn contains(&self, point: &Point3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Slab-method ray/box intersection.
    ///
    /// Returns the parameter interval `(t_enter, t_exit)` over which the ray
    /// is inside the box, or `None` if the ray misses it for every `t >= 0`.
    /// `t_enter` is negative when the ray origin is inside the box.
    /// Boundaries are inclusive: grazing a face counts as a hit.
    ///
    /// Axis-parallel rays are handled through IEEE semantics: a zero
    /// direction component gives ±∞ slab parameters (never constraining when
    /// the origin is inside that slab, always rejecting when outside), and
    /// the NaN from `0 · ∞` at exact slab boundaries is ignored by
    /// `f64::min`/`f64::max`.
    pub fn ray_intersect(&self, ray: &Ray3) -> Option<(f64, f64)> {
        let mut t_enter = f64::NEG_INFINITY;
        let mut t_exit = f64::INFINITY;
        for axis in 0..3 {
            let inv = 1.0 / ray.direction[axis];
            let mut t0 = (self.min[axis] - ray.origin[axis]) * inv;
            let mut t1 = (self.max[axis] - ray.origin[axis]) * inv;
            if inv < 0.0 {
                std::mem::swap(&mut t0, &mut t1);
            }
            // f64::max/min ignore NaN operands, so a NaN slab parameter
            // (origin exactly on a face of a parallel slab) never poisons
            // the interval.
            t_enter = t_enter.max(t0);
            t_exit = t_exit.min(t1);
        }
        if t_enter <= t_exit && t_exit >= 0.0 {
            Some((t_enter, t_exit))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_box() -> BoundingBox3 {
        BoundingBox3::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0))
    }

    #[test]
    fn ray_at_parameterizes_along_direction() {
        let ray = Ray3::new(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 2.0));
        assert_eq!(ray.at(0.0), Point3::new(1.0, 2.0, 3.0));
        assert_eq!(ray.at(1.5), Point3::new(1.0, 2.0, 6.0));
    }

    #[test]
    fn axis_parallel_ray_hits_front_and_back_faces() {
        let ray = Ray3::new(Point3::new(-1.0, 0.5, 0.5), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(unit_box().ray_intersect(&ray), Some((1.0, 2.0)));

        let ray_y = Ray3::new(Point3::new(0.5, -2.0, 0.5), Vector3::new(0.0, 1.0, 0.0));
        assert_eq!(unit_box().ray_intersect(&ray_y), Some((2.0, 3.0)));
    }

    #[test]
    fn axis_parallel_ray_outside_slab_misses() {
        let ray = Ray3::new(Point3::new(-1.0, 2.0, 0.5), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(unit_box().ray_intersect(&ray), None);
    }

    #[test]
    fn diagonal_ray_hits_corner_to_corner() {
        let ray = Ray3::new(Point3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
        let (t_enter, t_exit) = unit_box().ray_intersect(&ray).unwrap();
        assert_eq!((t_enter, t_exit), (1.0, 2.0));
        assert_eq!(ray.at(t_enter), Point3::new(0.0, 0.0, 0.0));
        assert_eq!(ray.at(t_exit), Point3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn ray_pointing_away_misses() {
        let ray = Ray3::new(Point3::new(2.0, 0.5, 0.5), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(unit_box().ray_intersect(&ray), None);
        let ray_back = Ray3::new(Point3::new(-1.0, 0.5, 0.5), Vector3::new(-1.0, 0.0, 0.0));
        assert_eq!(unit_box().ray_intersect(&ray_back), None);
    }

    #[test]
    fn origin_inside_gives_negative_entry() {
        let ray = Ray3::new(Point3::new(0.5, 0.5, 0.5), Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(unit_box().ray_intersect(&ray), Some((-0.5, 0.5)));
    }

    #[test]
    fn grazing_ray_on_face_boundary_hits() {
        // Origin exactly on the y = 1 face plane with dir.y = 0: the slab
        // computation produces 0·∞ = NaN, which must be ignored, and the
        // inclusive boundary counts as a hit.
        let ray = Ray3::new(Point3::new(-1.0, 1.0, 0.5), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(unit_box().ray_intersect(&ray), Some((1.0, 2.0)));
    }

    #[test]
    fn unnormalized_direction_scales_parameters() {
        let ray = Ray3::new(Point3::new(-1.0, 0.5, 0.5), Vector3::new(2.0, 0.0, 0.0));
        assert_eq!(unit_box().ray_intersect(&ray), Some((0.5, 1.0)));
    }

    #[test]
    fn ray_never_hits_empty_box() {
        let ray = Ray3::new(Point3::new(-1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(BoundingBox3::EMPTY.ray_intersect(&ray), None);
    }

    #[test]
    fn empty_box_semantics() {
        assert!(BoundingBox3::EMPTY.is_empty());
        assert!(!unit_box().is_empty());
        assert_eq!(BoundingBox3::EMPTY.surface_area(), 0.0);
        assert!(!BoundingBox3::EMPTY.contains(&Point3::origin()));

        // EMPTY is the union identity.
        assert_eq!(BoundingBox3::EMPTY.union(&unit_box()), unit_box());
        assert_eq!(unit_box().union(&BoundingBox3::EMPTY), unit_box());

        // Disjoint boxes intersect to an empty box.
        let far = BoundingBox3::new(Point3::new(5.0, 5.0, 5.0), Point3::new(6.0, 6.0, 6.0));
        assert!(unit_box().intersection(&far).is_empty());
        assert!(BoundingBox3::EMPTY.intersection(&unit_box()).is_empty());
    }

    #[test]
    fn from_points_builds_tight_box() {
        let bbox = BoundingBox3::from_points([
            Point3::new(1.0, -2.0, 0.5),
            Point3::new(-3.0, 4.0, 0.0),
            Point3::new(0.0, 0.0, -1.0),
        ]);
        assert_eq!(bbox.min, Point3::new(-3.0, -2.0, -1.0));
        assert_eq!(bbox.max, Point3::new(1.0, 4.0, 0.5));

        assert!(BoundingBox3::from_points([]).is_empty());

        // A single point is a degenerate but non-empty box.
        let p = Point3::new(1.0, 2.0, 3.0);
        let single = BoundingBox3::from_points([p]);
        assert!(!single.is_empty());
        assert!(single.contains(&p));
        assert_eq!(single.surface_area(), 0.0);
    }

    #[test]
    fn intersection_of_overlapping_boxes() {
        let a = BoundingBox3::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
        let b = BoundingBox3::new(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0));
        let overlap = a.intersection(&b);
        assert_eq!(overlap.min, Point3::new(1.0, 1.0, 1.0));
        assert_eq!(overlap.max, Point3::new(2.0, 2.0, 2.0));
    }

    #[test]
    fn dilate_grows_and_shrinks() {
        let grown = unit_box().dilate(0.5);
        assert_eq!(grown.min, Point3::new(-0.5, -0.5, -0.5));
        assert_eq!(grown.max, Point3::new(1.5, 1.5, 1.5));

        let shrunk = unit_box().dilate(-0.25);
        assert_eq!(shrunk.min, Point3::new(0.25, 0.25, 0.25));
        assert_eq!(shrunk.max, Point3::new(0.75, 0.75, 0.75));

        // Shrinking past the center empties the box.
        assert!(unit_box().dilate(-0.6).is_empty());
    }

    #[test]
    fn center_extents_surface_area() {
        let bbox = BoundingBox3::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 4.0));
        assert_eq!(bbox.center(), Point3::new(0.5, 1.0, 2.0));
        assert_eq!(bbox.extents(), Vector3::new(1.0, 2.0, 4.0));
        // 2·(1·2 + 2·4 + 4·1) = 28.
        assert_eq!(bbox.surface_area(), 28.0);
    }
}
