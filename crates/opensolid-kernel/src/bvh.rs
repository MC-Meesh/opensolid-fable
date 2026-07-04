//! Binary bounding-volume hierarchy over items with axis-aligned boxes.
//!
//! The tree is representation-agnostic: an item is any `(BoundingBox3, Id)`
//! pair, so the same index serves F-Rep triangle soups and B-Rep faces.
//! Geometry stays with the caller — ray and nearest-point queries take a
//! callback that resolves the exact item test, while the tree only prunes
//! by bounding box.
//!
//! Construction uses binned surface-area-heuristic (SAH) splits on the
//! largest centroid axis, falling back to an index-median split when the
//! centroids do not separate (e.g. many identical boxes), so building
//! always terminates and never produces an empty child.

use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{BoundingBox3, Point3, Ray3};

/// Items per leaf below which splitting stops.
const LEAF_SIZE: usize = 4;
/// Number of SAH candidate bins along the split axis.
const SAH_BINS: usize = 16;
/// Relative cost of a traversal step vs. one item intersection in the SAH.
const TRAVERSAL_COST: f64 = 1.0;

struct Node {
    bounds: BoundingBox3,
    kind: NodeKind,
}

enum NodeKind {
    /// Range `start..start + count` into the reordered item array.
    Leaf {
        start: usize,
        count: usize,
    },
    Inner {
        left: usize,
        right: usize,
    },
}

/// A static BVH. Build once with [`Bvh::build`]; queries never mutate.
pub struct Bvh<Id> {
    nodes: Vec<Node>,
    items: Vec<(BoundingBox3, Id)>,
}

impl<Id> Bvh<Id> {
    /// Build a BVH over `items`. Items are reordered internally; empty
    /// input yields an empty tree for which every query returns nothing.
    /// Empty bounding boxes are permitted; they can never be hit or
    /// overlapped, but still occupy leaf slots.
    pub fn build(mut items: Vec<(BoundingBox3, Id)>) -> Self {
        let mut nodes = Vec::new();
        if !items.is_empty() {
            let len = items.len();
            build_node(&mut nodes, &mut items, 0, len);
        }
        Self { nodes, items }
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// First item (in traversal order, not ray order) for which `hit`
    /// returns a parameter `t >= 0`. Use for occlusion-style queries where
    /// any hit suffices; use [`ray_nearest`](Self::ray_nearest) for the
    /// closest one. `hit` receives the ray and the item id.
    pub fn ray_any<F>(&self, ray: &Ray3, mut hit: F) -> Option<&Id>
    where
        F: FnMut(&Ray3, &Id) -> Option<f64>,
    {
        if self.nodes.is_empty() {
            return None;
        }
        let mut stack = vec![0usize];
        while let Some(ni) = stack.pop() {
            let node = &self.nodes[ni];
            if node.bounds.ray_intersect(ray).is_none() {
                continue;
            }
            match node.kind {
                NodeKind::Leaf { start, count } => {
                    for (_, id) in &self.items[start..start + count] {
                        if hit(ray, id).is_some_and(|t| t >= 0.0) {
                            return Some(id);
                        }
                    }
                }
                NodeKind::Inner { left, right } => {
                    stack.push(left);
                    stack.push(right);
                }
            }
        }
        None
    }

    /// Item with the smallest hit parameter `t >= 0`, with that parameter.
    /// Nodes whose boxes cannot contain a closer hit than the current best
    /// are pruned.
    pub fn ray_nearest<F>(&self, ray: &Ray3, mut hit: F) -> Option<(f64, &Id)>
    where
        F: FnMut(&Ray3, &Id) -> Option<f64>,
    {
        if self.nodes.is_empty() {
            return None;
        }
        let mut best: Option<(f64, &Id)> = None;
        let mut stack = vec![0usize];
        while let Some(ni) = stack.pop() {
            let node = &self.nodes[ni];
            let Some((t_enter, _)) = node.bounds.ray_intersect(ray) else {
                continue;
            };
            // t_enter is negative when the origin is inside the box; the
            // box can still contain hits from t = 0 on.
            if best.is_some_and(|(bt, _)| t_enter.max(0.0) > bt) {
                continue;
            }
            match node.kind {
                NodeKind::Leaf { start, count } => {
                    for (_, id) in &self.items[start..start + count] {
                        if let Some(t) = hit(ray, id) {
                            if t >= 0.0 && best.is_none_or(|(bt, _)| t < bt) {
                                best = Some((t, id));
                            }
                        }
                    }
                }
                NodeKind::Inner { left, right } => {
                    stack.push(left);
                    stack.push(right);
                }
            }
        }
        best
    }

    /// All cross pairs `(a, b)` whose item boxes overlap, from a
    /// simultaneous descent of both trees — the box-box broad phase for
    /// clash detection. Pair order within the result is unspecified.
    pub fn overlap_pairs<'a, 'b, J>(&'a self, other: &'b Bvh<J>) -> Vec<(&'a Id, &'b J)> {
        let mut out = Vec::new();
        if self.nodes.is_empty() || other.nodes.is_empty() {
            return out;
        }
        let mut stack = vec![(0usize, 0usize)];
        while let Some((ai, bi)) = stack.pop() {
            let a = &self.nodes[ai];
            let b = &other.nodes[bi];
            if a.bounds.intersection(&b.bounds).is_empty() {
                continue;
            }
            match (&a.kind, &b.kind) {
                (
                    &NodeKind::Leaf {
                        start: sa,
                        count: ca,
                    },
                    &NodeKind::Leaf {
                        start: sb,
                        count: cb,
                    },
                ) => {
                    for (abox, aid) in &self.items[sa..sa + ca] {
                        for (bbox, bid) in &other.items[sb..sb + cb] {
                            if !abox.intersection(bbox).is_empty() {
                                out.push((aid, bid));
                            }
                        }
                    }
                }
                (&NodeKind::Inner { left, right }, _) => {
                    stack.push((left, bi));
                    stack.push((right, bi));
                }
                (_, &NodeKind::Inner { left, right }) => {
                    stack.push((ai, left));
                    stack.push((ai, right));
                }
            }
        }
        out
    }

    /// Item minimizing `distance(query, id)`, with that distance. The
    /// callback must return the exact (non-negative) distance from the
    /// query point to the item; the traversal prunes any subtree whose box
    /// is farther than the current best, descending nearer boxes first.
    pub fn nearest<F>(&self, query: &Point3, mut distance: F) -> Option<(f64, &Id)>
    where
        F: FnMut(&Point3, &Id) -> f64,
    {
        if self.nodes.is_empty() {
            return None;
        }
        let mut best: Option<(f64, &Id)> = None;
        let mut stack = vec![0usize];
        while let Some(ni) = stack.pop() {
            let node = &self.nodes[ni];
            if best.is_some_and(|(bd, _)| box_distance(query, &node.bounds) >= bd) {
                continue;
            }
            match node.kind {
                NodeKind::Leaf { start, count } => {
                    for (_, id) in &self.items[start..start + count] {
                        let d = distance(query, id);
                        if best.is_none_or(|(bd, _)| d < bd) {
                            best = Some((d, id));
                        }
                    }
                }
                NodeKind::Inner { left, right } => {
                    // Push the farther child first so the nearer one is
                    // explored first and tightens the bound sooner.
                    let dl = box_distance(query, &self.nodes[left].bounds);
                    let dr = box_distance(query, &self.nodes[right].bounds);
                    if dl <= dr {
                        stack.push(right);
                        stack.push(left);
                    } else {
                        stack.push(left);
                        stack.push(right);
                    }
                }
            }
        }
        best
    }
}

impl Bvh<usize> {
    /// BVH over a mesh's triangles, with the triangle index as the id.
    pub fn from_triangle_mesh(mesh: &TriangleMesh) -> Self {
        Self::build(
            mesh.indices
                .iter()
                .enumerate()
                .map(|(i, tri)| {
                    (
                        BoundingBox3::from_points(tri.iter().map(|&v| mesh.positions[v])),
                        i,
                    )
                })
                .collect(),
        )
    }
}

/// Distance from a point to a box (0 inside); +∞ for an empty box, so
/// empty boxes are never the nearest candidate.
fn box_distance(p: &Point3, bounds: &BoundingBox3) -> f64 {
    if bounds.is_empty() {
        return f64::INFINITY;
    }
    let mut d2 = 0.0;
    for axis in 0..3 {
        let v = p[axis].clamp(bounds.min[axis], bounds.max[axis]) - p[axis];
        d2 += v * v;
    }
    d2.sqrt()
}

/// Build the subtree over `items[start..end]` (non-empty), appending nodes
/// and returning the subtree root's index.
fn build_node<Id>(
    nodes: &mut Vec<Node>,
    items: &mut [(BoundingBox3, Id)],
    start: usize,
    end: usize,
) -> usize {
    let count = end - start;
    let bounds = items[start..end]
        .iter()
        .fold(BoundingBox3::EMPTY, |acc, (b, _)| acc.union(b));

    let idx = nodes.len();
    nodes.push(Node {
        bounds,
        kind: NodeKind::Leaf { start, count },
    });
    if count <= LEAF_SIZE {
        return idx;
    }

    let mid = match sah_split(items, start, end, &bounds) {
        Split::At(mid) => mid,
        Split::Leaf => return idx,
        // Centroids don't separate (identical boxes, coplanar stacks):
        // halve by index so the tree still ends in bounded leaves.
        Split::Degenerate => start + count / 2,
    };

    let left = build_node(nodes, items, start, mid);
    let right = build_node(nodes, items, mid, end);
    nodes[idx].kind = NodeKind::Inner { left, right };
    idx
}

enum Split {
    /// Partition point: left = `start..mid`, right = `mid..end`.
    At(usize),
    /// SAH says intersecting everything here is cheaper than splitting.
    Leaf,
    /// No axis separates the centroids.
    Degenerate,
}

/// Binned SAH split of `items[start..end]` on the largest centroid axis.
/// On success the range is partitioned in place around the returned mid.
fn sah_split<Id>(
    items: &mut [(BoundingBox3, Id)],
    start: usize,
    end: usize,
    bounds: &BoundingBox3,
) -> Split {
    let count = end - start;
    let centroid_bounds = items[start..end]
        .iter()
        .fold(BoundingBox3::EMPTY, |acc, (b, _)| {
            acc.union(&BoundingBox3::new(b.center(), b.center()))
        });
    let extents = centroid_bounds.extents();
    let axis = (0..3)
        .max_by(|&a, &b| extents[a].total_cmp(&extents[b]))
        .unwrap();
    if extents[axis] <= 0.0 || extents[axis].is_nan() {
        return Split::Degenerate;
    }

    let lo = centroid_bounds.min[axis];
    let scale = SAH_BINS as f64 / extents[axis];
    let bin_of = |b: &BoundingBox3| (((b.center()[axis] - lo) * scale) as usize).min(SAH_BINS - 1);

    let mut bin_bounds = [BoundingBox3::EMPTY; SAH_BINS];
    let mut bin_counts = [0usize; SAH_BINS];
    for (b, _) in &items[start..end] {
        let bin = bin_of(b);
        bin_bounds[bin] = bin_bounds[bin].union(b);
        bin_counts[bin] += 1;
    }

    // Cost of splitting after bin k: SAH with the parent area normalizing.
    let mut best: Option<(f64, usize)> = None;
    for k in 0..SAH_BINS - 1 {
        let (mut lb, mut ln) = (BoundingBox3::EMPTY, 0usize);
        for i in 0..=k {
            lb = lb.union(&bin_bounds[i]);
            ln += bin_counts[i];
        }
        let (mut rb, mut rn) = (BoundingBox3::EMPTY, 0usize);
        for i in k + 1..SAH_BINS {
            rb = rb.union(&bin_bounds[i]);
            rn += bin_counts[i];
        }
        if ln == 0 || rn == 0 {
            continue;
        }
        let cost = TRAVERSAL_COST
            + (lb.surface_area() * ln as f64 + rb.surface_area() * rn as f64)
                / bounds.surface_area().max(f64::MIN_POSITIVE);
        if best.is_none_or(|(bc, _)| cost < bc) {
            best = Some((cost, k));
        }
    }

    let Some((best_cost, best_bin)) = best else {
        // Every centroid landed in one bin despite a positive extent
        // (possible with extreme coordinate ratios).
        return Split::Degenerate;
    };
    if best_cost >= count as f64 {
        return Split::Leaf;
    }

    // Partition in place: bins <= best_bin to the left.
    let mut mid = start;
    for i in start..end {
        if bin_of(&items[i].0) <= best_bin {
            items.swap(i, mid);
            mid += 1;
        }
    }
    debug_assert!(mid > start && mid < end);
    Split::At(mid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_core::types::Vector3;

    /// Deterministic LCG so test data is reproducible without a rand dep.
    struct Lcg(u64);

    impl Lcg {
        fn next_f64(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }

        fn range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + self.next_f64() * (hi - lo)
        }

        fn point(&mut self, half: f64) -> Point3 {
            Point3::new(
                self.range(-half, half),
                self.range(-half, half),
                self.range(-half, half),
            )
        }
    }

    type Tri = [Point3; 3];

    /// Möller–Trumbore ray/triangle intersection; `t >= 0` on hit.
    fn ray_triangle(ray: &Ray3, tri: &Tri) -> Option<f64> {
        let e1 = tri[1] - tri[0];
        let e2 = tri[2] - tri[0];
        let pvec = ray.direction.cross(&e2);
        let det = e1.dot(&pvec);
        if det.abs() < 1e-14 {
            return None;
        }
        let inv_det = 1.0 / det;
        let tvec = ray.origin - tri[0];
        let u = tvec.dot(&pvec) * inv_det;
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let qvec = tvec.cross(&e1);
        let v = ray.direction.dot(&qvec) * inv_det;
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let t = e2.dot(&qvec) * inv_det;
        (t >= 0.0).then_some(t)
    }

    fn tri_bbox(tri: &Tri) -> BoundingBox3 {
        BoundingBox3::from_points(tri.iter().copied())
    }

    /// `n` random triangles: a random anchor with two bounded random edges,
    /// so sizes vary but stay BVH-friendly.
    fn soup(rng: &mut Lcg, n: usize, half: f64) -> Vec<Tri> {
        (0..n)
            .map(|_| {
                let a = rng.point(half);
                let e1 = rng.point(0.4).coords;
                let e2 = rng.point(0.4).coords;
                [a, a + e1, a + e2]
            })
            .collect()
    }

    fn tri_bvh(tris: &[Tri]) -> Bvh<usize> {
        Bvh::build(
            tris.iter()
                .enumerate()
                .map(|(i, t)| (tri_bbox(t), i))
                .collect(),
        )
    }

    fn random_ray(rng: &mut Lcg) -> Ray3 {
        let origin = rng.point(3.0);
        let mut dir = rng.point(1.0).coords;
        if dir.norm() < 1e-9 {
            dir = Vector3::new(1.0, 0.0, 0.0);
        }
        Ray3::new(origin, dir)
    }

    #[test]
    fn ray_nearest_matches_brute_force_on_random_soup() {
        let mut rng = Lcg(42);
        let tris = soup(&mut rng, 300, 2.0);
        let bvh = tri_bvh(&tris);
        let mut hits = 0;
        for _ in 0..200 {
            // Aim from outside the soup at a point inside it so a healthy
            // fraction of rays actually hit triangles.
            let origin = rng.point(1.0) + Vector3::new(0.0, 0.0, 6.0);
            let target = rng.point(1.5);
            let ray = Ray3::new(origin, target - origin);
            let brute = tris
                .iter()
                .enumerate()
                .filter_map(|(i, t)| ray_triangle(&ray, t).map(|t| (t, i)))
                .min_by(|a, b| a.0.total_cmp(&b.0));
            let bvh_hit = bvh
                .ray_nearest(&ray, |r, &i| ray_triangle(r, &tris[i]))
                .map(|(t, &i)| (t, i));
            match (brute, bvh_hit) {
                (None, None) => {}
                (Some((bt, _)), Some((t, _))) => {
                    // Ids may differ only on exact-t ties; the parameter
                    // must agree exactly (identical arithmetic both sides).
                    assert_eq!(bt, t, "ray {ray:?}");
                    hits += 1;
                }
                (b, v) => panic!("brute {b:?} vs bvh {v:?} for ray {ray:?}"),
            }
        }
        // The comparison must exercise real hits, not vacuous misses.
        assert!(hits > 50, "only {hits} rays hit the soup");
    }

    #[test]
    fn ray_any_agrees_with_brute_force_existence() {
        let mut rng = Lcg(7);
        let tris = soup(&mut rng, 200, 2.0);
        let bvh = tri_bvh(&tris);
        for _ in 0..200 {
            let ray = random_ray(&mut rng);
            let brute = tris.iter().any(|t| ray_triangle(&ray, t).is_some());
            let any = bvh.ray_any(&ray, |r, &i| ray_triangle(r, &tris[i]));
            assert_eq!(brute, any.is_some(), "ray {ray:?}");
            // Any reported item must actually be hit.
            if let Some(&i) = any {
                assert!(ray_triangle(&ray, &tris[i]).is_some());
            }
        }
    }

    #[test]
    fn overlap_pairs_match_brute_force() {
        let mut rng = Lcg(1234);
        let a = soup(&mut rng, 80, 1.5);
        let b = soup(&mut rng, 60, 1.5);
        let bvh_a = tri_bvh(&a);
        let bvh_b = tri_bvh(&b);

        let mut brute: Vec<(usize, usize)> = Vec::new();
        for (i, ta) in a.iter().enumerate() {
            for (j, tb) in b.iter().enumerate() {
                if !tri_bbox(ta).intersection(&tri_bbox(tb)).is_empty() {
                    brute.push((i, j));
                }
            }
        }
        let mut pairs: Vec<(usize, usize)> = bvh_a
            .overlap_pairs(&bvh_b)
            .into_iter()
            .map(|(&i, &j)| (i, j))
            .collect();
        brute.sort_unstable();
        pairs.sort_unstable();
        assert!(!brute.is_empty(), "test data produced no overlaps");
        assert_eq!(brute, pairs);
    }

    #[test]
    fn nearest_point_hook_matches_brute_force() {
        let mut rng = Lcg(99);
        let points: Vec<Point3> = (0..300).map(|_| rng.point(3.0)).collect();
        let bvh = Bvh::build(
            points
                .iter()
                .enumerate()
                .map(|(i, p)| (BoundingBox3::from_points([*p]), i))
                .collect(),
        );
        for _ in 0..100 {
            let q = rng.point(4.0);
            let brute = points
                .iter()
                .enumerate()
                .map(|(i, p)| ((p - q).norm(), i))
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .unwrap();
            let (d, &i) = bvh.nearest(&q, |q, &i| (points[i] - q).norm()).unwrap();
            assert_eq!(brute.0, d, "query {q:?}");
            assert_eq!(brute.1, i, "query {q:?}");
        }
    }

    #[test]
    fn single_item_tree() {
        let tri: Tri = [
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        ];
        let bvh = Bvh::build(vec![(tri_bbox(&tri), 0usize)]);
        assert_eq!(bvh.len(), 1);

        let hit_ray = Ray3::new(Point3::new(0.2, 0.2, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let miss_ray = Ray3::new(Point3::new(5.0, 5.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let hit = |r: &Ray3, i: &usize| {
            assert_eq!(*i, 0);
            ray_triangle(r, &tri)
        };
        assert_eq!(bvh.ray_nearest(&hit_ray, hit).map(|(t, _)| t), Some(1.0));
        assert!(bvh.ray_nearest(&miss_ray, hit).is_none());
        assert!(bvh.ray_any(&hit_ray, hit).is_some());
        assert!(bvh.ray_any(&miss_ray, hit).is_none());

        let (d, _) = bvh
            .nearest(&Point3::new(0.0, 0.0, 3.0), |q, _| {
                (q - Point3::new(0.0, 0.0, 1.0)).norm()
            })
            .unwrap();
        assert_eq!(d, 2.0);
    }

    #[test]
    fn coplanar_soup_builds_and_queries() {
        // All triangles in the z = 0 plane: every box has zero z-extent and
        // the SAH centroid range on z is empty.
        let mut rng = Lcg(5);
        let tris: Vec<Tri> = (0..120)
            .map(|_| {
                let a = Point3::new(rng.range(-2.0, 2.0), rng.range(-2.0, 2.0), 0.0);
                let e1 = Vector3::new(rng.range(-0.3, 0.3), rng.range(-0.3, 0.3), 0.0);
                let e2 = Vector3::new(rng.range(-0.3, 0.3), rng.range(-0.3, 0.3), 0.0);
                [a, a + e1, a + e2]
            })
            .collect();
        let bvh = tri_bvh(&tris);
        for _ in 0..100 {
            // Rays through the plane from above.
            let ray = Ray3::new(
                Point3::new(rng.range(-2.0, 2.0), rng.range(-2.0, 2.0), 1.0),
                Vector3::new(0.0, 0.0, -1.0),
            );
            let brute = tris.iter().any(|t| ray_triangle(&ray, t).is_some());
            assert_eq!(
                brute,
                bvh.ray_any(&ray, |r, &i| ray_triangle(r, &tris[i]))
                    .is_some(),
                "ray {ray:?}"
            );
        }
    }

    #[test]
    fn identical_boxes_terminate_via_median_fallback() {
        // 100 items with the same box: no axis separates the centroids, so
        // only the index-median fallback keeps the build from recursing
        // forever.
        let bbox = BoundingBox3::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
        let bvh = Bvh::build((0..100).map(|i| (bbox, i)).collect());
        assert_eq!(bvh.len(), 100);
        let ray = Ray3::new(Point3::new(0.5, 0.5, -1.0), Vector3::new(0.0, 0.0, 1.0));
        // All 100 share the box; any-hit must find one of them.
        assert!(bvh.ray_any(&ray, |_, _| Some(1.0)).is_some());
    }

    #[test]
    fn empty_tree_returns_nothing() {
        let bvh: Bvh<usize> = Bvh::build(Vec::new());
        assert!(bvh.is_empty());
        let ray = Ray3::new(Point3::origin(), Vector3::new(1.0, 0.0, 0.0));
        assert!(bvh.ray_any(&ray, |_, _| Some(0.0)).is_none());
        assert!(bvh.ray_nearest(&ray, |_, _| Some(0.0)).is_none());
        assert!(bvh.nearest(&Point3::origin(), |_, _| 0.0).is_none());
        assert!(bvh.overlap_pairs(&bvh).is_empty());
        let single = Bvh::build(vec![(
            BoundingBox3::from_points([Point3::origin()]),
            0usize,
        )]);
        assert!(bvh.overlap_pairs(&single).is_empty());
        assert!(single.overlap_pairs(&bvh).is_empty());
    }

    #[test]
    fn from_triangle_mesh_indexes_triangles() {
        let mesh = TriangleMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 5.0),
                Point3::new(1.0, 0.0, 5.0),
                Point3::new(0.0, 1.0, 5.0),
            ],
            normals: Vec::new(),
            indices: vec![[0, 1, 2], [3, 4, 5]],
        };
        let bvh = Bvh::from_triangle_mesh(&mesh);
        assert_eq!(bvh.len(), 2);
        let ray = Ray3::new(Point3::new(0.2, 0.2, -1.0), Vector3::new(0.0, 0.0, 1.0));
        let tri_of = |i: usize| -> Tri {
            let [a, b, c] = mesh.indices[i];
            [mesh.positions[a], mesh.positions[b], mesh.positions[c]]
        };
        let (t, &id) = bvh
            .ray_nearest(&ray, |r, &i| ray_triangle(r, &tri_of(i)))
            .unwrap();
        assert_eq!(id, 0, "nearest must be the z=0 triangle");
        assert_eq!(t, 1.0);
    }
}
