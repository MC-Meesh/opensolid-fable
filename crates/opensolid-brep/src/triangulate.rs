//! Ear-clipping triangulation of a single simple planar polygon.
//!
//! Planar caps (extrusion end faces, planar B-Rep faces) used to be
//! fan-triangulated — from a centroid or the loop's first vertex — which
//! is only correct when the polygon is star-shaped from that apex. For a
//! concave outline (U/S/C shapes) a fan spills across the concavity and
//! emits overlapping triangles of mixed winding: the signed volume still
//! cancels to the right value and every edge is still used twice, so the
//! mesh even passes [`is_closed_manifold`], but it self-overlaps and
//! poisons downstream consumers (SDF conversion, rendering) (of-6dw).
//!
//! [`ear_clip`] is O(n²) ear clipping — the same core loop the boolean
//! tessellator runs (`boolean::triangulate_mesh_face`) but specialized to
//! one loop with no holes. It triangulates concave polygons correctly.
//!
//! [`is_closed_manifold`]: opensolid_core::mesh::TriangleMesh::is_closed_manifold

/// Triangulate a simple polygon given by its vertices in the plane, in
/// order (either winding, no holes, no self-intersections). Returns
/// triangles as index triples into `uv`, each wound counterclockwise
/// (positive signed area in the `uv` plane) regardless of the input
/// winding. Returns an empty vector for fewer than three vertices.
///
/// Callers projecting a 3D planar loop onto a plane basis `(e_u, e_v)`
/// with `e_u × e_v = n` get triangles wound counterclockwise about `n`,
/// i.e. facing along `+n`; flip the triple to face the other way.
///
/// Robust to the long **collinear vertex runs** that `sdf_to_brep` recovery
/// leaves along straight facet edges (of-6sq): no near-degenerate sliver is
/// emitted (a corner is deferred until it clips into a positive-area
/// triangle), and the containment test carries a tolerance so sub-nanometre
/// noise cannot make a run-spanning ear look empty — clipping such a false
/// ear would strand the run as an untriangulated collinear remnant.
pub(crate) fn ear_clip(uv: &[(f64, f64)]) -> Vec<[usize; 3]> {
    let n = uv.len();
    if n < 3 {
        return Vec::new();
    }

    // Ear clipping needs a counterclockwise loop so a positive corner
    // cross product marks a convex (candidate-ear) corner. Detect the
    // input winding from the signed area and walk the indices in reverse
    // when it runs clockwise; the emitted triples then always come out
    // counterclockwise, independent of how the caller wound the loop.
    let mut area2 = 0.0;
    for i in 0..n {
        let a = uv[i];
        let b = uv[(i + 1) % n];
        area2 += a.0 * b.1 - b.0 * a.1;
    }
    let mut idx: Vec<usize> = (0..n).collect();
    if area2 < 0.0 {
        idx.reverse();
    }

    // Containment tolerance in twice-area units, scaled to the polygon. A
    // boundary vertex sitting on a collinear run must count as *inside* any
    // ear whose diagonal skims that run; otherwise sub-nanometre dual-
    // contouring noise lets a run-spanning ear look empty and get clipped,
    // stranding the run (of-6sq). `1e-9 · extent²` is a ~1e-9-relative
    // perpendicular slack — far above the noise, far below any real feature.
    let extent = uv
        .iter()
        .fold(0.0_f64, |m, &(x, y)| m.max(x.abs()).max(y.abs()));
    let eps = (extent * extent).max(1.0) * 1e-9;

    let mut tris: Vec<[usize; 3]> = Vec::with_capacity(n - 2);
    // n distinct vertices remove at most n-2 ears; the guard is a safety
    // net for pathological (self-touching) input, not the normal path.
    let mut guard = 0usize;
    while idx.len() > 3 {
        guard += 1;
        if guard > 100_000 {
            break;
        }
        let m = idx.len();
        let mut clipped = false;
        for i in 0..m {
            let (ia, ib, ic) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
            let (a, b, c) = (uv[ia], uv[ib], uv[ic]);
            if !is_clippable_ear(a, b, c, &idx, uv, ia, ib, ic, eps) {
                continue;
            }
            tris.push([ia, ib, ic]);
            idx.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            // Fallback: clip the least-reflex corner to guarantee progress
            // on nearly-degenerate polygons (mirrors the boolean clipper).
            let m = idx.len();
            let mut best = (f64::NEG_INFINITY, 0usize);
            for i in 0..m {
                let (a, b, c) = (uv[idx[(i + m - 1) % m]], uv[idx[i]], uv[idx[(i + 1) % m]]);
                let cross = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
                if cross > best.0 {
                    best = (cross, i);
                }
            }
            let i = best.1;
            let m = idx.len();
            tris.push([idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]]);
            idx.remove(i);
        }
    }
    if idx.len() == 3 {
        tris.push([idx[0], idx[1], idx[2]]);
    }
    tris
}

/// Whether corner `b` (with neighbours `a`, `c`) is a clippable ear:
/// cleanly convex — not reflex, and not so thin it would emit a sliver the
/// mesh→SDF gate rejects (`2·area > 1e-12·longest²`, applied here with
/// margin) — and enclosing no other polygon vertex. A deferred thin/collinear
/// corner clips later, once a neighbouring ear has made it non-collinear.
#[allow(clippy::too_many_arguments)]
fn is_clippable_ear(
    a: (f64, f64),
    b: (f64, f64),
    c: (f64, f64),
    idx: &[usize],
    uv: &[(f64, f64)],
    ia: usize,
    ib: usize,
    ic: usize,
    eps: f64,
) -> bool {
    let cross = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    if cross <= 0.0 {
        return false; // reflex or collinear corner — not an ear
    }
    let longest_sq = dist_sq(a, b).max(dist_sq(b, c)).max(dist_sq(c, a));
    if cross <= 1e-9 * longest_sq {
        return false; // too thin — defer to avoid a sliver
    }
    for &other in idx {
        if other == ia || other == ib || other == ic {
            continue;
        }
        if point_in_triangle(uv[other], a, b, c, eps) {
            return false;
        }
    }
    true
}

/// Squared distance between two points.
fn dist_sq(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (a.0 - b.0, a.1 - b.1);
    dx * dx + dy * dy
}

/// Is `p` inside (or within `eps`, in twice-area units, of the boundary of)
/// triangle `a b c`? Sign-consistent barycentric test, robust to either
/// triangle winding. The `eps` slack means a vertex grazing an edge counts
/// as inside, so an ear whose diagonal skims a collinear run is rejected.
fn point_in_triangle(p: (f64, f64), a: (f64, f64), b: (f64, f64), c: (f64, f64), eps: f64) -> bool {
    let sign = |p1: (f64, f64), p2: (f64, f64), p3: (f64, f64)| {
        (p1.0 - p3.0) * (p2.1 - p3.1) - (p2.0 - p3.0) * (p1.1 - p3.1)
    };
    let d1 = sign(p, a, b);
    let d2 = sign(p, b, c);
    let d3 = sign(p, c, a);
    let has_neg = d1 < -eps || d2 < -eps || d3 < -eps;
    let has_pos = d1 > eps || d2 > eps || d3 > eps;
    !(has_neg && has_pos)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn signed_area(uv: &[(f64, f64)]) -> f64 {
        let n = uv.len();
        let mut a2 = 0.0;
        for i in 0..n {
            let a = uv[i];
            let b = uv[(i + 1) % n];
            a2 += a.0 * b.1 - b.0 * a.1;
        }
        a2 / 2.0
    }

    fn tri_area(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
        ((b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)) / 2.0
    }

    /// Every triangle is counterclockwise and their areas sum to the
    /// polygon area — i.e. the triangulation tiles the polygon exactly,
    /// with no overlap and no gaps.
    fn assert_tiles(uv: &[(f64, f64)]) {
        let tris = ear_clip(uv);
        assert_eq!(tris.len(), uv.len() - 2, "n-2 triangles expected");
        let mut sum = 0.0;
        for t in &tris {
            let area = tri_area(uv[t[0]], uv[t[1]], uv[t[2]]);
            assert!(
                area > 0.0,
                "triangle {t:?} is not counterclockwise (area {area})"
            );
            sum += area;
        }
        let expected = signed_area(uv).abs();
        assert!(
            (sum - expected).abs() < 1e-9 * (1.0 + expected),
            "triangle areas sum to {sum}, expected polygon area {expected}"
        );
    }

    #[test]
    fn fewer_than_three_vertices() {
        assert!(ear_clip(&[]).is_empty());
        assert!(ear_clip(&[(0.0, 0.0)]).is_empty());
        assert!(ear_clip(&[(0.0, 0.0), (1.0, 0.0)]).is_empty());
    }

    #[test]
    fn convex_quad() {
        assert_tiles(&[(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]);
    }

    #[test]
    fn concave_u_profile() {
        // The U outline from of-6dw: a fan from any apex spills across the
        // notch, but ear clipping tiles it exactly.
        assert_tiles(&[
            (0.0, 0.0),
            (3.0, 0.0),
            (3.0, 3.0),
            (2.0, 3.0),
            (2.0, 1.0),
            (1.0, 1.0),
            (1.0, 3.0),
            (0.0, 3.0),
        ]);
    }

    #[test]
    fn clockwise_input_is_normalized_to_ccw() {
        // Same U, wound clockwise: output must still be counterclockwise.
        assert_tiles(&[
            (0.0, 3.0),
            (1.0, 3.0),
            (1.0, 1.0),
            (2.0, 1.0),
            (2.0, 3.0),
            (3.0, 3.0),
            (3.0, 0.0),
            (0.0, 0.0),
        ]);
    }

    #[test]
    fn concave_arrow() {
        // A four-point arrowhead (one reflex vertex): the classic case a
        // first-vertex fan gets wrong.
        assert_tiles(&[(0.0, 0.0), (2.0, 1.0), (0.0, 2.0), (1.0, 1.0)]);
    }

    #[test]
    fn collinear_run_on_edges_keeps_every_vertex() {
        // A square whose bottom and right edges carry extra collinear
        // midpoints — the runs `sdf_to_brep` recovery leaves along straight
        // facet edges. They must be triangulated into positive-area
        // triangles (no slivers) while every vertex stays referenced, so a
        // neighbouring face's shared edge still lines up (of-6sq).
        let uv = [
            (0.0, 0.0),
            (0.5, 0.0),
            (1.0, 0.0),
            (1.0, 0.5),
            (1.0, 1.0),
            (0.0, 1.0),
        ];
        assert_tiles(&uv);
        let mut referenced = std::collections::HashSet::new();
        for t in ear_clip(&uv) {
            referenced.extend(t);
        }
        assert_eq!(referenced.len(), uv.len(), "every vertex must be used");
    }

    #[test]
    fn thin_strip_with_two_collinear_runs_and_noise() {
        // A tall thin rectangle whose two long edges are collinear runs
        // carrying sub-nanometre noise — a cylinder side facet that
        // `sdf_to_brep` recovers. Ear clipping must zig-zag between the runs
        // without stranding one, whichever way the noise happens to bow each
        // run, and emit no sliver. Exercised end-to-end by the kernel's
        // `brep_sdf_round_trip_preserves_volume_across_two_cycles`.
        let m = 20;
        let noise = |k: usize| ((k as f64 * 12.9898).sin() * 43758.547).fract() * 2e-9 - 1e-9;
        for seed in 0..8 {
            let mut uv = Vec::new();
            for k in 0..m {
                let v = -0.83 + 1.66 * k as f64 / (m - 1) as f64;
                uv.push((-0.0136 + noise(k + seed), v + noise(k + seed + 100)));
            }
            for k in 0..m {
                let v = 0.83 - 1.66 * k as f64 / (m - 1) as f64;
                uv.push((-0.0943 + noise(k + seed + 200), v + noise(k + seed + 300)));
            }
            assert_tiles(&uv);
        }
    }
}
