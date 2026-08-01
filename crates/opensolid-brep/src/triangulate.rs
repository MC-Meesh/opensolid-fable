//! Ear-clipping triangulation of a planar polygon, with or without holes.
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
//! [`ear_clip_rings`] extends it to faces with holes (every drilled plate
//! has them, of-fc8): each hole is spliced into the outer loop along a
//! bridge — a doubled-back segment to a visible outer vertex — turning the
//! multiply-connected region into one simple polygon that ear clipping then
//! handles unchanged. This mirrors `boolean::triangulate_mesh_face`, which
//! bridges the same way.
//!
//! [`is_closed_manifold`]: opensolid_core::mesh::TriangleMesh::is_closed_manifold

use std::collections::HashSet;

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
///
/// Public because the kernel's STEP import fallback tessellator clips
/// planar face outlines with the same routine.
pub fn ear_clip(uv: &[(f64, f64)]) -> Vec<[usize; 3]> {
    if uv.len() < 3 {
        return Vec::new();
    }
    ear_clip_indexed(uv, (0..uv.len()).collect())
}

/// Triangulate a planar face with holes: `rings[0]` is the outer loop and
/// `rings[1..]` are hole loops, each given in order with either winding (the
/// windings are normalized here, so callers need not orient holes opposite
/// the outer loop). Returns triangles as index triples into the rings'
/// concatenation — `rings[0]`'s vertices first, then each hole's in the order
/// given — so a caller that concatenates its 3D points the same way can index
/// them directly. Triangles come out counterclockwise in the `uv` plane, as
/// [`ear_clip`]'s do.
///
/// Returns `None` if some hole cannot be bridged to the outer loop, which
/// means the rings are not a valid face region (a hole outside the outer
/// loop, overlapping holes, self-intersecting boundary). With no holes this
/// is exactly [`ear_clip`] and never fails.
pub fn ear_clip_rings(rings: &[Vec<(f64, f64)>]) -> Option<Vec<[usize; 3]>> {
    let Some(outer) = rings.first() else {
        return Some(Vec::new());
    };
    if outer.len() < 3 {
        return Some(Vec::new());
    }
    if rings.len() == 1 {
        return Some(ear_clip(outer));
    }

    // The returned triples index the rings' concatenation, so the vertex data
    // is laid out exactly as given and never permuted; windings are fixed by
    // ordering *indices* instead.
    let uv: Vec<(f64, f64)> = rings.concat();
    let mut base = 0;
    let mut ring_idx: Vec<Vec<usize>> = Vec::with_capacity(rings.len());
    for ring in rings {
        ring_idx.push((base..base + ring.len()).collect());
        base += ring.len();
    }

    // Bridging needs consistent windings: the outer loop counterclockwise and
    // each hole clockwise, so that splicing a hole in runs it against the
    // outer loop's traversal and leaves one simple, counterclockwise polygon.
    // A hole handed to us counterclockwise would splice into a figure-eight.
    if signed_area2(outer) < 0.0 {
        ring_idx[0].reverse();
    }
    let mut holes: Vec<Vec<usize>> = Vec::new();
    for (h, ring) in ring_idx[1..].iter_mut().zip(&rings[1..]) {
        // A hole of fewer than 3 vertices cuts nothing out; skip it rather
        // than bridge to it. Its vertices keep their slots in `uv` regardless.
        if ring.len() < 3 {
            continue;
        }
        if signed_area2(ring) > 0.0 {
            h.reverse();
        }
        holes.push(std::mem::take(h));
    }
    let mut polygon = std::mem::take(&mut ring_idx[0]);
    if holes.is_empty() {
        return Some(ear_clip_indexed(&uv, polygon));
    }

    // Splice holes rightmost-first (Eberly's ordering): a hole's max-u vertex
    // is guaranteed to see out of its own hole to the right, and taking the
    // rightmost hole first keeps every later hole's bridge target reachable.
    holes.sort_by(|a, b| max_u(&uv, b).total_cmp(&max_u(&uv, a)));

    // Vertices that already carry a bridge, and so already appear twice in
    // `polygon`. Bridging to one of them a second time would make it a *triple*
    // point — see the preference below.
    let mut bridged: HashSet<usize> = HashSet::new();
    for pos in 0..holes.len() {
        let hole = &holes[pos];
        // The hole's rightmost vertex is where the bridge starts.
        let hi_local = (0..hole.len())
            .max_by(|&a, &b| uv[hole[a]].0.total_cmp(&uv[hole[b]].0))
            .expect("hole has at least 3 vertices");
        let h_idx = hole[hi_local];
        // Rings the bridge must not cross: this hole and every hole not yet
        // spliced. Already-spliced holes are polygon segments and so are
        // covered by the polygon check. A bridge that only dodges the current
        // hole can still cut a later one, whose splice would then
        // self-intersect the polygon.
        let unspliced = &holes[pos..];
        // Bridge to the nearest polygon vertex the segment can reach without
        // crossing anything.
        let mut candidates: Vec<usize> = (0..polygon.len()).collect();
        candidates.sort_by(|&a, &b| {
            dist_sq(uv[polygon[a]], uv[h_idx]).total_cmp(&dist_sq(uv[polygon[b]], uv[h_idx]))
        });
        // Prefer a target that does not already carry a bridge (of-kll8).
        //
        // A bridge's own vertices each appear twice in the spliced polygon,
        // which is harmless: the two occurrences are separated by a slit walked
        // out and back, so the boundary still leaves the vertex the same way it
        // arrived. Anchoring a *second* bridge at one of them is not — the
        // vertex becomes a genuine pinch joining three boundary excursions, the
        // polygon stops being simple, and the two-ears theorem stops applying
        // to it. Ear clipping then runs out of clippable ears with hundreds of
        // vertices still in hand and hands the rest to the least-reflex
        // fallback, which ignores containment and laps the polygon in
        // overlapping triangles: `nist_ctc_03`'s 8-hole plate face came out
        // combinatorially closed carrying over twice its own area.
        //
        // This is only a preference. A polygon always has more vertices than
        // bridge endpoints, so a non-bridged candidate exists, but it need not
        // be *reachable*; refusing the whole face over that would be a
        // regression against simply taking the pinch.
        let clear = |c: &usize| bridge_is_clear(&uv, &polygon, unspliced, h_idx, polygon[*c]);
        let cand = candidates
            .iter()
            .copied()
            .find(|c| !bridged.contains(&polygon[*c]) && clear(c))
            .or_else(|| candidates.iter().copied().find(clear))?;
        // Splice: ...p, h, h+1, ..., h, p... — the hole is traversed once and
        // the bridge is walked out and back, so the result stays a single
        // closed polygon whose area is the outer loop's less the holes'.
        let p_idx = polygon[cand];
        bridged.insert(h_idx);
        bridged.insert(p_idx);
        let hn = holes[pos].len();
        let mut spliced = Vec::with_capacity(polygon.len() + hn + 2);
        spliced.extend_from_slice(&polygon[..=cand]);
        for k in 0..=hn {
            spliced.push(holes[pos][(hi_local + k) % hn]);
        }
        spliced.push(p_idx);
        spliced.extend_from_slice(&polygon[cand + 1..]);
        polygon = spliced;
    }

    Some(ear_clip_indexed(&uv, polygon))
}

/// Twice the signed area of a closed polygon: positive when counterclockwise,
/// negative when clockwise. Public because callers assembling rings for
/// [`ear_clip_rings`] need it to tell the outer loop from the holes.
pub fn signed_area2(uv: &[(f64, f64)]) -> f64 {
    let n = uv.len();
    let mut area2 = 0.0;
    for i in 0..n {
        let a = uv[i];
        let b = uv[(i + 1) % n];
        area2 += a.0 * b.1 - b.0 * a.1;
    }
    area2
}

/// Largest `u` over a ring's vertices.
fn max_u(uv: &[(f64, f64)], ring: &[usize]) -> f64 {
    ring.iter().fold(f64::NEG_INFINITY, |m, &i| m.max(uv[i].0))
}

/// Does the bridge `from`→`to` cross any polygon segment, or any segment of
/// the not-yet-spliced hole rings — or *run through* one of their vertices?
/// Segments sharing an endpoint with the bridge — including the bridge's own
/// two endpoints' incident edges — do not count as crossings.
fn bridge_is_clear(
    uv: &[(f64, f64)],
    polygon: &[usize],
    unspliced: &[Vec<usize>],
    from: usize,
    to: usize,
) -> bool {
    let (p, q) = (uv[from], uv[to]);
    for ring in std::iter::once(polygon).chain(unspliced.iter().map(Vec::as_slice)) {
        let n = ring.len();
        for i in 0..n {
            if segments_cross(p, q, uv[ring[i]], uv[ring[(i + 1) % n]]) {
                return false;
            }
            if vertex_on_segment(p, q, uv[ring[i]]) {
                return false;
            }
        }
    }
    true
}

/// Does `v` lie *on* the open segment `p`→`q`, strictly between its ends?
///
/// [`segments_cross`] is a strict *proper* crossing test — it sees a segment
/// pass from one side of the bridge to the other, and nothing else. A ring
/// vertex sitting exactly on the bridge does not do that: both of its incident
/// edges leave from the same point on the line, so neither one properly
/// crosses, and the bridge is declared clear. Splicing along it then makes the
/// polygon *touch* that ring instead of merely reaching it, and a self-touching
/// polygon is no longer covered by the two-ears theorem — ear clipping can and
/// does run out of clippable ears with hundreds of vertices left, at which
/// point the least-reflex fallback laps the polygon emitting overlapping
/// triangles (of-kll8).
///
/// Equal holes drilled in a row make this the *normal* case, not a corner
/// one: their rightmost vertices come out collinear and evenly spaced, so the
/// bridge from the first hole to the third runs exactly through the second's.
/// Both of `nist_ctc_03`'s 8-hole plate faces laid a bridge that way, one at
/// the grazed vertex's exact midpoint.
///
/// Rejecting the bridge costs nothing: the caller simply takes the next
/// candidate vertex, and only a hole with no clear bridge at all fails.
fn vertex_on_segment(p: (f64, f64), q: (f64, f64), v: (f64, f64)) -> bool {
    let (dx, dy) = (q.0 - p.0, q.1 - p.1);
    let length_sq = dx * dx + dy * dy;
    if length_sq == 0.0 {
        return false;
    }
    // Strictly between the ends: a vertex *at* either end is one of the
    // bridge's own endpoints (or a duplicate of one, sharing its coordinates
    // exactly), which is the whole point of the bridge.
    let t = ((v.0 - p.0) * dx + (v.1 - p.1) * dy) / length_sq;
    if t <= 0.0 || t >= 1.0 {
        return false;
    }
    // Perpendicular offset of 1e-9 of the bridge's own length, expressed in
    // twice-area units to stay division-free: `|cross| <= 1e-9 * length²`.
    let cross = dx * (v.1 - p.1) - dy * (v.0 - p.0);
    cross * cross <= 1e-18 * length_sq * length_sq
}

/// Strict proper crossing test: segments that merely share an endpoint (as a
/// bridge always does with the two edges at each of its ends) do not cross.
fn segments_cross(p1: (f64, f64), p2: (f64, f64), q1: (f64, f64), q2: (f64, f64)) -> bool {
    let eps = 1e-14;
    if dist_sq(p1, q1) < eps
        || dist_sq(p1, q2) < eps
        || dist_sq(p2, q1) < eps
        || dist_sq(p2, q2) < eps
    {
        return false;
    }
    let d = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
    };
    let (d1, d2) = (d(q1, q2, p1), d(q1, q2, p2));
    let (d3, d4) = (d(p1, p2, q1), d(p1, p2, q2));
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

/// Ear-clip the closed polygon `idx` (indices into `uv`, either winding).
/// Indices may repeat — a bridged hole polygon walks its bridge vertices
/// twice — so the ear tests compare by index, never by position.
fn ear_clip_indexed(uv: &[(f64, f64)], mut idx: Vec<usize>) -> Vec<[usize; 3]> {
    let n = idx.len();
    if n < 3 {
        return Vec::new();
    }

    // Ear clipping needs a counterclockwise loop so a positive corner
    // cross product marks a convex (candidate-ear) corner. Detect the
    // winding from the signed area and walk the indices in reverse
    // when it runs clockwise; the emitted triples then always come out
    // counterclockwise, independent of how the caller wound the loop.
    let ring: Vec<(f64, f64)> = idx.iter().map(|&i| uv[i]).collect();
    if signed_area2(&ring) < 0.0 {
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
        // Look for an ear twice: with the containment slack, and — only if
        // that found none at all — without it (of-kll8).
        //
        // The slack only ever *rejects* ears, and it is priced off the whole
        // polygon's extent while an ear is as small as the feature it belongs
        // to. A 3.2 m plate drilled with 1 mm holes prices it at 2.6e-3 in
        // twice-area units, an order of magnitude past a hole ear's own
        // 2.8e-4: every ear along such a hole reads as containing something
        // and none of them clip. Reaching the fallback below over that is far
        // worse than clipping a strictly empty ear, because the fallback
        // ignores containment altogether and pays for its guaranteed progress
        // in overlapping triangles.
        //
        // This cannot strand a collinear run the way of-6sq describes: the
        // strict pass runs only where the tolerant one has already given up,
        // so what it competes against is the fallback, not a later ear.
        'search: for tolerance in [eps, 0.0] {
            for i in 0..m {
                let (ia, ib, ic) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
                let (a, b, c) = (uv[ia], uv[ib], uv[ic]);
                if !is_clippable_ear(a, b, c, &idx, uv, ia, ib, ic, tolerance) {
                    continue;
                }
                tris.push([ia, ib, ic]);
                idx.remove(i);
                clipped = true;
                break 'search;
            }
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

    /// Every triangle is counterclockwise, no triangle straddles a hole, and
    /// the areas sum to (outer − holes) — i.e. the triangulation tiles the
    /// holed region exactly, with no overlap, no gaps, and nothing laid down
    /// over a hole.
    fn assert_tiles_rings(rings: &[Vec<(f64, f64)>]) {
        let uv = rings.concat();
        let tris = ear_clip_rings(rings).expect("rings must triangulate");
        let mut sum = 0.0;
        for t in &tris {
            let area = tri_area(uv[t[0]], uv[t[1]], uv[t[2]]);
            assert!(
                area > -1e-12,
                "triangle {t:?} is not counterclockwise (area {area})"
            );
            sum += area;
            // A triangle centroid inside a hole means the clipper paved over
            // the hole — the exact failure a bridgeless ear clip makes, and
            // one the area sum alone can miss if another triangle is dropped.
            let centroid = (
                (uv[t[0]].0 + uv[t[1]].0 + uv[t[2]].0) / 3.0,
                (uv[t[0]].1 + uv[t[1]].1 + uv[t[2]].1) / 3.0,
            );
            for hole in &rings[1..] {
                assert!(
                    !point_in_polygon(centroid, hole),
                    "triangle {t:?} lies inside a hole"
                );
            }
        }
        let expected = signed_area(&rings[0]).abs()
            - rings[1..].iter().map(|h| signed_area(h).abs()).sum::<f64>();
        assert!(
            (sum - expected).abs() < 1e-9 * (1.0 + expected.abs()),
            "triangle areas sum to {sum}, expected holed area {expected}"
        );
    }

    /// Even-odd ray cast; only used on the strict interior (triangle
    /// centroids), so boundary grazing is not a concern.
    fn point_in_polygon(p: (f64, f64), poly: &[(f64, f64)]) -> bool {
        let n = poly.len();
        let mut inside = false;
        for i in 0..n {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            if (a.1 > p.1) != (b.1 > p.1) {
                let x = a.0 + (p.1 - a.1) / (b.1 - a.1) * (b.0 - a.0);
                if p.0 < x {
                    inside = !inside;
                }
            }
        }
        inside
    }

    fn square(cx: f64, cy: f64, half: f64) -> Vec<(f64, f64)> {
        vec![
            (cx - half, cy - half),
            (cx + half, cy - half),
            (cx + half, cy + half),
            (cx - half, cy + half),
        ]
    }

    /// A circle of `n` points, counterclockwise — the loop a drilled hole's
    /// circular edge samples to.
    fn circle(cx: f64, cy: f64, r: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / n as f64;
                (cx + r * a.cos(), cy + r * a.sin())
            })
            .collect()
    }

    #[test]
    fn rings_without_holes_match_ear_clip() {
        let outer = square(0.0, 0.0, 1.0);
        assert_eq!(
            ear_clip_rings(std::slice::from_ref(&outer)),
            Some(ear_clip(&outer))
        );
        assert_eq!(ear_clip_rings(&[]), Some(Vec::new()));
    }

    #[test]
    fn drilled_plate_face_leaves_the_hole_open() {
        // The of-fc8 case: a plate face with one circular hole. Before hole
        // bridging this whole shape was rejected outright.
        assert_tiles_rings(&[square(0.0, 0.0, 2.0), circle(0.0, 0.0, 0.5, 16)]);
    }

    #[test]
    fn hole_winding_is_normalized() {
        // Callers hand holes in whichever winding their topology stores. A
        // counterclockwise hole must be recognized and reversed, not spliced
        // into a figure-eight.
        let mut ccw_hole = circle(0.0, 0.0, 0.5, 12);
        assert!(signed_area2(&ccw_hole) > 0.0, "fixture must be ccw");
        assert_tiles_rings(&[square(0.0, 0.0, 2.0), ccw_hole.clone()]);
        ccw_hole.reverse();
        assert_tiles_rings(&[square(0.0, 0.0, 2.0), ccw_hole]);
        // Same for a clockwise outer loop.
        let mut cw_outer = square(0.0, 0.0, 2.0);
        cw_outer.reverse();
        assert_tiles_rings(&[cw_outer, circle(0.0, 0.0, 0.5, 12)]);
    }

    #[test]
    fn many_holes_in_one_face() {
        // A four-hole bolt pattern: each hole bridges independently, and a
        // bridge must not cut through a hole spliced later (the boolean
        // clipper's `hole_bridge_avoids_unspliced_holes` case).
        assert_tiles_rings(&[
            square(0.0, 0.0, 3.0),
            circle(-1.5, -1.5, 0.4, 10),
            circle(1.5, -1.5, 0.4, 10),
            circle(1.5, 1.5, 0.4, 10),
            circle(-1.5, 1.5, 0.4, 10),
        ]);
    }

    /// A hole pointing right, whose rightmost vertex is unique and whose other
    /// vertices trail off to the lower left — the shape that makes the bridge
    /// target predictable.
    fn barb(cx: f64, cy: f64) -> Vec<(f64, f64)> {
        vec![(cx, cy), (cx - 1.0, cy - 3.0), (cx - 3.0, cy - 1.0)]
    }

    #[test]
    fn a_bridge_grazing_a_ring_vertex_is_not_clear() {
        // of-kll8. The bridge runs from one hole's tip straight down to
        // another's, passing exactly through the tip of a third hole between
        // them without ever entering it. Nothing *crosses*, so the proper-
        // intersection test alone waves the bridge through and the spliced
        // polygon comes out touching itself — the state ear clipping cannot
        // work out of, and the state `nist_ctc_03`'s plate faces were both in.
        let rings = [
            square(0.0, 0.0, 30.0),
            barb(10.0, 20.0),
            barb(10.0, 0.0),
            barb(10.0, 10.0),
        ];
        let uv: Vec<(f64, f64)> = rings.concat();
        let index = |ring: usize, k: usize| rings[..ring].iter().map(Vec::len).sum::<usize>() + k;
        let (from, to) = (index(1, 0), index(2, 0));
        assert_eq!((uv[from], uv[to]), ((10.0, 20.0), (10.0, 0.0)));
        let outer: Vec<usize> = (0..rings[0].len()).collect();
        let grazed: Vec<usize> = (0..rings[3].len()).map(|k| index(3, k)).collect();
        assert!(
            bridge_is_clear(&uv, &outer, &[], from, to),
            "nothing but the outer loop is in the way"
        );
        assert!(
            !bridge_is_clear(&uv, &outer, std::slice::from_ref(&grazed), from, to),
            "the third hole's tip is on the bridge"
        );
    }

    #[test]
    fn a_hole_vertex_never_anchors_two_bridges() {
        // of-kll8. Four holes of assorted sizes, positioned so that the third
        // one spliced finds its nearest reachable target on the *bridge* the
        // second one just laid down. Anchoring there turns that vertex from a
        // slit — walked out and back, which the clipper handles — into a pinch
        // where three excursions of the boundary meet, and the polygon stops
        // being one ear clipping can finish: it laid down 1.5x the area here,
        // and over twice it on `nist_ctc_03`'s 8-hole plate faces.
        //
        // The layout is a random one from a 400-case sweep; 17 of those cases
        // folded, and this is the smallest.
        let rings = vec![
            square(0.0, 0.0, 100.0),
            circle(-69.8, 69.2, 8.9, 32),
            circle(90.3, 86.5, 3.8, 32),
            circle(89.1, 40.5, 7.9, 32),
            circle(-17.6, 57.1, 2.2, 32),
        ];
        assert_tiles_rings(&rings);
    }

    #[test]
    fn small_holes_in_a_large_face_tile_without_overlap() {
        // of-kll8. The ear containment slack is priced off the whole polygon's
        // extent, so on a face three orders of magnitude wider than its holes
        // it exceeds a hole ear's entire area and every ear along that hole
        // reads as occupied. The clipper then has no ear to take and falls
        // back on clipping the least-reflex corner, which is only guaranteed
        // to make progress — not to stay inside the polygon.
        let mut rings = vec![square(0.0, 0.0, 1600.0)];
        for k in 0..8 {
            rings.push(circle(0.0, -52.5 + 15.0 * k as f64, 1.0, 96));
        }
        assert_tiles_rings(&rings);
    }

    #[test]
    fn vertex_on_segment_only_sees_the_strict_interior() {
        let (p, q) = ((0.0, 0.0), (10.0, 0.0));
        assert!(vertex_on_segment(p, q, (5.0, 0.0)), "midpoint is on it");
        assert!(
            vertex_on_segment(p, q, (5.0, 1e-12)),
            "within the perpendicular slack"
        );
        assert!(
            !vertex_on_segment(p, q, (5.0, 1e-3)),
            "a real feature off to the side is not on it"
        );
        // The bridge's own endpoints, and duplicates of them, are the point of
        // the bridge — never a blocker.
        assert!(!vertex_on_segment(p, q, p));
        assert!(!vertex_on_segment(p, q, q));
        // Collinear but past an end.
        assert!(!vertex_on_segment(p, q, (-1.0, 0.0)));
        assert!(!vertex_on_segment(p, q, (11.0, 0.0)));
        // A degenerate bridge has no interior.
        assert!(!vertex_on_segment(p, p, (0.0, 0.0)));
    }

    #[test]
    fn holes_in_a_concave_outline() {
        // Concave outer loop plus holes: the two features that each broke a
        // naive triangulator, together.
        let outer = vec![
            (0.0, 0.0),
            (6.0, 0.0),
            (6.0, 6.0),
            (4.0, 6.0),
            (4.0, 2.0),
            (2.0, 2.0),
            (2.0, 6.0),
            (0.0, 6.0),
        ];
        assert_tiles_rings(&[outer, circle(1.0, 1.0, 0.4, 10), circle(5.0, 1.0, 0.4, 10)]);
    }

    #[test]
    fn hole_touching_the_outer_loop_is_not_a_crossing() {
        // A slot whose hole vertex sits exactly on the outer edge: the bridge
        // shares an endpoint with outer segments, which is not a crossing.
        // Rejecting it would fail an otherwise valid face.
        assert_tiles_rings(&[square(0.0, 0.0, 2.0), circle(0.0, 0.0, 1.0, 8)]);
    }

    #[test]
    fn degenerate_hole_keeps_later_indices_aligned() {
        // A hole that samples to fewer than 3 points cuts nothing out, but its
        // vertices still occupy their slots: a caller's parallel 3D point list
        // is indexed by the same concatenation, so dropping them would shift
        // every later ring's triangles onto the wrong points.
        let rings = vec![
            square(0.0, 0.0, 3.0),
            vec![(0.0, 0.0), (0.1, 0.0)],
            circle(1.5, 1.5, 0.4, 10),
        ];
        let uv = rings.concat();
        let tris = ear_clip_rings(&rings).expect("degenerate hole must not fail the face");
        let real_hole = &rings[2];
        for t in &tris {
            let centroid = (
                (uv[t[0]].0 + uv[t[1]].0 + uv[t[2]].0) / 3.0,
                (uv[t[0]].1 + uv[t[1]].1 + uv[t[2]].1) / 3.0,
            );
            assert!(
                !point_in_polygon(centroid, real_hole),
                "triangle {t:?} paved over the real hole — indices shifted"
            );
        }
        let area: f64 = tris
            .iter()
            .map(|t| tri_area(uv[t[0]], uv[t[1]], uv[t[2]]))
            .sum();
        let expected = 36.0 - signed_area(real_hole).abs();
        assert!((area - expected).abs() < 1e-9, "area {area} != {expected}");
    }

    #[test]
    fn hole_outside_the_outer_loop_is_rejected() {
        // Not a valid face region: the bridge would have to cross the outer
        // loop. Better to report it than to emit a garbage triangulation.
        assert_eq!(
            ear_clip_rings(&[square(0.0, 0.0, 1.0), circle(5.0, 5.0, 0.4, 8)]),
            None
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
