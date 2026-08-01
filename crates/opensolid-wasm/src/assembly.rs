//! Assembly bindings (of-2y4.8): the kernel's assembly layer —
//! [`Assembly`](opensolid_kernel::assembly::Assembly) instancing, the mate
//! solver, interference detection, and aggregate mass properties — exposed to
//! JS so the MCP `assemble` tool can drive it.
//!
//! The shape of the API mirrors the kernel: build a [`WasmAssembly`], add
//! placed instances of existing [`WasmShape`]s, add mates between named
//! features on those instances, then `solve()`. Reports come back as JSON
//! strings (the crate's convention for structured results — see
//! [`WasmShape::measure`]); `assembledShape()` hands back the placed union as
//! an ordinary [`WasmShape`] so every existing tool (mesh, measure, export,
//! screenshots) works on the assembly unchanged.
//!
//! Instances referencing the same [`WasmShape`] share one
//! [`Part`](opensolid_kernel::assembly::Part) — ten bolts are ten poses over
//! one geometry, per `docs/design/ASSEMBLIES.md` §1.

use std::sync::Arc;

use nalgebra::UnitQuaternion;
use wasm_bindgen::prelude::*;

use opensolid_core::types::{Point3, Transform3, Vector3};
use opensolid_kernel::assembly::{
    Assembly, Feature, FeatureRef, Instance, Mate, Part, SolveStatus,
};
use opensolid_kernel::mass_properties;

use crate::bounded::BoundedShape;
use crate::{WasmShape, json_escape, json_num};

/// A placed, solvable multi-part assembly.
///
/// Wraps the kernel [`Assembly`] plus the per-instance [`BoundedShape`]s
/// needed to rebuild the placed union after a solve moves the poses.
#[wasm_bindgen]
pub struct WasmAssembly {
    inner: Assembly,
    /// Each instance's source shape (SDF + tracked bounds), kept so
    /// [`assembled_shape`](Self::assembled_shape) can place it by the
    /// *current* (possibly solved) pose.
    sources: Vec<BoundedShape>,
    /// Distinct parts already wrapped, so instances of one shape share one
    /// [`Part`] (matched by SDF pointer identity).
    parts: Vec<Arc<Part>>,
    /// Mass-property failure per instance (`None` when the part measured
    /// clean). A part whose mesh does not close still places, solves, and
    /// clash-checks; only the aggregate mass report degrades, and it says so.
    mass_errors: Vec<Option<String>>,
}

impl Default for WasmAssembly {
    fn default() -> Self {
        Self::new()
    }
}

/// Read a JS-supplied `[x, y, z]` triple.
fn vec3(v: &[f64], what: &str) -> Result<Vector3, String> {
    if v.len() != 3 || v.iter().any(|c| !c.is_finite()) {
        return Err(format!("{what} must be [x, y, z] with finite components"));
    }
    Ok(Vector3::new(v[0], v[1], v[2]))
}

/// Build the rigid placement from a translation and an axis–angle rotation.
/// A zero axis with a zero angle is the identity rotation; a zero axis with a
/// non-zero angle is rejected (the rotation is unspecified).
fn placement(translation: &[f64], axis: &[f64], angle_deg: f64) -> Result<Transform3, String> {
    let t = vec3(translation, "translation")?;
    let a = vec3(axis, "rotation axis")?;
    if !angle_deg.is_finite() {
        return Err("rotation angle must be a finite number of degrees".to_string());
    }
    let angle = angle_deg.to_radians();
    let rot = if angle == 0.0 {
        Transform3::identity().rotation
    } else {
        let n = a.norm();
        if n < 1e-12 {
            return Err("rotation axis must be non-zero when the angle is non-zero".to_string());
        }
        UnitQuaternion::from_scaled_axis(a / n * angle)
    };
    Ok(Transform3::from_parts(t.into(), rot))
}

/// Parse one mate feature from its wire form: a `kind` string plus a point
/// and (for planes and axes) a direction, all in the instance's local frame.
fn feature(kind: &str, point: &[f64], direction: &[f64]) -> Result<Feature, String> {
    let p: Point3 = vec3(point, "feature point")?.into();
    match kind {
        "plane" => Feature::plane(p, vec3(direction, "plane normal")?).map_err(|e| e.to_string()),
        "axis" => Feature::axis(p, vec3(direction, "axis direction")?).map_err(|e| e.to_string()),
        "point" => Ok(Feature::point(p)),
        other => Err(format!(
            "unknown feature kind '{other}'; use plane, axis, or point"
        )),
    }
}

/// Serialize a pose as JSON: translation plus rotation as both a unit
/// quaternion (exact) and axis–angle (legible).
fn transform_json(t: &Transform3) -> String {
    let tr = t.translation.vector;
    let q = t.rotation;
    let axis_angle = q.scaled_axis();
    let angle = axis_angle.norm();
    let axis = if angle > 1e-12 {
        axis_angle / angle
    } else {
        Vector3::z()
    };
    format!(
        "{{\"translation\":[{},{},{}],\"quaternion\":[{},{},{},{}],\
         \"rotationAxis\":[{},{},{}],\"rotationDeg\":{}}}",
        json_num(tr.x),
        json_num(tr.y),
        json_num(tr.z),
        json_num(q.i),
        json_num(q.j),
        json_num(q.k),
        json_num(q.w),
        json_num(axis.x),
        json_num(axis.y),
        json_num(axis.z),
        json_num(angle.to_degrees()),
    )
}

#[wasm_bindgen]
impl WasmAssembly {
    /// An empty assembly.
    #[wasm_bindgen(constructor)]
    pub fn new() -> WasmAssembly {
        WasmAssembly {
            inner: Assembly::new(),
            sources: Vec::new(),
            parts: Vec::new(),
            mass_errors: Vec::new(),
        }
    }

    /// Add a placed instance of `shape` and return its index (the handle
    /// mates reference).
    ///
    /// `translation` is `[x, y, z]`; the rotation is `angle_deg` degrees
    /// about `axis` (any non-zero length; `[0,0,0]` with angle 0 is
    /// identity). A `fixed` instance is ground for the solver; `density`
    /// scales its mass. Instances of the same `shape` object share one part
    /// — no geometry is copied.
    #[wasm_bindgen(js_name = addInstance)]
    #[allow(clippy::too_many_arguments)]
    pub fn add_instance(
        &mut self,
        shape: &WasmShape,
        translation: Vec<f64>,
        axis: Vec<f64>,
        angle_deg: f64,
        fixed: bool,
        density: f64,
        name: &str,
    ) -> Result<usize, String> {
        let transform = placement(&translation, &axis, angle_deg)?;
        if !(density.is_finite() && density >= 0.0) {
            return Err(format!(
                "density must be a non-negative finite number, got {density}"
            ));
        }
        let (part, mass_error) = self.part_for(shape);
        let index = self.inner.insert(
            Instance::new(part, transform)
                .fixed(fixed)
                .density(density)
                .named(name),
        );
        self.sources.push(shape.inner.clone());
        self.mass_errors.push(mass_error);
        Ok(index)
    }

    /// Add a mate between a feature on instance `a_instance` and a feature
    /// on instance `b_instance`; returns the mate's index.
    ///
    /// `kind` is `coincident` (plane–plane or point–plane), `concentric`
    /// (axis–axis), or `distance` (plane–plane or point–point, requiring
    /// `value`). Features are given in each instance's local frame:
    /// `*_feature` is `plane` / `axis` / `point`, `*_point` is a point on it,
    /// and `*_direction` is the plane normal or axis direction (ignored for
    /// point features — pass `[0,0,0]`).
    #[wasm_bindgen(js_name = addMate)]
    #[allow(clippy::too_many_arguments)]
    pub fn add_mate(
        &mut self,
        kind: &str,
        a_instance: usize,
        a_feature: &str,
        a_point: Vec<f64>,
        a_direction: Vec<f64>,
        b_instance: usize,
        b_feature: &str,
        b_point: Vec<f64>,
        b_direction: Vec<f64>,
        value: Option<f64>,
    ) -> Result<usize, String> {
        let count = self.inner.len();
        for (label, i) in [("a", a_instance), ("b", b_instance)] {
            if i >= count {
                return Err(format!(
                    "mate side {label} references instance {i}, but the assembly has {count} \
                     instance(s)"
                ));
            }
        }
        let a = FeatureRef::new(
            a_instance,
            feature(a_feature, &a_point, &a_direction).map_err(|e| format!("side a: {e}"))?,
        );
        let b = FeatureRef::new(
            b_instance,
            feature(b_feature, &b_point, &b_direction).map_err(|e| format!("side b: {e}"))?,
        );
        let mate = match kind {
            "coincident" => Mate::coincident(a, b).map_err(|e| e.to_string())?,
            "concentric" => Mate::concentric(a, b).map_err(|e| e.to_string())?,
            "distance" => {
                let v = value.ok_or("distance mate requires a value")?;
                if !v.is_finite() {
                    return Err(format!("distance value must be finite, got {v}"));
                }
                Mate::distance(a, b, v).map_err(|e| e.to_string())?
            }
            other => {
                return Err(format!(
                    "unknown mate kind '{other}'; use coincident, concentric, or distance"
                ));
            }
        };
        Ok(self.inner.add_mate(mate))
    }

    /// Number of instances added so far.
    #[wasm_bindgen(js_name = instanceCount)]
    pub fn instance_count(&self) -> usize {
        self.inner.len()
    }

    /// Solve the mates and write the resolved poses back into the instances.
    ///
    /// Returns JSON: `status` (`converged` | `over_constrained`),
    /// `residualNorm`, `iterations`, `freeDof` (remaining unconstrained
    /// degrees of freedom — a seated bolt free to spin reports 1), and the
    /// resolved `transforms` per instance in insertion order. Solving with
    /// no mates converges trivially and leaves every pose unchanged.
    pub fn solve(&mut self) -> String {
        let result = self.inner.solve_in_place();
        let status = match result.status {
            SolveStatus::Converged => "converged",
            SolveStatus::OverConstrained => "over_constrained",
        };
        let transforms = result
            .transforms
            .iter()
            .map(transform_json)
            .collect::<Vec<_>>()
            .join(",");
        format!(
            "{{\"status\":\"{status}\",\"residualNorm\":{},\"iterations\":{},\
             \"freeDof\":{},\"transforms\":[{transforms}]}}",
            json_num(result.residual_norm),
            result.iterations,
            result.free_dof,
        )
    }

    /// Current pose of every instance (insertion order) as JSON — the same
    /// shape `solve` reports, without solving.
    pub fn transforms(&self) -> String {
        let list = self
            .inner
            .instances()
            .iter()
            .map(|i| transform_json(&i.transform))
            .collect::<Vec<_>>()
            .join(",");
        format!("[{list}]")
    }

    /// Check every instance pair for interference (shared interior volume)
    /// at the current poses.
    ///
    /// Returns JSON: `interferes` (any pair clashes), `checkedPairs`, and
    /// `pairs` — one `{a, b, aName, bName, volume}` entry per clashing pair,
    /// where `volume` is the estimated overlap volume.
    pub fn interferences(&self) -> String {
        let n = self.inner.len();
        let clashes = self.inner.all_interferences();
        let names: Vec<&str> = self
            .inner
            .instances()
            .iter()
            .map(|i| i.name.as_str())
            .collect();
        let pairs = clashes
            .iter()
            .map(|(i, j, report)| {
                format!(
                    "{{\"a\":{i},\"b\":{j},\"aName\":\"{}\",\"bName\":\"{}\",\"volume\":{}}}",
                    json_escape(names[*i]),
                    json_escape(names[*j]),
                    json_num(report.volume),
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        format!(
            "{{\"interferes\":{},\"checkedPairs\":{},\"pairs\":[{pairs}]}}",
            !clashes.is_empty(),
            n * n.saturating_sub(1) / 2,
        )
    }

    /// Aggregate mass properties at the current poses, composed from the
    /// per-part cached properties without re-meshing.
    ///
    /// Returns JSON: `volume`, `surfaceArea`, `mass`, `centroid`, `inertia`
    /// (3×3, about the centroid, assembly frame), plus `massErrors` naming
    /// any instance whose part could not be measured (those contribute
    /// nothing, so the aggregate is a lower bound and says so). An empty or
    /// zero-mass assembly returns `{"error": …}` instead.
    #[wasm_bindgen(js_name = massProperties)]
    pub fn mass_properties(&self) -> String {
        let errors = self
            .mass_errors
            .iter()
            .enumerate()
            .filter_map(|(i, e)| {
                e.as_ref().map(|msg| {
                    format!(
                        "{{\"instance\":{i},\"name\":\"{}\",\"error\":\"{}\"}}",
                        json_escape(&self.inner.instances()[i].name),
                        json_escape(msg),
                    )
                })
            })
            .collect::<Vec<_>>()
            .join(",");
        match self.inner.mass_properties() {
            Ok(mp) => {
                let i = &mp.inertia;
                format!(
                    "{{\"volume\":{},\"surfaceArea\":{},\"mass\":{},\
                     \"centroid\":[{},{},{}],\
                     \"inertia\":[[{},{},{}],[{},{},{}],[{},{},{}]],\
                     \"massErrors\":[{errors}]}}",
                    json_num(mp.volume),
                    json_num(mp.surface_area),
                    json_num(mp.mass),
                    json_num(mp.centroid.x),
                    json_num(mp.centroid.y),
                    json_num(mp.centroid.z),
                    json_num(i[(0, 0)]),
                    json_num(i[(0, 1)]),
                    json_num(i[(0, 2)]),
                    json_num(i[(1, 0)]),
                    json_num(i[(1, 1)]),
                    json_num(i[(1, 2)]),
                    json_num(i[(2, 0)]),
                    json_num(i[(2, 1)]),
                    json_num(i[(2, 2)]),
                )
            }
            Err(err) => format!(
                "{{\"error\":\"{}\",\"massErrors\":[{errors}]}}",
                json_escape(&err.to_string()),
            ),
        }
    }

    /// The placed assembly as one [`WasmShape`]: every instance's shape
    /// carried through its current (possibly solved) pose and unioned. The
    /// result works with every shape tool — mesh, measure, validate, export,
    /// silhouettes — exactly like a `create_model` script result.
    ///
    /// Errors on an empty assembly.
    #[wasm_bindgen(js_name = assembledShape)]
    pub fn assembled_shape(&self) -> Result<WasmShape, String> {
        let mut placed = self
            .inner
            .instances()
            .iter()
            .zip(&self.sources)
            .map(|(inst, src)| src.transform(&inst.transform));
        let first = placed.next().ok_or("assembly has no instances")?;
        Ok(WasmShape::sdf_only(
            placed.fold(first, |acc, next| acc.union(&next)),
        ))
    }

    /// The shared [`Part`] for `shape`, reusing an existing wrapper when the
    /// same shape object was added before (SDF pointer identity), plus the
    /// mass-measurement error if its mesh failed to close.
    fn part_for(&mut self, shape: &WasmShape) -> (Arc<Part>, Option<String>) {
        if let Some(existing) = self
            .parts
            .iter()
            .find(|p| p.shape.ptr_eq(&shape.inner.shape))
        {
            let error = self
                .inner
                .instances()
                .iter()
                .zip(&self.mass_errors)
                .find(|(inst, _)| Arc::ptr_eq(&inst.part, existing))
                .and_then(|(_, e)| e.clone());
            return (existing.clone(), error);
        }
        // Measure through the same mesh `measure`/`validate` read (the exact
        // tessellation when the shape carries one, an adaptive SDF mesh
        // otherwise), so the assembly's mass report agrees with the per-part
        // tools. A mesh that does not close degrades to zero mass properties
        // with the reason recorded — placement, solving, and interference do
        // not need mass.
        let measured = shape.with_measure_mesh(None, mass_properties);
        let (props, error) = match measured {
            Ok(props) => (props, None),
            Err(err) => (
                opensolid_kernel::MassProperties {
                    volume: 0.0,
                    surface_area: 0.0,
                    centroid: Point3::origin(),
                    inertia: nalgebra::Matrix3::zeros(),
                },
                Some(err.to_string()),
            ),
        };
        let part = Arc::new(Part::new(
            shape.inner.shape.clone(),
            shape.inner.bounds,
            props,
        ));
        self.parts.push(part.clone());
        (part, error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(json: &str) -> serde_json_value::Value {
        serde_json_value::parse(json)
    }

    /// Minimal JSON reader for the tests: the crate hand-writes JSON and has
    /// no serde dependency, so assertions parse with a tiny recursive-descent
    /// reader rather than pulling one in.
    mod serde_json_value {
        use std::collections::BTreeMap;

        #[derive(Debug, Clone, PartialEq)]
        pub enum Value {
            Null,
            Bool(bool),
            Num(f64),
            Str(String),
            Arr(Vec<Value>),
            Obj(BTreeMap<String, Value>),
        }

        impl Value {
            pub fn get(&self, key: &str) -> &Value {
                match self {
                    Value::Obj(m) => m.get(key).unwrap_or_else(|| panic!("missing key {key}")),
                    other => panic!("not an object: {other:?}"),
                }
            }
            pub fn idx(&self, i: usize) -> &Value {
                match self {
                    Value::Arr(v) => &v[i],
                    other => panic!("not an array: {other:?}"),
                }
            }
            pub fn len(&self) -> usize {
                match self {
                    Value::Arr(v) => v.len(),
                    other => panic!("not an array: {other:?}"),
                }
            }
            pub fn num(&self) -> f64 {
                match self {
                    Value::Num(n) => *n,
                    other => panic!("not a number: {other:?}"),
                }
            }
            pub fn str(&self) -> &str {
                match self {
                    Value::Str(s) => s,
                    other => panic!("not a string: {other:?}"),
                }
            }
            pub fn boolean(&self) -> bool {
                match self {
                    Value::Bool(b) => *b,
                    other => panic!("not a bool: {other:?}"),
                }
            }
        }

        pub fn parse(s: &str) -> Value {
            let bytes = s.as_bytes();
            let mut pos = 0;
            let v = value(bytes, &mut pos);
            skip_ws(bytes, &mut pos);
            assert_eq!(pos, bytes.len(), "trailing JSON at {pos} in {s}");
            v
        }

        fn skip_ws(b: &[u8], p: &mut usize) {
            while *p < b.len() && (b[*p] as char).is_whitespace() {
                *p += 1;
            }
        }

        fn value(b: &[u8], p: &mut usize) -> Value {
            skip_ws(b, p);
            match b[*p] {
                b'{' => {
                    *p += 1;
                    let mut m = BTreeMap::new();
                    skip_ws(b, p);
                    if b[*p] == b'}' {
                        *p += 1;
                        return Value::Obj(m);
                    }
                    loop {
                        skip_ws(b, p);
                        let k = match value(b, p) {
                            Value::Str(s) => s,
                            other => panic!("object key must be a string: {other:?}"),
                        };
                        skip_ws(b, p);
                        assert_eq!(b[*p], b':');
                        *p += 1;
                        m.insert(k, value(b, p));
                        skip_ws(b, p);
                        match b[*p] {
                            b',' => *p += 1,
                            b'}' => {
                                *p += 1;
                                return Value::Obj(m);
                            }
                            c => panic!("unexpected {:?} in object", c as char),
                        }
                    }
                }
                b'[' => {
                    *p += 1;
                    let mut v = Vec::new();
                    skip_ws(b, p);
                    if b[*p] == b']' {
                        *p += 1;
                        return Value::Arr(v);
                    }
                    loop {
                        v.push(value(b, p));
                        skip_ws(b, p);
                        match b[*p] {
                            b',' => *p += 1,
                            b']' => {
                                *p += 1;
                                return Value::Arr(v);
                            }
                            c => panic!("unexpected {:?} in array", c as char),
                        }
                    }
                }
                b'"' => {
                    *p += 1;
                    let mut s = String::new();
                    while b[*p] != b'"' {
                        if b[*p] == b'\\' {
                            *p += 1;
                        }
                        s.push(b[*p] as char);
                        *p += 1;
                    }
                    *p += 1;
                    Value::Str(s)
                }
                b't' => {
                    *p += 4;
                    Value::Bool(true)
                }
                b'f' => {
                    *p += 5;
                    Value::Bool(false)
                }
                b'n' => {
                    *p += 4;
                    Value::Null
                }
                _ => {
                    let start = *p;
                    while *p < b.len()
                        && matches!(b[*p], b'0'..=b'9' | b'-' | b'+' | b'.' | b'e' | b'E')
                    {
                        *p += 1;
                    }
                    Value::Num(std::str::from_utf8(&b[start..*p]).unwrap().parse().unwrap())
                }
            }
        }
    }

    fn cube(half: f64) -> WasmShape {
        WasmShape::box3(half, half, half)
    }

    #[test]
    fn add_instance_places_and_counts() {
        let mut asm = WasmAssembly::new();
        let part = cube(0.5);
        let i0 = asm
            .add_instance(&part, vec![0.0; 3], vec![0.0; 3], 0.0, true, 1.0, "base")
            .unwrap();
        let i1 = asm
            .add_instance(
                &part,
                vec![2.0, 0.0, 0.0],
                vec![0.0; 3],
                0.0,
                false,
                1.0,
                "top",
            )
            .unwrap();
        assert_eq!((i0, i1), (0, 1));
        assert_eq!(asm.instance_count(), 2);

        let t = parse(&asm.transforms());
        assert_eq!(t.len(), 2);
        assert_eq!(t.idx(1).get("translation").idx(0).num(), 2.0);
    }

    #[test]
    fn same_shape_object_shares_one_part() {
        let mut asm = WasmAssembly::new();
        let bolt = cube(0.5);
        asm.add_instance(&bolt, vec![0.0; 3], vec![0.0; 3], 0.0, false, 1.0, "a")
            .unwrap();
        asm.add_instance(
            &bolt,
            vec![5.0, 0.0, 0.0],
            vec![0.0; 3],
            0.0,
            false,
            1.0,
            "b",
        )
        .unwrap();
        assert_eq!(asm.parts.len(), 1);
        assert!(Arc::ptr_eq(
            &asm.inner.instances()[0].part,
            &asm.inner.instances()[1].part
        ));
    }

    #[test]
    fn bad_placement_and_density_are_rejected() {
        let mut asm = WasmAssembly::new();
        let part = cube(0.5);
        assert!(
            asm.add_instance(&part, vec![0.0, 0.0], vec![0.0; 3], 0.0, false, 1.0, "")
                .is_err(),
            "short translation"
        );
        assert!(
            asm.add_instance(&part, vec![0.0; 3], vec![0.0; 3], 90.0, false, 1.0, "")
                .is_err(),
            "zero axis with non-zero angle"
        );
        assert!(
            asm.add_instance(&part, vec![0.0; 3], vec![0.0; 3], 0.0, false, -1.0, "")
                .is_err(),
            "negative density"
        );
        assert_eq!(asm.instance_count(), 0);
    }

    #[test]
    fn add_mate_validates_indices_kind_and_features() {
        let mut asm = WasmAssembly::new();
        let part = cube(0.5);
        asm.add_instance(&part, vec![0.0; 3], vec![0.0; 3], 0.0, true, 1.0, "a")
            .unwrap();
        asm.add_instance(
            &part,
            vec![2.0, 0.0, 0.0],
            vec![0.0; 3],
            0.0,
            false,
            1.0,
            "b",
        )
        .unwrap();

        let plane = |i: usize| (i, "plane", vec![0.0, 0.0, 0.5], vec![0.0, 0.0, 1.0]);
        // Out-of-range instance.
        let (bi, bf, bp, bd) = plane(7);
        let err = asm
            .add_mate(
                "coincident",
                0,
                "plane",
                vec![0.0; 3],
                vec![0.0, 0.0, 1.0],
                bi,
                bf,
                bp,
                bd,
                None,
            )
            .unwrap_err();
        assert!(err.contains("instance 7"), "{err}");

        // Unknown kinds and feature kinds.
        assert!(
            asm.add_mate(
                "weld",
                0,
                "plane",
                vec![0.0; 3],
                vec![0.0, 0.0, 1.0],
                1,
                "plane",
                vec![0.0; 3],
                vec![0.0, 0.0, 1.0],
                None
            )
            .unwrap_err()
            .contains("unknown mate kind"),
        );
        assert!(
            asm.add_mate(
                "coincident",
                0,
                "edge",
                vec![0.0; 3],
                vec![0.0, 0.0, 1.0],
                1,
                "plane",
                vec![0.0; 3],
                vec![0.0, 0.0, 1.0],
                None
            )
            .unwrap_err()
            .contains("unknown feature kind"),
        );

        // Feature pairing rules come from the kernel: concentric needs axes.
        assert!(
            asm.add_mate(
                "concentric",
                0,
                "plane",
                vec![0.0; 3],
                vec![0.0, 0.0, 1.0],
                1,
                "plane",
                vec![0.0; 3],
                vec![0.0, 0.0, 1.0],
                None
            )
            .unwrap_err()
            .contains("axis"),
        );

        // Distance requires a value.
        assert!(
            asm.add_mate(
                "distance",
                0,
                "plane",
                vec![0.0; 3],
                vec![0.0, 0.0, 1.0],
                1,
                "plane",
                vec![0.0; 3],
                vec![0.0, 0.0, 1.0],
                None
            )
            .unwrap_err()
            .contains("value"),
        );

        // A valid mate lands.
        let idx = asm
            .add_mate(
                "coincident",
                0,
                "plane",
                vec![0.0, 0.0, 0.5],
                vec![0.0, 0.0, 1.0],
                1,
                "plane",
                vec![0.0, 0.0, -0.5],
                vec![0.0, 0.0, -1.0],
                None,
            )
            .unwrap();
        assert_eq!(idx, 0);
    }

    #[test]
    fn solve_seats_a_stacked_cube() {
        // Ground cube [-0.5, 0.5]³; floating cube starts far away and mates
        // its bottom face flush to the ground's top face.
        let mut asm = WasmAssembly::new();
        let part = cube(0.5);
        asm.add_instance(&part, vec![0.0; 3], vec![0.0; 3], 0.0, true, 1.0, "base")
            .unwrap();
        asm.add_instance(
            &part,
            vec![7.0, 3.0, 9.0],
            vec![0.0; 3],
            0.0,
            false,
            1.0,
            "lid",
        )
        .unwrap();
        asm.add_mate(
            "coincident",
            0,
            "plane",
            vec![0.0, 0.0, 0.5],
            vec![0.0, 0.0, 1.0],
            1,
            "plane",
            vec![0.0, 0.0, -0.5],
            vec![0.0, 0.0, -1.0],
            None,
        )
        .unwrap();

        let result = parse(&asm.solve());
        assert_eq!(result.get("status").str(), "converged");
        assert!(result.get("residualNorm").num() < 1e-8);
        // A single plane-plane mate leaves in-plane translation (2) and spin
        // about the normal (1) free.
        assert_eq!(result.get("freeDof").num(), 3.0);
        // The lid's bottom face is now at z = 0.5: its center sits at z = 1.
        let lid_z = result
            .get("transforms")
            .idx(1)
            .get("translation")
            .idx(2)
            .num();
        assert!((lid_z - 1.0).abs() < 1e-8, "lid center z = {lid_z}");
        // The fixed base did not move.
        let base_t = result.get("transforms").idx(0).get("translation");
        assert_eq!(base_t.idx(0).num(), 0.0);
    }

    #[test]
    fn interferences_report_overlap_and_clear() {
        let mut asm = WasmAssembly::new();
        let part = cube(0.5);
        asm.add_instance(&part, vec![0.0; 3], vec![0.0; 3], 0.0, true, 1.0, "a")
            .unwrap();
        // Overlaps half the cube: shared volume 0.5.
        asm.add_instance(
            &part,
            vec![0.5, 0.0, 0.0],
            vec![0.0; 3],
            0.0,
            false,
            1.0,
            "b",
        )
        .unwrap();
        // Far away: clear.
        asm.add_instance(
            &part,
            vec![10.0, 0.0, 0.0],
            vec![0.0; 3],
            0.0,
            false,
            1.0,
            "c",
        )
        .unwrap();

        let report = parse(&asm.interferences());
        assert!(report.get("interferes").boolean());
        assert_eq!(report.get("checkedPairs").num(), 3.0);
        assert_eq!(report.get("pairs").len(), 1);
        let pair = report.get("pairs").idx(0);
        assert_eq!(pair.get("a").num(), 0.0);
        assert_eq!(pair.get("b").num(), 1.0);
        assert_eq!(pair.get("aName").str(), "a");
        let vol = pair.get("volume").num();
        assert!((vol - 0.5).abs() < 0.05, "overlap volume {vol}");
    }

    #[test]
    fn mass_properties_aggregate_two_cubes() {
        let mut asm = WasmAssembly::new();
        let part = cube(0.5);
        asm.add_instance(&part, vec![0.0; 3], vec![0.0; 3], 0.0, true, 1.0, "a")
            .unwrap();
        asm.add_instance(
            &part,
            vec![2.0, 0.0, 0.0],
            vec![0.0; 3],
            0.0,
            false,
            3.0,
            "b",
        )
        .unwrap();

        let mp = parse(&asm.mass_properties());
        // Meshed unit cubes: volume ≈ 1 each (mesh tolerance, not exact).
        let volume = mp.get("volume").num();
        assert!((volume - 2.0).abs() < 0.02, "volume {volume}");
        let mass = mp.get("mass").num();
        assert!((mass - 4.0).abs() < 0.04, "mass {mass}");
        // Centroid pulled toward the dense instance: (1·0 + 3·2)/4 = 1.5.
        let cx = mp.get("centroid").idx(0).num();
        assert!((cx - 1.5).abs() < 0.02, "centroid x = {cx}");
        assert_eq!(mp.get("massErrors").len(), 0);
    }

    #[test]
    fn empty_assembly_reports_errors_not_panics() {
        let asm = WasmAssembly::new();
        let mp = parse(&asm.mass_properties());
        assert!(mp.get("error").str().contains("no instances"));
        assert!(asm.assembled_shape().is_err());
        let report = parse(&asm.interferences());
        assert!(!report.get("interferes").boolean());
        assert_eq!(report.get("checkedPairs").num(), 0.0);
    }

    #[test]
    fn assembled_shape_is_a_normal_shape_at_solved_poses() {
        let mut asm = WasmAssembly::new();
        let part = cube(0.5);
        asm.add_instance(&part, vec![0.0; 3], vec![0.0; 3], 0.0, true, 1.0, "base")
            .unwrap();
        asm.add_instance(
            &part,
            vec![9.0, 9.0, 9.0],
            vec![0.0; 3],
            0.0,
            false,
            1.0,
            "lid",
        )
        .unwrap();
        asm.add_mate(
            "coincident",
            0,
            "plane",
            vec![0.0, 0.0, 0.5],
            vec![0.0, 0.0, 1.0],
            1,
            "plane",
            vec![0.0, 0.0, -0.5],
            vec![0.0, 0.0, -1.0],
            None,
        )
        .unwrap();
        asm.solve();

        let shape = asm.assembled_shape().unwrap();
        // The mate constrains only the seating plane; in-plane translation is
        // free DOF, so the lid keeps x = y = 9 and drops to z = 1. Material at
        // its seated center…
        assert!(shape.distance(9.0, 9.0, 1.0) < 0.0);
        // …and none left where it started.
        assert!(shape.distance(9.0, 9.0, 9.0) > 0.0);
        // The union measures as one closed two-cube solid.
        let measured = parse(&shape.measure(None));
        let volume = measured.get("volume").num();
        assert!((volume - 2.0).abs() < 0.05, "assembled volume {volume}");
    }

    #[test]
    fn rotated_instance_places_geometry_by_axis_angle() {
        // A 1×1×4 slab rotated 90° about x: extent moves from z to y.
        let mut asm = WasmAssembly::new();
        let slab = WasmShape::box3(0.5, 0.5, 2.0);
        asm.add_instance(
            &slab,
            vec![0.0; 3],
            vec![1.0, 0.0, 0.0],
            90.0,
            false,
            1.0,
            "slab",
        )
        .unwrap();
        let shape = asm.assembled_shape().unwrap();
        assert!(shape.distance(0.0, 1.8, 0.0) < 0.0, "material along +y");
        assert!(shape.distance(0.0, 0.0, 1.8) > 0.0, "no material along +z");
    }
}
