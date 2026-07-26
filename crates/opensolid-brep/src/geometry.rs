//! Geometry store: arenas for the parametric geometry that topology
//! references (`spec/03-topology.md` §3 pairs every `TopologyStore` with a
//! `GeometryStore`).
//!
//! Topology and geometry are deliberately separate stores: many topological
//! entities can share one geometric definition (both fins of a seam edge,
//! faces split during booleans), and operations that only rewire
//! connectivity never need to touch geometry. [`Edge::curve`] and
//! [`Face::surface`] hold [`EntityId`]s into these arenas.
//!
//! 2D parameter-space curves live here too ([`Curve2`], backing
//! [`Fin::pcurve`]). They sit in their own arena rather than alongside the
//! 3D curves because they are a different kind of thing: a pcurve is only
//! meaningful relative to the surface whose parameter space it lives in, and
//! sharing follows fin-level trim rather than edge-level geometry — the two
//! fins of a seam edge share one [`Curve3`] but need *different* pcurves
//! (see [`crate::pcurve`]).
//!
//! [`Edge::curve`]: crate::topology::Edge::curve
//! [`Face::surface`]: crate::topology::Face::surface
//! [`Fin::pcurve`]: crate::topology::Fin::pcurve

use crate::curve::Curve3;
use crate::pcurve::Curve2;
use crate::surface::Surface3;
use opensolid_core::{Arena, EntityId};

/// Centralized store for geometric definitions, one typed arena per kind.
#[derive(Default)]
pub struct GeometryStore {
    pub curves: Arena<Curve3>,
    pub surfaces: Arena<Surface3>,
    pub pcurves: Arena<Curve2>,
}

impl GeometryStore {
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a curve and return its id.
    pub fn add_curve(&mut self, curve: Curve3) -> EntityId<Curve3> {
        self.curves.insert(curve)
    }

    /// Insert a surface and return its id.
    pub fn add_surface(&mut self, surface: Surface3) -> EntityId<Surface3> {
        self.surfaces.insert(surface)
    }

    /// Look up a curve. `None` if the id is stale.
    pub fn curve(&self, id: EntityId<Curve3>) -> Option<&Curve3> {
        self.curves.get(id)
    }

    /// Look up a surface. `None` if the id is stale.
    pub fn surface(&self, id: EntityId<Surface3>) -> Option<&Surface3> {
        self.surfaces.get(id)
    }

    /// Insert a 2D parameter-space curve and return its id.
    pub fn add_pcurve(&mut self, pcurve: Curve2) -> EntityId<Curve2> {
        self.pcurves.insert(pcurve)
    }

    /// Look up a pcurve. `None` if the id is stale.
    pub fn pcurve(&self, id: EntityId<Curve2>) -> Option<&Curve2> {
        self.pcurves.get(id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_core::{Point3, Vector3};

    #[test]
    fn store_round_trips_curves_and_surfaces() {
        let mut geo = GeometryStore::new();
        let line = Curve3::line(Point3::origin(), Vector3::x()).expect("valid line");
        let plane = Surface3::plane(Point3::origin(), Vector3::z()).expect("valid plane");

        let curve_id = geo.add_curve(line.clone());
        let surface_id = geo.add_surface(plane.clone());

        assert_eq!(geo.curve(curve_id), Some(&line));
        assert_eq!(geo.surface(surface_id), Some(&plane));

        geo.curves.remove(curve_id);
        assert_eq!(geo.curve(curve_id), None);
        assert_eq!(geo.surface(surface_id), Some(&plane));
    }

    #[test]
    fn store_round_trips_pcurves_independently_of_curves() {
        use crate::pcurve::Curve2;
        use opensolid_core::types::{Point2, Vector2};

        let mut geo = GeometryStore::new();
        let curve = Curve3::line(Point3::origin(), Vector3::x()).expect("valid line");
        let pcurve = Curve2::line(Point2::origin(), Vector2::x()).expect("valid pcurve");

        let curve_id = geo.add_curve(curve);
        let pcurve_id = geo.add_pcurve(pcurve.clone());
        assert_eq!(geo.pcurve(pcurve_id), Some(&pcurve));

        // Separate arenas: retiring an edge's 3D curve leaves the fins'
        // trim geometry addressable (and vice versa).
        geo.curves.remove(curve_id);
        assert_eq!(geo.pcurve(pcurve_id), Some(&pcurve));
    }
}
