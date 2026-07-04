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
//! 2D parameter-space curves (`Fin::pcurve`) are not stored here yet; the
//! SP-curve representation is a later issue.
//!
//! [`Edge::curve`]: crate::topology::Edge::curve
//! [`Face::surface`]: crate::topology::Face::surface

use crate::curve::Curve3;
use crate::surface::Surface3;
use opensolid_core::{Arena, EntityId};

/// Centralized store for geometric definitions, one typed arena per kind.
#[derive(Default)]
pub struct GeometryStore {
    pub curves: Arena<Curve3>,
    pub surfaces: Arena<Surface3>,
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
}
