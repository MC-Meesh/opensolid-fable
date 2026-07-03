//! STL export: binary and ASCII stereolithography writers.
//!
//! Binary layout (little-endian, per the 3D Systems spec):
//!
//! ```text
//! 80 bytes  header (opaque comment, must not start with "solid")
//!  4 bytes  u32 triangle count
//! 50 bytes  per triangle:
//!             3 × f32 facet normal
//!             3 × (3 × f32) vertex positions
//!             u16 attribute byte count (0)
//! ```
//!
//! Total size is exactly `84 + 50 * triangle_count` bytes.

use std::io::{self, Write};

use crate::mesh::TriangleMesh;
use opensolid_core::types::Vector3;

/// Fixed 80-byte binary header tag (zero-padded). Must not begin with
/// "solid", which some readers use to sniff ASCII STL.
const BINARY_HEADER_TAG: &[u8] = b"OpenSolid binary STL";

/// Geometric facet normal of triangle `tri` via the right-hand rule;
/// zero vector for degenerate triangles.
fn facet_normal(mesh: &TriangleMesh, tri: [usize; 3]) -> Vector3 {
    let [a, b, c] = tri.map(|i| mesh.positions[i]);
    let n = (b - a).cross(&(c - a));
    let norm = n.norm();
    if norm > 1e-20 {
        n / norm
    } else {
        Vector3::zeros()
    }
}

fn write_f32_triple<W: Write>(writer: &mut W, x: f64, y: f64, z: f64) -> io::Result<()> {
    writer.write_all(&(x as f32).to_le_bytes())?;
    writer.write_all(&(y as f32).to_le_bytes())?;
    writer.write_all(&(z as f32).to_le_bytes())
}

/// Write `mesh` as binary STL.
///
/// Facet normals are recomputed from vertex positions; coordinates are
/// narrowed to `f32` as the format requires. Fails with
/// [`io::ErrorKind::InvalidInput`] if the mesh has more than `u32::MAX`
/// triangles. Panics if any index is out of bounds.
pub fn write_stl_binary<W: Write>(mesh: &TriangleMesh, writer: &mut W) -> io::Result<()> {
    let count = u32::try_from(mesh.triangle_count()).map_err(|_| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            "mesh exceeds u32::MAX triangles, not representable in binary STL",
        )
    })?;

    let mut header = [0u8; 80];
    header[..BINARY_HEADER_TAG.len()].copy_from_slice(BINARY_HEADER_TAG);
    writer.write_all(&header)?;
    writer.write_all(&count.to_le_bytes())?;

    for &tri in &mesh.indices {
        let n = facet_normal(mesh, tri);
        write_f32_triple(writer, n.x, n.y, n.z)?;
        for &i in &tri {
            let p = mesh.positions[i];
            write_f32_triple(writer, p.x, p.y, p.z)?;
        }
        // Attribute byte count: always zero (nonzero values are
        // vendor-specific and widely misinterpreted).
        writer.write_all(&0u16.to_le_bytes())?;
    }
    Ok(())
}

/// Write `mesh` as ASCII STL under the solid name `name`.
///
/// Facet normals are recomputed from vertex positions. Coordinates keep full
/// `f64` precision using Rust's shortest round-trip float formatting. `name`
/// should be a single token without newlines. Panics if any index is out of
/// bounds.
pub fn write_stl_ascii<W: Write>(
    mesh: &TriangleMesh,
    writer: &mut W,
    name: &str,
) -> io::Result<()> {
    writeln!(writer, "solid {name}")?;
    for &tri in &mesh.indices {
        let n = facet_normal(mesh, tri);
        writeln!(writer, "  facet normal {} {} {}", n.x, n.y, n.z)?;
        writeln!(writer, "    outer loop")?;
        for &i in &tri {
            let p = mesh.positions[i];
            writeln!(writer, "      vertex {} {} {}", p.x, p.y, p.z)?;
        }
        writeln!(writer, "    endloop")?;
        writeln!(writer, "  endfacet")?;
    }
    writeln!(writer, "endsolid {name}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::test_meshes::unit_box;
    use opensolid_core::types::Point3;

    #[test]
    fn binary_golden_layout_for_unit_box() {
        let mesh = unit_box();
        let mut buf = Vec::new();
        write_stl_binary(&mesh, &mut buf).unwrap();

        // 84-byte preamble + 50 bytes per triangle.
        assert_eq!(buf.len(), 84 + 50 * 12);
        assert_eq!(buf.len(), 684);

        // Header: tag then zero padding; must not start with "solid".
        assert!(buf.starts_with(BINARY_HEADER_TAG));
        assert!(buf[BINARY_HEADER_TAG.len()..80].iter().all(|&b| b == 0));
        assert_ne!(&buf[..5], b"solid");

        // Little-endian u32 triangle count.
        assert_eq!(u32::from_le_bytes(buf[80..84].try_into().unwrap()), 12);

        // First facet normal is the -z box face: (0, 0, -1) as f32 LE.
        let f = |off: usize| f32::from_le_bytes(buf[off..off + 4].try_into().unwrap());
        assert_eq!((f(84), f(88), f(92)), (0.0, 0.0, -1.0));

        // First vertex of the first facet is the box origin.
        assert_eq!((f(96), f(100), f(104)), (0.0, 0.0, 0.0));

        // Every attribute byte count is zero.
        for t in 0..12 {
            let off = 84 + t * 50 + 48;
            assert_eq!(&buf[off..off + 2], &[0, 0], "attribute bytes, facet {t}");
        }
    }

    #[test]
    fn binary_empty_mesh_is_preamble_only() {
        let mut buf = Vec::new();
        write_stl_binary(&TriangleMesh::new(), &mut buf).unwrap();
        assert_eq!(buf.len(), 84);
        assert_eq!(u32::from_le_bytes(buf[80..84].try_into().unwrap()), 0);
    }

    /// Minimal ASCII STL parser, for testing our own output only.
    fn parse_ascii_stl(text: &str) -> (String, Vec<(Vector3, [Point3; 3])>) {
        let floats = |line: &str, skip: usize| -> Vec<f64> {
            line.split_whitespace()
                .skip(skip)
                .map(|tok| tok.parse().expect("float token"))
                .collect()
        };
        let mut lines = text.lines();
        let name = lines
            .next()
            .expect("solid line")
            .strip_prefix("solid ")
            .expect("starts with 'solid '")
            .to_string();

        let mut facets = Vec::new();
        let mut normal = None;
        let mut vertices: Vec<Point3> = Vec::new();
        for line in lines {
            let trimmed = line.trim_start();
            if let Some(rest) = trimmed.strip_prefix("facet normal ") {
                let v = floats(rest, 0);
                normal = Some(Vector3::new(v[0], v[1], v[2]));
            } else if let Some(rest) = trimmed.strip_prefix("vertex ") {
                let v = floats(rest, 0);
                vertices.push(Point3::new(v[0], v[1], v[2]));
            } else if trimmed == "endfacet" {
                let verts: [Point3; 3] = std::mem::take(&mut vertices).try_into().unwrap();
                facets.push((normal.take().expect("normal before endfacet"), verts));
            }
        }
        (name, facets)
    }

    #[test]
    fn ascii_round_trips_through_own_parser() {
        let mesh = unit_box();
        let mut buf = Vec::new();
        write_stl_ascii(&mesh, &mut buf, "box").unwrap();
        let text = String::from_utf8(buf).unwrap();

        assert!(text.starts_with("solid box\n"));
        assert!(text.ends_with("endsolid box\n"));

        let (name, facets) = parse_ascii_stl(&text);
        assert_eq!(name, "box");
        assert_eq!(facets.len(), 12);

        for (t, &(normal, verts)) in facets.iter().enumerate() {
            // Full-precision f64 formatting round-trips positions exactly.
            for (k, vert) in verts.iter().enumerate() {
                assert_eq!(
                    *vert, mesh.positions[mesh.indices[t][k]],
                    "facet {t} vertex {k}"
                );
            }
            // Written facet normal matches the geometric one and is unit length.
            let expected = facet_normal(&mesh, mesh.indices[t]);
            assert!((normal - expected).norm() < 1e-15, "facet {t} normal");
            assert!((normal.norm() - 1.0).abs() < 1e-12, "facet {t} unit length");
        }
    }

    #[test]
    fn facet_normal_recomputed_not_trusted() {
        // Give the mesh deliberately wrong stored normals; STL output must
        // still carry the geometric facet normal.
        let mut mesh = unit_box();
        for n in &mut mesh.normals {
            *n = Vector3::new(9.0, 9.0, 9.0);
        }
        let mut buf = Vec::new();
        write_stl_ascii(&mesh, &mut buf, "box").unwrap();
        let (_, facets) = parse_ascii_stl(&String::from_utf8(buf).unwrap());
        assert_eq!(facets[0].0, Vector3::new(0.0, 0.0, -1.0));
    }

    #[test]
    fn degenerate_triangle_gets_zero_normal() {
        let p = Point3::new(1.0, 2.0, 3.0);
        let mesh = TriangleMesh {
            positions: vec![p, p, p],
            normals: vec![Vector3::zeros(); 3],
            indices: vec![[0, 1, 2]],
        };
        assert_eq!(facet_normal(&mesh, [0, 1, 2]), Vector3::zeros());
        let mut buf = Vec::new();
        write_stl_binary(&mesh, &mut buf).unwrap();
        assert_eq!(buf.len(), 84 + 50);
    }
}
