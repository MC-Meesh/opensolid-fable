//! Wavefront OBJ export: indexed positions + per-vertex normals.

use std::io::{self, Write};

use crate::mesh::TriangleMesh;

/// Write `mesh` as Wavefront OBJ.
///
/// Emits one `v` line per position, one `vn` line per normal (the arrays are
/// parallel, so normal *k* belongs to vertex *k*), then one `f` line per
/// triangle using 1-based `index//index` references (position and normal
/// indices coincide). Coordinates keep full `f64` precision using Rust's
/// shortest round-trip float formatting. Panics if any index is out of
/// bounds.
pub fn write_obj<W: Write>(mesh: &TriangleMesh, writer: &mut W) -> io::Result<()> {
    writeln!(writer, "# OpenSolid OBJ export")?;
    writeln!(
        writer,
        "# {} vertices, {} triangles",
        mesh.vertex_count(),
        mesh.triangle_count()
    )?;
    for p in &mesh.positions {
        writeln!(writer, "v {} {} {}", p.x, p.y, p.z)?;
    }
    for n in &mesh.normals {
        writeln!(writer, "vn {} {} {}", n.x, n.y, n.z)?;
    }
    for tri in &mesh.indices {
        writeln!(
            writer,
            "f {a}//{a} {b}//{b} {c}//{c}",
            a = tri[0] + 1,
            b = tri[1] + 1,
            c = tri[2] + 1,
        )?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::test_meshes::unit_box;

    #[test]
    fn line_format_spot_checks() {
        let mesh = unit_box();
        let mut buf = Vec::new();
        write_obj(&mesh, &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        let lines: Vec<&str> = text.lines().collect();

        assert_eq!(lines[0], "# OpenSolid OBJ export");
        assert_eq!(lines[1], "# 36 vertices, 12 triangles");

        // Soup mesh: 36 positions, 36 normals, 12 faces, plus 2 comments.
        assert_eq!(lines.iter().filter(|l| l.starts_with("v ")).count(), 36);
        assert_eq!(lines.iter().filter(|l| l.starts_with("vn ")).count(), 36);
        assert_eq!(lines.iter().filter(|l| l.starts_with("f ")).count(), 12);
        assert_eq!(lines.len(), 2 + 36 + 36 + 12);

        // Sections appear in v, vn, f order.
        let first = |prefix: &str| lines.iter().position(|l| l.starts_with(prefix)).unwrap();
        assert!(first("v ") < first("vn "));
        assert!(first("vn ") < first("f "));

        // First vertex is the box origin; shortest-round-trip f64 formatting
        // renders integral coordinates without a decimal point.
        assert_eq!(lines[2], "v 0 0 0");
        // First triangle is the first three soup vertices, 1-based, with
        // position//normal indices matching.
        assert_eq!(
            *lines.iter().find(|l| l.starts_with("f ")).unwrap(),
            "f 1//1 2//2 3//3"
        );
        assert_eq!(*lines.last().unwrap(), "f 34//34 35//35 36//36");

        // First normal line is the -z face normal.
        assert_eq!(
            *lines.iter().find(|l| l.starts_with("vn ")).unwrap(),
            "vn 0 0 -1"
        );
    }

    #[test]
    fn welded_mesh_references_shared_vertices() {
        let welded = unit_box().weld(0.0);
        assert_eq!(welded.vertex_count(), 8);

        let mut buf = Vec::new();
        write_obj(&welded, &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();

        assert_eq!(text.lines().filter(|l| l.starts_with("v ")).count(), 8);
        assert_eq!(text.lines().filter(|l| l.starts_with("vn ")).count(), 8);

        // Every face reference stays within the 8 shared vertices.
        for line in text.lines().filter(|l| l.starts_with("f ")) {
            for token in line.split_whitespace().skip(1) {
                let (pos, norm) = token.split_once("//").expect("index//index format");
                let pos: usize = pos.parse().unwrap();
                assert_eq!(pos.to_string(), norm, "position and normal indices match");
                assert!((1..=8).contains(&pos), "index {pos} out of range");
            }
        }
    }

    #[test]
    fn empty_mesh_writes_header_only() {
        let mut buf = Vec::new();
        write_obj(&TriangleMesh::new(), &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        assert_eq!(text, "# OpenSolid OBJ export\n# 0 vertices, 0 triangles\n");
    }
}
