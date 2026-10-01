// Timing mirror for quadextractor: regenerates the bit-identical large
// input the C++ quadextractor_diff_dump tool times (32x32 triangulated
// grid, pure formula, UVs at half scale, vertex UVs on) and extracts it
// 3 times. Prints one `T quadext ...` line per sample in the same shape
// as the C++ tool; only the elapsed ms are compared across sides.
// Run release: cargo test --release -p retopo_core --test quad_extractor_timing
use retopo_core::quad_extractor::QuadExtractor;
use retopo_core::vector2::Vector2;
use retopo_core::vector3::Vector3;
use std::time::Instant;

fn make_grid(w: usize, h: usize) -> (Vec<Vector3>, Vec<Vec<usize>>, Vec<Vec<Vector2>>) {
    let id = |x: usize, y: usize| y * (w + 1) + x;
    let mut vertices = Vec::new();
    for y in 0..=h {
        for x in 0..=w {
            vertices.push(Vector3::new(x as f64, y as f64, 0.0));
        }
    }
    let mut triangles = Vec::new();
    for y in 0..h {
        for x in 0..w {
            let a = id(x, y);
            let b = id(x + 1, y);
            let c = id(x + 1, y + 1);
            let d = id(x, y + 1);
            triangles.push(vec![a, b, c]);
            triangles.push(vec![a, c, d]);
        }
    }
    let mut uvs = Vec::with_capacity(triangles.len());
    for t in &triangles {
        let mut row = Vec::with_capacity(3);
        for k in 0..3 {
            let v = vertices[t[k]];
            row.push(Vector2::new(v.x() * 0.5, v.y() * 0.5));
        }
        uvs.push(row);
    }
    (vertices, triangles, uvs)
}

#[test]
fn timing_large_extract() {
    for sample in 0..3 {
        let (vertices, triangles, uvs) = make_grid(32, 32);
        let mut extractor = QuadExtractor::new(&vertices, &triangles, &uvs);
        extractor.set_compute_vertex_uvs(true);
        let t0 = Instant::now();
        let ok = extractor.extract();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        println!(
            "T quadext sample={sample} tris={} ok={} verts={} quads={} ms={:.3}",
            triangles.len(),
            if ok { 1 } else { 0 },
            extractor.remeshed_vertices().len(),
            extractor.remeshed_quads().len(),
            ms
        );
        assert!(ok, "timing extract failed");
    }
}
