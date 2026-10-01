//! Replica goldens for the glb port: module-level transcription of the
//! `tests/test_cli_glb.cpp` shape contracts (writer output re-parses with
//! accessor counts matching the source mesh, empty output fails loudly,
//! truncated input fails) plus the direct API contracts (extension checks,
//! uvs overload, no-clear warn/err, `None` out-params). Any divergence is a
//! port defect.

use retopo_core::glb::{
    has_glb_extension, is_supported_input_extension, load_glb_positions_and_triangles, save_glb,
    save_glb_with_uvs,
};
use retopo_core::vector2::Vector2;
use retopo_core::vector3::Vector3;
use std::sync::atomic::{AtomicUsize, Ordering};

static TEMP_COUNTER: AtomicUsize = AtomicUsize::new(0);

fn temp_path(suffix: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!(
        "retopo_rs_glbgolden_{}_{}{suffix}",
        std::process::id(),
        TEMP_COUNTER.fetch_add(1, Ordering::SeqCst)
    ))
}

fn fan_tris(faces: &[Vec<usize>]) -> Vec<Vec<usize>> {
    let mut tris = Vec::new();
    for face in faces {
        if face.len() < 3 {
            continue;
        }
        for i in 1..face.len() - 1 {
            tris.push(vec![face[0], face[i], face[i + 1]]);
        }
    }
    tris
}

fn quad_mesh() -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let verts = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let faces = vec![vec![0, 1, 2, 3], vec![0, 2, 1]];
    (verts, faces)
}

/// Minimal GLB validation mirroring `readGlbShape` in test_cli_glb.cpp:
// magic, version 2, total length, JSON chunk type, first two accessor counts.
fn read_glb_shape(bytes: &[u8]) -> Option<(usize, usize)> {
    if bytes.len() < 20 || &bytes[0..4] != b"glTF" {
        return None;
    }
    let version = u32::from_le_bytes(bytes[4..8].try_into().ok()?);
    let total = u32::from_le_bytes(bytes[8..12].try_into().ok()?) as usize;
    let json_len = u32::from_le_bytes(bytes[12..16].try_into().ok()?) as usize;
    let json_type = u32::from_le_bytes(bytes[16..20].try_into().ok()?);
    if version != 2 || total != bytes.len() || json_type != 0x4E4F534A {
        return None;
    }
    let json = std::str::from_utf8(bytes.get(20..20 + json_len)?).ok()?;
    let first = json.find("\"count\":")?;
    let second = json[first + 1..].find("\"count\":")? + first + 1;
    let verts = json[first + 8..].split(['}', ',']).next()?.parse().ok()?;
    let indices = json[second + 8..].split(['}', ',']).next()?.parse().ok()?;
    Some((verts, indices))
}

#[test]
fn extension_checks() {
    assert!(has_glb_extension("model.glb"));
    assert!(has_glb_extension("model.GLB"));
    assert!(has_glb_extension("MODEL.GlB"));
    assert!(!has_glb_extension("model.obj"));
    assert!(!has_glb_extension("model"));
    assert!(!has_glb_extension("model."));
    assert!(has_glb_extension(".glb"));
    assert!(!has_glb_extension("dir.glb/file"));
    assert!(is_supported_input_extension("a.obj"));
    assert!(is_supported_input_extension("a.OBJ"));
    assert!(is_supported_input_extension("a.glb"));
    assert!(!is_supported_input_extension("a.txt"));
    assert!(!is_supported_input_extension(""));
}

#[test]
fn writer_output_reparses_with_matching_counts() {
    // CLI test (b): accessor counts match the source mesh.
    let (verts, faces) = quad_mesh();
    let path = temp_path(".glb");
    assert!(save_glb(&path, "golden", &verts, &faces));
    let bytes = std::fs::read(&path).expect("read back");
    let _ = std::fs::remove_file(&path);
    let (vc, ic) = read_glb_shape(&bytes).expect("valid glb");
    assert_eq!(vc, verts.len());
    assert_eq!(ic, fan_tris(&faces).len() * 3);
    assert_eq!(ic % 3, 0);
}

#[test]
fn writer_uvs_twin_keeps_geometry_identical() {
    // CLI --uvs contract at module level: TEXCOORD_0 appears, and the
    // positions+indices bytes match the flag-off output exactly.
    let (verts, faces) = quad_mesh();
    let uvs = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(1.0, 0.0),
        Vector2::new(1.0, 1.0),
        Vector2::new(0.0, 1.0),
    ];
    let plain_path = temp_path(".glb");
    let uvs_path = temp_path(".glb");
    assert!(save_glb(&plain_path, "golden", &verts, &faces));
    assert!(save_glb_with_uvs(&uvs_path, "golden", &verts, &faces, &uvs));
    let plain = std::fs::read(&plain_path).expect("read back");
    let with_uvs = std::fs::read(&uvs_path).expect("read back");
    let _ = std::fs::remove_file(&plain_path);
    let _ = std::fs::remove_file(&uvs_path);
    assert!(with_uvs.len() > plain.len());
    // UV bytes append after indices: find the shared BIN prefix.
    let json_len = u32::from_le_bytes(plain[12..16].try_into().unwrap()) as usize;
    let ujson_len = u32::from_le_bytes(with_uvs[12..16].try_into().unwrap()) as usize;
    let plain_bin = &plain[20 + json_len + 8..];
    let uvs_bin = &with_uvs[20 + ujson_len + 8..];
    assert!(uvs_bin.starts_with(plain_bin));
    let ujson = std::str::from_utf8(&with_uvs[20..20 + ujson_len]).unwrap();
    assert!(ujson.contains("TEXCOORD_0"));
}

#[test]
fn empty_output_fails_loudly() {
    // CLI test (f): no silent 0-vertex success file.
    let path = temp_path(".glb");
    assert!(!save_glb(&path, "golden", &[], &[]));
    assert!(!std::fs::exists(&path).unwrap());
    let verts = vec![Vector3::new(0.0, 0.0, 0.0)];
    assert!(!save_glb(&path, "golden", &verts, &[vec![0]]));
    assert!(!std::fs::exists(&path).unwrap());
}

#[test]
fn out_of_range_index_fails() {
    let (verts, _) = quad_mesh();
    let path = temp_path(".glb");
    assert!(!save_glb(&path, "golden", &verts, &[vec![0, 1, 9]]));
    assert!(!std::fs::exists(&path).unwrap());
}

#[test]
fn uvs_length_mismatch_fails() {
    let (verts, faces) = quad_mesh();
    let path = temp_path(".glb");
    assert!(!save_glb_with_uvs(
        &path,
        "golden",
        &verts,
        &faces,
        &[Vector2::new(0.0, 0.0)]
    ));
    assert!(!std::fs::exists(&path).unwrap());
}

#[test]
fn tetra_fixture_shape() {
    // CLI test (a) at module level: the committed tetra loads sane.
    let path =
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/fixtures/tetra.glb");
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let mut warn = String::new();
    let mut err = String::new();
    assert!(load_glb_positions_and_triangles(
        &path,
        &mut positions,
        &mut triangles,
        Some(&mut warn),
        Some(&mut err),
    ));
    assert!(err.is_empty());
    assert!(!positions.is_empty());
    assert!(!triangles.is_empty());
    assert_eq!(positions.len() % 3, 0);
    let nverts = positions.len() / 3;
    for tri in &triangles {
        assert_eq!(tri.len(), 3);
        assert!(tri.iter().all(|&i| i < nverts));
    }
}

#[test]
fn save_then_load_round_trip() {
    let (verts, faces) = quad_mesh();
    let path = temp_path(".glb");
    assert!(save_glb(&path, "golden", &verts, &faces));
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let mut warn = String::new();
    let mut err = String::new();
    assert!(load_glb_positions_and_triangles(
        &path,
        &mut positions,
        &mut triangles,
        Some(&mut warn),
        Some(&mut err),
    ));
    let _ = std::fs::remove_file(&path);
    let expected: Vec<f32> = verts
        .iter()
        .flat_map(|v| [v.x() as f32, v.y() as f32, v.z() as f32])
        .collect();
    assert_eq!(positions, expected);
    assert_eq!(triangles, fan_tris(&faces));
}

#[test]
fn truncated_input_fails() {
    // CLI test (e): truncated .glb exits 1; here the load returns false.
    let (verts, faces) = quad_mesh();
    let src = temp_path(".glb");
    assert!(save_glb(&src, "golden", &verts, &faces));
    let bytes = std::fs::read(&src).expect("read back");
    let _ = std::fs::remove_file(&src);
    let bad = temp_path(".glb");
    std::fs::write(&bad, &bytes[..100.min(bytes.len())]).expect("write trunc");
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let mut err = String::new();
    assert!(!load_glb_positions_and_triangles(
        &bad,
        &mut positions,
        &mut triangles,
        None,
        Some(&mut err),
    ));
    let _ = std::fs::remove_file(&bad);
    assert!(err.contains("failed to parse GLB file"));
}

#[test]
fn missing_file_fails_with_cleared_outputs() {
    // Failures clear positions/triangles but leave warn/err untouched.
    let mut positions = vec![9.0f32];
    let mut triangles = vec![vec![7]];
    let mut warn = String::from("W");
    let mut err = String::from("E");
    assert!(!load_glb_positions_and_triangles(
        std::path::Path::new("/nonexistent-dir-rs-glb-9f3/does-not-exist.glb"),
        &mut positions,
        &mut triangles,
        Some(&mut warn),
        Some(&mut err),
    ));
    assert!(positions.is_empty());
    assert!(triangles.is_empty());
    assert_eq!(warn, "W");
    assert!(err.contains("failed to parse GLB file"));
}

#[test]
fn no_clear_contract_on_success() {
    // Success without skips leaves warn/err exactly as passed in.
    let (verts, faces) = quad_mesh();
    let path = temp_path(".glb");
    assert!(save_glb(&path, "golden", &verts, &faces));
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let mut warn = String::from("keep-warn");
    let mut err = String::from("keep-err");
    assert!(load_glb_positions_and_triangles(
        &path,
        &mut positions,
        &mut triangles,
        Some(&mut warn),
        Some(&mut err),
    ));
    let _ = std::fs::remove_file(&path);
    assert_eq!(warn, "keep-warn");
    assert_eq!(err, "keep-err");
}

#[test]
fn none_out_params() {
    // `None` warn/err behave like ignored out-params (no crash, same result).
    let (verts, faces) = quad_mesh();
    let path = temp_path(".glb");
    assert!(save_glb(&path, "golden", &verts, &faces));
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    assert!(load_glb_positions_and_triangles(
        &path,
        &mut positions,
        &mut triangles,
        None,
        None
    ));
    assert_eq!(triangles, fan_tris(&faces));
    let _ = std::fs::remove_file(&path);
    let (mut positions, mut triangles) = (vec![9.0f32], vec![vec![7]]);
    assert!(!load_glb_positions_and_triangles(
        std::path::Path::new("/nonexistent-dir-rs-glb-9f3/does-not-exist.glb"),
        &mut positions,
        &mut triangles,
        None,
        None,
    ));
    assert!(positions.is_empty());
    assert!(triangles.is_empty());
}
