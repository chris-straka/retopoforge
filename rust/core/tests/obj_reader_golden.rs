//! Replica goldens for the obj_reader port: 1:1 transcription of the
//! `tests/test_objreader.cpp` groups (loader), the `tests/test_weld.cpp`
//! groups (weld; `MeshSeparator::splitToIslands` is a sibling module, so the
//! two island-count assertions use a small local shared-corner connectivity
//! count instead), and the weld-stats groups from
//! `tests/test_input_validation.cpp`. Any divergence is a port defect.

use retopo_core::obj_reader::{
    WeldStats, load_obj_positions_and_triangles, weld_positions_and_triangles,
};
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};

static TEMP_COUNTER: AtomicUsize = AtomicUsize::new(0);

// Write OBJ text to a temp file, load it, remove the file. Returns the loader
// result; fills positions/triangles/warn/err from the call.
fn load_string(
    obj_text: &str,
    positions: &mut Vec<f32>,
    triangles: &mut Vec<Vec<usize>>,
    warn: &mut String,
    err: &mut String,
) -> bool {
    let path: PathBuf = std::env::temp_dir().join(format!(
        "retopo_rs_objreader_test_{}_{}.obj",
        std::process::id(),
        TEMP_COUNTER.fetch_add(1, Ordering::SeqCst)
    ));
    std::fs::write(&path, obj_text).expect("write temp obj");
    let ok = load_obj_positions_and_triangles(&path, positions, triangles, Some(warn), Some(err));
    let _ = std::fs::remove_file(&path);
    ok
}

// Shared-corner island count (what `MeshSeparator::splitToIslands` computes on
// these inputs; that module is a sibling's, so the weld goldens count locally).
fn island_count(triangles: &[Vec<usize>]) -> usize {
    let n = triangles.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    for i in 0..n {
        for j in (i + 1)..n {
            let shared = triangles[i].iter().any(|c| triangles[j].contains(c));
            if shared {
                let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                parent[a] = b;
            }
        }
    }
    let mut roots: Vec<usize> = (0..n).map(|i| find(&mut parent, i)).collect();
    roots.sort_unstable();
    roots.dedup();
    roots.len()
}

#[test]
fn full_feature_lines_with_ignored_records() {
    // v/vt/vn lines with full v/vt/vn corners; groups, materials, object
    // names, smoothing groups and comments are ignored.
    let obj = "# comment line\n\
                 mtllib test.mtl\n\
                 o TestObject\n\
                 v 0 0 0\n\
                 v 1 0 0\n\
                 v 0 1 0\n\
                 vt 0 0\n\
                 vn 0 0 1\n\
                 vp 0.5\n\
                 g group1\n\
                 usemtl mat1\n\
                 s 1\n\
                 f 1/1/1 2/1/1 3/1/1\n";
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    let (mut warn, mut err) = (String::new(), String::new());
    assert!(load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert!(err.is_empty());
    assert_eq!(positions, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]);
    assert_eq!(triangles, vec![vec![0, 1, 2]]);
    let _ = warn;
}

#[test]
fn corner_forms_are_equivalent() {
    // Corner forms v, v/vt, v//vn and v/vt/vn are equivalent; the vt/vn
    // parts are ignored even when their indices are out of range (9).
    let obj = "v 0 0 0\n\
                 v 2 0 0\n\
                 v 0 3 0\n\
                 f 1 2 3\n\
                 f 1/1 2/1 3/1\n\
                 f 1//1 2//1 3//1\n\
                 f 1/9/9 2/9/9 3/9/9\n";
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    let (mut warn, mut err) = (String::new(), String::new());
    assert!(load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert!(err.is_empty());
    assert_eq!(positions, vec![0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0, 0.0]);
    assert_eq!(
        triangles,
        vec![vec![0, 1, 2], vec![0, 1, 2], vec![0, 1, 2], vec![0, 1, 2]]
    );
}

#[test]
fn negative_relative_indices() {
    // Negative (relative) indices count back from the last vertex.
    let obj = "v 0 0 0\n\
                 v 1 0 0\n\
                 v 1 1 0\n\
                 f -3 -2 -1\n";
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    let (mut warn, mut err) = (String::new(), String::new());
    assert!(load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert!(err.is_empty());
    assert_eq!(positions, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0]);
    assert_eq!(triangles, vec![vec![0, 1, 2]]);
}

#[test]
fn convex_quad_ear_clip() {
    // Convex quad: ear-clip triangulation starting from the first corner.
    let obj = "v 0 0 0\n\
                 v 1 0 0\n\
                 v 1 1 0\n\
                 v 0 1 0\n\
                 f 1 2 3 4\n";
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    let (mut warn, mut err) = (String::new(), String::new());
    assert!(load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert!(err.is_empty());
    assert_eq!(
        positions,
        vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0]
    );
    assert_eq!(triangles, vec![vec![0, 1, 2], vec![0, 2, 3]]);
}

#[test]
fn concave_pentagon_ear_clip() {
    // Concave pentagon (notch at vertex 3 = (1,1)): ear clipping skips the
    // reflex corner (0,2,3) and emits three triangles covering all corners.
    let obj = "v 0 0 0\n\
                 v 3 0 0\n\
                 v 1 1 0\n\
                 v 3 2 0\n\
                 v 0 2 0\n\
                 f 1 2 3 4 5\n";
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    let (mut warn, mut err) = (String::new(), String::new());
    assert!(load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert!(err.is_empty());
    assert_eq!(
        positions,
        vec![
            0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 1.0, 1.0, 0.0, 3.0, 2.0, 0.0, 0.0, 2.0, 0.0
        ]
    );
    assert_eq!(triangles, vec![vec![0, 1, 2], vec![2, 3, 4], vec![0, 2, 4]]);
    assert_eq!(triangles.len(), 3); // n-gon yields n-2 triangles
}

#[test]
fn crlf_line_endings() {
    // CRLF line endings parse identically to LF.
    let obj = "v 0 0 0\r\n\
                 v 1 0 0\r\n\
                 v 0 1 0\r\n\
                 f 1 2 3\r\n";
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    let (mut warn, mut err) = (String::new(), String::new());
    assert!(load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert!(err.is_empty());
    assert_eq!(positions, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]);
    assert_eq!(triangles, vec![vec![0, 1, 2]]);
}

#[test]
fn short_face_emits_nothing() {
    // Degenerate face with fewer than 3 corners: loads fine, emits nothing.
    let obj = "v 0 0 0\nv 1 0 0\nf 1 2\n";
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    let (mut warn, mut err) = (String::new(), String::new());
    assert!(load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert!(err.is_empty());
    assert_eq!(positions, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0]);
    assert!(triangles.is_empty());
}

#[test]
fn repeated_corners_pass_through() {
    // Degenerate face with repeated corners passes through as-is.
    let obj = "v 0 0 0\nf 1 1 1\n";
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    let (mut warn, mut err) = (String::new(), String::new());
    assert!(load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert!(err.is_empty());
    assert_eq!(positions, vec![0.0, 0.0, 0.0]);
    assert_eq!(triangles, vec![vec![0, 0, 0]]);
}

#[test]
fn zero_face_index_fails_untouched() {
    // Zero face index is invalid: the load fails and leaves outputs alone.
    let obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 0 3\n";
    let (mut positions, mut triangles) = (vec![9.0], vec![vec![7]]);
    let (mut warn, mut err) = (String::new(), String::new());
    assert!(!load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert!(err.contains("Failed parse"));
    assert_eq!(positions, vec![9.0]);
    assert_eq!(triangles, vec![vec![7]]);
}

#[test]
fn missing_file_fails_untouched() {
    // Missing file: the load fails with a "Cannot open file" error.
    let (mut positions, mut triangles) = (vec![9.0], vec![vec![7]]);
    let (mut warn, mut err) = (String::new(), String::new());
    let ok = load_obj_positions_and_triangles(
        std::path::Path::new("/nonexistent-dir-38f2/does-not-exist.obj"),
        &mut positions,
        &mut triangles,
        Some(&mut warn),
        Some(&mut err),
    );
    assert!(!ok);
    assert!(warn.is_empty());
    assert!(err.contains("Cannot open file"));
    assert_eq!(positions, vec![9.0]);
    assert_eq!(triangles, vec![vec![7]]);
}

#[test]
fn none_out_params() {
    // `None` warn/err behave like ignored out-params (no crash, same result).
    let obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
    let path: PathBuf = std::env::temp_dir().join(format!(
        "retopo_rs_objreader_test_{}_{}.obj",
        std::process::id(),
        TEMP_COUNTER.fetch_add(1, Ordering::SeqCst)
    ));
    std::fs::write(&path, obj).expect("write temp obj");
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    assert!(load_obj_positions_and_triangles(
        &path,
        &mut positions,
        &mut triangles,
        None,
        None
    ));
    assert_eq!(triangles, vec![vec![0, 1, 2]]);
    let _ = std::fs::remove_file(&path);
    // And on the failure path.
    let (mut positions, mut triangles) = (vec![9.0], vec![vec![7]]);
    assert!(!load_obj_positions_and_triangles(
        std::path::Path::new("/nonexistent-dir-38f2/does-not-exist.obj"),
        &mut positions,
        &mut triangles,
        None,
        None
    ));
    assert_eq!(positions, vec![9.0]);
    assert_eq!(triangles, vec![vec![7]]);
    // Unused-variable guard for the C++ `warn` parity (loader never warns).
    let mut warn = String::new();
    let mut err = String::new();
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    assert!(load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert!(warn.is_empty());
}

#[test]
fn weld_deindexed_soup() {
    // De-indexed soup: two triangles sharing an edge, written as six
    // independent vertices, plus one index-degenerate triangle. Welding
    // merges the duplicated edge corners and drops the degenerate face.
    let mut positions = vec![
        0.0, 0.0, 0.0, // 0
        1.0, 0.0, 0.0, // 1
        0.0, 1.0, 0.0, // 2
        1.0, 0.0, 0.0, // 3 == 1
        1.0, 1.0, 0.0, // 4
        0.0, 1.0, 0.0, // 5 == 2
    ];
    let mut triangles = vec![vec![0, 1, 2], vec![3, 4, 5], vec![0, 0, 1]];
    weld_positions_and_triangles(&mut positions, &mut triangles, None);
    assert_eq!(
        positions,
        vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0]
    );
    assert_eq!(triangles, vec![vec![0, 1, 2], vec![1, 3, 2]]);
}

#[test]
fn weld_soup_is_one_island() {
    // The welded soup above is one island; unwelded it is two (no shared
    // corner indices, so the splitter cannot join the triangles).
    let mut positions = vec![
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
    ];
    let mut triangles = vec![vec![0, 1, 2], vec![3, 4, 5]];
    assert_eq!(island_count(&triangles), 2);
    weld_positions_and_triangles(&mut positions, &mut triangles, None);
    assert_eq!(island_count(&triangles), 1);
}

#[test]
fn weld_identity_path() {
    // Already-welded input is byte-identical after the call (identity fast
    // path): the bench models take this path, so their counts cannot move.
    let mut positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0];
    let mut triangles = vec![vec![0, 1, 2], vec![0, 2, 3]];
    let before_positions = positions.clone();
    let before_triangles = triangles.clone();
    weld_positions_and_triangles(&mut positions, &mut triangles, None);
    assert_eq!(positions, before_positions);
    assert_eq!(triangles, before_triangles);
}

#[test]
fn weld_collapsed_corners_drop() {
    // A triangle whose three corners are distinct vertices at one position
    // collapses under the remap and is dropped as degenerate.
    let mut positions = vec![
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    ];
    let mut triangles = vec![vec![0, 1, 2], vec![0, 3, 4]];
    weld_positions_and_triangles(&mut positions, &mut triangles, None);
    assert_eq!(positions, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]);
    assert_eq!(triangles, vec![vec![0, 1, 2]]);
}

#[test]
fn weld_no_triangles() {
    // No triangles: nothing to weld, positions stay as they are.
    let mut positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0];
    let mut triangles: Vec<Vec<usize>> = Vec::new();
    weld_positions_and_triangles(&mut positions, &mut triangles, None);
    assert_eq!(positions, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0]);
    assert!(triangles.is_empty());
}

#[test]
fn weld_nan_face_drops_with_stats() {
    // A good triangle plus a NaN-cornered one: the weld drops exactly the
    // NaN face, removes its now-orphaned vertices, and reports the reason.
    // No NaN survives in the positions.
    let nan = f32::NAN;
    let mut positions = vec![
        0.0, 0.0, 0.0, // 0
        1.0, 0.0, 0.0, // 1
        0.0, 1.0, 0.0, // 2
        nan, 0.0, 0.0, // 3
        9.0, 0.0, 0.0, // 4
        9.0, 1.0, 0.0, // 5
    ];
    let mut triangles = vec![vec![0, 1, 2], vec![3, 4, 5]];
    let mut stats = WeldStats::default();
    weld_positions_and_triangles(&mut positions, &mut triangles, Some(&mut stats));
    assert_eq!(stats.non_finite_dropped, 1);
    assert_eq!(stats.degenerate_dropped, 0);
    assert_eq!(triangles, vec![vec![0, 1, 2]]);
    assert!(positions.iter().all(|v| v.is_finite()));
    assert_eq!(positions.len(), 9);
}

#[test]
fn weld_infinity_drops() {
    // Infinity (including the 1e999-overflow spelling, which strtod rounds
    // to inf) is dropped the same way.
    let inf = f32::INFINITY;
    let mut positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, inf, 1.0, 0.0];
    let mut triangles = vec![vec![0, 1, 2]];
    let mut stats = WeldStats::default();
    weld_positions_and_triangles(&mut positions, &mut triangles, Some(&mut stats));
    assert_eq!(stats.non_finite_dropped, 1);
    assert!(triangles.is_empty());
    // The 1e999 spelling arrives as inf through the loader too.
    let obj = "v 0 0 0\nv 1 0 0\nv 1e999 1 0\nf 1 2 3\n";
    let (mut positions, mut triangles) = (Vec::new(), Vec::new());
    let (mut warn, mut err) = (String::new(), String::new());
    assert!(load_string(
        obj,
        &mut positions,
        &mut triangles,
        &mut warn,
        &mut err
    ));
    assert_eq!(positions[6], f32::INFINITY);
    let mut stats = WeldStats::default();
    weld_positions_and_triangles(&mut positions, &mut triangles, Some(&mut stats));
    assert_eq!(stats.non_finite_dropped, 1);
    assert!(triangles.is_empty());
}

#[test]
fn weld_stats_clean_input() {
    // Stats on clean input: zero drops, byte-identical input (identity path).
    let mut positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0];
    let mut triangles = vec![vec![0, 1, 2], vec![0, 2, 3]];
    let before_positions = positions.clone();
    let before_triangles = triangles.clone();
    let mut stats = WeldStats::default();
    weld_positions_and_triangles(&mut positions, &mut triangles, Some(&mut stats));
    assert_eq!(stats.non_finite_dropped, 0);
    assert_eq!(stats.degenerate_dropped, 0);
    assert_eq!(positions, before_positions);
    assert_eq!(triangles, before_triangles);
}

#[test]
fn weld_stats_degenerate_accounting() {
    // Degenerate accounting: an index-degenerate face counts before the
    // remap, and a face whose distinct-but-coincident corners collapse
    // under the remap counts after it.
    let mut positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    let mut triangles = vec![vec![0, 1, 2], vec![0, 0, 1]];
    let mut stats = WeldStats::default();
    weld_positions_and_triangles(&mut positions, &mut triangles, Some(&mut stats));
    assert_eq!(stats.degenerate_dropped, 1);
    assert_eq!(stats.non_finite_dropped, 0);
    assert_eq!(triangles.len(), 1);

    let mut positions = vec![
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    ];
    let mut triangles = vec![vec![0, 1, 2], vec![0, 3, 4]];
    let mut stats = WeldStats::default();
    weld_positions_and_triangles(&mut positions, &mut triangles, Some(&mut stats));
    assert_eq!(stats.degenerate_dropped, 1);
    assert_eq!(triangles.len(), 1);
}
