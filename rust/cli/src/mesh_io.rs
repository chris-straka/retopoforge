//! Mesh IO: OBJ/GLB loading (with weld-on-load) and saving.
//!
//! Loading dispatches on the file extension, warns/errors exactly as the
//! core loaders report, then welds positions and triangles. Saving
//! writes OBJ directly or GLB through the core writer, with optional UVs.

use crate::RETOPO_VERSION;
use crate::format::g_format;
use retopo_core::glb as glb_io;
use retopo_core::obj_reader::{self, WeldStats};
use retopo_core::vector2::Vector2;
use retopo_core::vector3::Vector3;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

pub(crate) struct LoadedMesh {
    pub(crate) vertices: Vec<Vector3>,
    pub(crate) triangles: Vec<Vec<usize>>,
    pub(crate) pre_weld_vertices: usize,
    pub(crate) pre_weld_triangles: usize,
    pub(crate) weld_stats: WeldStats,
}

type CoreLoader = fn(
    &Path,
    &mut Vec<f32>,
    &mut Vec<Vec<usize>>,
    Option<&mut String>,
    Option<&mut String>,
) -> bool;

fn load_with(filename: &Path, loader: CoreLoader) -> Option<LoadedMesh> {
    let mut positions: Vec<f32> = Vec::new();
    let mut loaded_triangles: Vec<Vec<usize>> = Vec::new();
    let mut warn = String::new();
    let mut err = String::new();
    let ok = loader(
        filename,
        &mut positions,
        &mut loaded_triangles,
        Some(&mut warn),
        Some(&mut err),
    );
    if !warn.is_empty() {
        eprintln!("WARN: {warn}");
    }
    if !err.is_empty() {
        eprintln!("{err}");
    }
    if !ok {
        return None;
    }
    let pre_weld_vertices = positions.len() / 3;
    let pre_weld_triangles = loaded_triangles.len();
    let mut weld_stats = WeldStats::default();
    obj_reader::weld_positions_and_triangles(
        &mut positions,
        &mut loaded_triangles,
        Some(&mut weld_stats),
    );
    let mut vertices = Vec::with_capacity(positions.len() / 3);
    for i in 0..positions.len() / 3 {
        vertices.push(Vector3::new(
            positions[3 * i] as f64,
            positions[3 * i + 1] as f64,
            positions[3 * i + 2] as f64,
        ));
    }
    Some(LoadedMesh {
        vertices,
        triangles: loaded_triangles,
        pre_weld_vertices,
        pre_weld_triangles,
        weld_stats,
    })
}

pub(crate) fn load_mesh(filename: &Path) -> Option<LoadedMesh> {
    // The extension checks take the lossy path text, matching argv which
    // is already lossy by the time it reaches the CLI.
    if glb_io::has_glb_extension(&filename.to_string_lossy()) {
        load_with(filename, glb_io::load_glb_positions_and_triangles)
    } else {
        load_with(filename, obj_reader::load_obj_positions_and_triangles)
    }
}

/// The `Loaded ...` / `Welded input: ...` stderr block (suppressed by
/// `--quiet` at the call sites' discretion — here via `quiet`).
pub(crate) fn report_loaded(loaded: &LoadedMesh, quiet: bool) {
    if quiet {
        return;
    }
    eprintln!(
        "Loaded {} vertices, {} triangles",
        loaded.vertices.len(),
        loaded.triangles.len()
    );
    if loaded.pre_weld_vertices != loaded.vertices.len()
        || loaded.pre_weld_triangles != loaded.triangles.len()
    {
        eprintln!(
            "Welded input: {} -> {} vertices, {} -> {} triangles",
            loaded.pre_weld_vertices,
            loaded.vertices.len(),
            loaded.pre_weld_triangles,
            loaded.triangles.len()
        );
    }
}

pub(crate) fn warn_dropped_non_finite(non_finite_dropped: usize) {
    if non_finite_dropped > 0 {
        eprintln!(
            "Warning: dropped {non_finite_dropped} input triangles with non-finite corners (NaN or infinity)"
        );
    }
}

fn write_obj_header(file: &mut File) -> bool {
    writeln!(file, "# retopoforge {RETOPO_VERSION}").is_ok()
        && writeln!(file, "# https://github.com/chris-straka/retopoforge").is_ok()
}

fn write_obj_vertices(file: &mut File, vertices: &[Vector3]) -> bool {
    for v in vertices {
        if writeln!(
            file,
            "v {} {} {}",
            g_format(v.x()),
            g_format(v.y()),
            g_format(v.z())
        )
        .is_err()
        {
            return false;
        }
    }
    true
}

fn write_obj_face(file: &mut File, face: &[usize], with_uvs: bool) -> bool {
    if write!(file, "f").is_err() {
        return false;
    }
    for index in face {
        let corner = index + 1;
        let wrote = if with_uvs {
            write!(file, " {corner}/{corner}")
        } else {
            write!(file, " {corner}")
        };
        if wrote.is_err() {
            return false;
        }
    }
    writeln!(file).is_ok()
}

fn save_obj(
    filename: &Path,
    vertices: &[Vector3],
    quads: &[Vec<usize>],
    uvs: Option<&[Vector2]>,
) -> bool {
    let mut file = match File::create(filename) {
        Ok(file) => file,
        Err(_) => return false,
    };
    if !write_obj_header(&mut file) {
        return false;
    }
    if !write_obj_vertices(&mut file, vertices) {
        return false;
    }
    if let Some(uvs) = uvs {
        for uv in uvs {
            if writeln!(file, "vt {} {}", g_format(uv.x()), g_format(uv.y())).is_err() {
                return false;
            }
        }
    }
    for face in quads {
        if !write_obj_face(&mut file, face, uvs.is_some()) {
            return false;
        }
    }
    file.flush().is_ok()
}

pub(crate) fn save_mesh(
    filename: &Path,
    vertices: &[Vector3],
    quads: &[Vec<usize>],
    uvs: Option<&[Vector2]>,
) -> bool {
    if vertices.is_empty() {
        return false;
    }
    let have_uvs = uvs.map(|u| u.len() == vertices.len()).unwrap_or(false);
    let uvs = uvs.filter(|_| have_uvs);
    if glb_io::has_glb_extension(&filename.to_string_lossy()) {
        let generator = format!("retopoforge {RETOPO_VERSION}");
        return match uvs {
            Some(uvs) => glb_io::save_glb_with_uvs(filename, &generator, vertices, quads, uvs),
            None => glb_io::save_glb(filename, &generator, vertices, quads),
        };
    }
    save_obj(filename, vertices, quads, uvs)
}

/// `stem()`/`extension()` per `std::filesystem::path` rules (kept for
/// output-name stability): dotfiles (`.obj`) and `.`/`..` have no
/// extension, a trailing dot (`foo.`) yields the `.` extension,
/// otherwise the extension runs from the last dot inclusive. `ext`
/// comes back with the `.obj` default applied when empty.
pub(crate) fn cpp_stem_ext(file_name: &str) -> (String, String) {
    if file_name.is_empty() || file_name == "." || file_name == ".." {
        return (file_name.to_string(), ".obj".to_string());
    }
    match file_name.rfind('.') {
        None => (file_name.to_string(), ".obj".to_string()),
        Some(0) => (file_name.to_string(), ".obj".to_string()),
        Some(dot) => (file_name[..dot].to_string(), file_name[dot..].to_string()),
    }
}

/// Insert `_lod<index>` before the extension, keeping the parent
/// directory (a bare `/name` keeps its leading slash). Operates on the
/// lossy path text, matching argv which is already lossy by the time it
/// reaches the CLI.
pub(crate) fn lod_output_path(base_output: &Path, lod_index: usize) -> PathBuf {
    let base_output = base_output.to_string_lossy();
    lod_output_path_str(&base_output, lod_index)
}

fn lod_output_path_str(base_output: &str, lod_index: usize) -> PathBuf {
    let (parent, file_name) = match base_output.rfind('/') {
        Some(slash) => (&base_output[..slash], &base_output[slash + 1..]),
        None => ("", base_output),
    };
    let (stem, ext) = cpp_stem_ext(file_name);
    let name = format!("{stem}_lod{lod_index}{ext}");
    if parent.is_empty() {
        if base_output.starts_with('/') {
            PathBuf::from(format!("/{name}"))
        } else {
            PathBuf::from(name)
        }
    } else {
        PathBuf::from(format!("{parent}/{name}"))
    }
}

/// Final path component as text, falling back to the whole path when it
/// has no file name.
pub(crate) fn file_name_of(path: &Path) -> String {
    path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.to_string_lossy().into_owned())
}
