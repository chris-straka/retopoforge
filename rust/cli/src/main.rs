// Line-by-line mirror of `cli/main.cpp` (the `retopo` headless CLI).
//
// Same 19 flags, same file formats, same exit codes, same stdout/stderr
// bytes on every CLI-owned line. Engine-owned output differs where the
// joined `retopo_core` lanes deliberately dropped C++ module-level `cerr`
// diagnostics (project stderr-gap memo: quad/frame/parameterizer/engine
// precedent); the e2e harness (`tests/e2e_diff.rs`) audits every such
// line class instead of silently diverging.
//
// Mirror notes (each deliberate divergence from a naive transcription):
// - C++ `ostream << double` prints `%.6g`; Rust `{}` prints shortest
//   round-trip. Every double the CLI prints goes through [`g_format`],
//   which replicates `%.6g` exactly (differentially checked against the
//   C++ binary's own output in the e2e harness).
// - `strtod`/`strtol` accept leading whitespace, `+` signs, `inf`/`nan`,
//   and (empty string only) no digits at all; Rust `parse` does not.
//   [`parse_double`] and [`parse_int`] replicate the C acceptance set,
//   including the empty-string-yields-zero quirk. Two exotic spellings
//   stay divergent by mechanism: hex floats (`0x1p3`, no Rust parser)
//   and `nan(payload)` (see DECISIONS in the lane report).
// - Rust float/int `parse` accepts `_` separators, which `strtod`/
//   `strtol` reject; numeric tokens containing `_` are rejected here.
// - The C++ progress state is a bare struct mutated from worker threads
//   (a benign data race); here it sits behind a `Mutex`, so content
//   matches while racy interleavings can order lines differently on
//   multi-island runs (single-island runs are byte-identical).
// - `process::exit` does not flush `stdout`; every early exit flushes
//   explicitly first.

use retopo_core::auto_remesher::{AutoRemesher, ModelType};
use retopo_core::glb as glb_io;
use retopo_core::mesh_separator::MeshSeparator;
use retopo_core::obj_reader::{self, WeldStats};
use retopo_core::vector2::Vector2;
use retopo_core::vector3::Vector3;
use std::ffi::c_void;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::Instant;

const RETOPO_VERSION: &str = "0.1.0";

struct Params {
    input_path: String,
    output_path: String,
    report_path: String,
    lod_targets: Vec<i64>,
    target_quads: i32,
    edge_scaling: f64,
    sharp_edge_degrees: f64,
    smooth_normal_degrees: f64,
    adaptivity: f64,
    anisotropy: f64,
    model_type: ModelType,
    symmetry_enabled: bool,
    symmetry_axis: i32,
    guides_path: String,
    features_path: String,
    density_path: String,
    emit_uvs: bool,
    quiet: bool,
}

impl Params {
    fn new() -> Self {
        Self {
            input_path: String::new(),
            output_path: String::new(),
            report_path: String::new(),
            lod_targets: Vec::new(),
            target_quads: 50000,
            edge_scaling: 1.0,
            sharp_edge_degrees: 90.0,
            smooth_normal_degrees: 0.0,
            adaptivity: 1.0,
            anisotropy: 1.0,
            model_type: ModelType::Organic,
            symmetry_enabled: false,
            symmetry_axis: -1,
            guides_path: String::new(),
            features_path: String::new(),
            density_path: String::new(),
            emit_uvs: false,
            quiet: false,
        }
    }
}

fn print_usage(argv0: &str) {
    print!(
        concat!(
            "Usage: {argv0} --input <file.obj|file.glb|dir> --output <output.obj|output.glb|dir> [options]\n",
            "\n",
            "Options:\n",
            "  -i, --input <file|dir>      Input .obj or .glb file to remesh (required).\n",
            "                              A directory remeshes every .obj and .glb\n",
            "                              in it (non-recursive); --output is then a\n",
            "                              directory (created if missing) and each\n",
            "                              input foo.ext is written as <dir>/foo.ext.\n",
            "  -o, --output <path>         Output file path (required): .obj writes\n",
            "                              OBJ, .glb writes GLB (quads triangulated).\n",
            "                              A directory in batch mode (see --input).\n",
            "  --report <report.txt>       Write a stats report file (optional)\n",
            "  --target-quads <count>      Target quad count (default: 50000,\n",
            "                              ignored when --lods is given)\n",
            "  --lods <q0,q1,...>          Emit a full LOD chain in one run, e.g.\n",
            "                              --lods 10000,5000,2000 writes\n",
            "                              <stem>_lod0.<ext>, <stem>_lod1.<ext>, ...\n",
            "                              next to --output, keeping its extension\n",
            "                              (overrides --target-quads)\n",
            "  --edge-scaling <factor>     Edge scaling factor (default: 1.0, range: 1.0-4.0)\n",
            "  --sharp-edge <degrees>      Sharp edge dihedral angle threshold\n",
            "                              (default: 90.0, range: 30.0-180.0)\n",
            "  --smooth-normal <degrees>   Smooth normal angle threshold\n",
            "                              (default: 0.0, range: 0.0-180.0)\n",
            "  --adaptivity <value>        Curvature-adaptive quad density\n",
            "                              (default: 1.0, range: 0.0-1.0)\n",
            "  --anisotropy <value>        Curvature-adaptive quad elongation\n",
            "                              (default: 1.0, range: 0.0-1.0)\n",
            "  --model-type <organic|hardsurface>\n",
            "                              Model type hint (default: organic)\n",
            "  --symmetry <off|auto|x|y|z>  Mirror-symmetry constraints\n",
            "                              (default: off). auto detects the dominant\n",
            "                              plane; x/y/z pin it. Falls back to\n",
            "                              unconstrained output when the input scores\n",
            "                              below threshold on the chosen plane\n",
            "  --guides <file>             Guide-curve constraints: quad edge flow\n",
            "                              follows the polylines (eye/mouth loops).\n",
            "                              File format: one 'x y z' point per line,\n",
            "                              blank lines separate polylines, '#' starts\n",
            "                              a comment. Points live in input-mesh\n",
            "                              coordinates. Single-file and --lods runs\n",
            "                              only (rejected in batch mode)\n",
            "  --features <file>           Sharp/feature constraints: crisp edges\n",
            "                              along the polylines (hard-surface props).\n",
            "                              Same file format as --guides: one\n",
            "                              'x y z' point per line, blank lines\n",
            "                              separate polylines, '#' starts a comment.\n",
            "                              Points live in input-mesh coordinates.\n",
            "                              Single-file and --lods runs only\n",
            "                              (rejected in batch mode)\n",
            "  --density <file>            Local density control: one multiplier\n",
            "                              per input vertex (OBJ v-line order),\n",
            "                              1.0 = unchanged, range 0.25-4.0 (values\n",
            "                              outside clamp). '#' starts a comment.\n",
            "                              Strong localized refinement saturates\n",
            "                              (~2.3x realized for 4x asks); mild masks\n",
            "                              realize nearly fully. Single-file and\n",
            "                              --lods runs only (rejected in batch mode)\n",
            "  --uvs <on|off>              Emit remeshed UVs from the internal\n",
            "                              parameterization (default: off). OBJ\n",
            "                              gains vt lines + v/vt corners, GLB gains\n",
            "                              TEXCOORD_0. Off keeps every output byte\n",
            "                              identical to before\n",
            "  --quiet                     Silence progress and info output; only\n",
            "                              warnings, errors and the report print\n",
            "  -h, --help                  Show this help\n",
            "  -v, --version               Show version\n",
        ),
        argv0 = argv0
    );
}

// Replicates C++ `ostream << double` (== `printf("%.6g")`): 6 significant
// digits, `%e` style when the decimal exponent is < -4 or >= 6, trailing
// zeros stripped. The 6 rounded digits come from Rust's `{:.5e}`
// (correctly rounded, same as libc `printf`), so only the style
// selection and zero-stripping are hand-rolled. NaN prints signless
// `nan` (macOS libc++ behavior, matching the glb lane's pin); -0.0
// keeps its sign (`-0`), as `%g` does.
fn g_format(value: f64) -> String {
    if value.is_nan() {
        return "nan".to_string();
    }
    if value.is_infinite() {
        return if value.is_sign_negative() {
            "-inf".to_string()
        } else {
            "inf".to_string()
        };
    }
    if value == 0.0 {
        return if value.is_sign_negative() {
            "-0".to_string()
        } else {
            "0".to_string()
        };
    }
    let sign = if value.is_sign_negative() { "-" } else { "" };
    // `1.23457e-7` style: 6 significant digits, correctly rounded.
    let sci = format!("{:.5e}", value.abs());
    let epos = sci.find('e').expect("scientific format lacks 'e'");
    let digits: String = sci[..epos].chars().filter(|c| *c != '.').collect();
    debug_assert_eq!(digits.len(), 6);
    let exp: i32 = sci[epos + 1..].parse().expect("bad scientific exponent");
    let digits = digits.into_bytes();
    if exp < -4 || exp >= 6 {
        // `%e` style: strip trailing zero digits, keep at least one.
        let mut end = digits.len();
        while end > 1 && digits[end - 1] == b'0' {
            end -= 1;
        }
        let mut out = String::with_capacity(16);
        out.push_str(sign);
        out.push(digits[0] as char);
        if end > 1 {
            out.push('.');
            for &d in &digits[1..end] {
                out.push(d as char);
            }
        }
        out.push('e');
        out.push_str(&format!("{exp:+03}"));
        out
    } else if exp >= 0 {
        // `%f` style, point inside/right of the digits.
        let int_len = (exp + 1) as usize;
        let mut out = String::with_capacity(16);
        out.push_str(sign);
        out.push_str(std::str::from_utf8(&digits[..int_len]).expect("digits"));
        let mut end = digits.len();
        while end > int_len && digits[end - 1] == b'0' {
            end -= 1;
        }
        if end > int_len {
            out.push('.');
            out.push_str(std::str::from_utf8(&digits[int_len..end]).expect("digits"));
        }
        out
    } else {
        // `%f` style, 0.000<digits>.
        let mut end = digits.len();
        while end > 0 && digits[end - 1] == b'0' {
            end -= 1;
        }
        let mut out = String::with_capacity(16);
        out.push_str(sign);
        out.push_str("0.");
        for _ in 0..(-exp - 1) {
            out.push('0');
        }
        out.push_str(std::str::from_utf8(&digits[..end]).expect("digits"));
        out
    }
}

fn flush_stdout() {
    let _ = std::io::stdout().flush();
}

fn take_value(args: &[String], i: &mut usize, flag: &str) -> Option<String> {
    let arg = &args[*i];
    if let Some(eq) = arg.find('=') {
        return Some(arg[eq + 1..].to_string());
    }
    if *i + 1 >= args.len() {
        eprintln!("Error: {flag} requires a value");
        return None;
    }
    *i += 1;
    Some(args[*i].clone())
}

// `strtod` acceptance set: leading C-locale whitespace skipped, full
// consumption required, empty string yields 0.0 (no conversion, but the
// end pointer still lands on the NUL, so the C++ check passes). `_`
// rejected (Rust `parse` accepts it, `strtod` stops at it).
fn parse_double(text: &str, flag: &str) -> Option<f64> {
    if text.is_empty() {
        return Some(0.0);
    }
    let stripped = text.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    let ok = !stripped.is_empty()
        && !stripped.as_bytes().contains(&b'_')
        && !stripped.trim_start_matches(['+', '-']).starts_with("0x")
        && !stripped.trim_start_matches(['+', '-']).starts_with("0X");
    if !ok {
        eprintln!("Error: {flag} expects a number, got '{text}'");
        return None;
    }
    match stripped.parse::<f64>() {
        Ok(value) => Some(value),
        Err(_) => {
            eprintln!("Error: {flag} expects a number, got '{text}'");
            None
        }
    }
}

// `strtol` base-10 acceptance set, same shape as [`parse_double`]:
// leading whitespace, optional sign, full consumption, empty yields 0.
// Overflow clamps to `LONG_MAX`/`LONG_MIN` (errno ignored), then the
// negativity check runs on the clamped value and the `int` cast wraps,
// exactly like the C++ `static_cast<int>(value)`.
fn parse_int(text: &str, flag: &str) -> Option<i32> {
    if text.is_empty() {
        return Some(0);
    }
    let stripped = text.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    if stripped.is_empty() || stripped.as_bytes().contains(&b'_') {
        eprintln!("Error: {flag} expects a non-negative integer, got '{text}'");
        return None;
    }
    let value: i64 = match stripped.parse::<i64>() {
        Ok(value) => value,
        Err(err) => {
            use std::num::IntErrorKind::*;
            match err.kind() {
                PosOverflow => i64::MAX,
                NegOverflow => i64::MIN,
                _ => {
                    eprintln!("Error: {flag} expects a non-negative integer, got '{text}'");
                    return None;
                }
            }
        }
    };
    if value < 0 {
        eprintln!("Error: {flag} expects a non-negative integer, got '{text}'");
        return None;
    }
    Some(value as i32)
}

fn parse_lods(text: &str, out: &mut Vec<i64>) -> bool {
    out.clear();
    let mut start = 0usize;
    while start <= text.len() {
        let end = text[start..]
            .find(',')
            .map(|p| start + p)
            .unwrap_or(text.len());
        let mut token = &text[start..end];
        token = token.trim_matches([' ', '\t']);
        let value = match parse_int(token, "--lods") {
            Some(value) => value,
            None => return false,
        };
        if value <= 0 {
            eprintln!("Error: --lods expects positive integers, got '{token}'");
            return false;
        }
        out.push(value as i64);
        start = end + 1;
    }
    if out.is_empty() {
        eprintln!("Error: --lods expects a comma-separated list, got '{text}'");
        return false;
    }
    true
}

// Split file bytes into `getline`-style lines: split on `\n`, drop the
// phantom segment a trailing newline would add, keep `\r` (the C++
// skip-sets treat it as blank, and error echoes keep it).
fn split_lines(data: &[u8]) -> Vec<&[u8]> {
    if data.is_empty() {
        return Vec::new();
    }
    let mut lines: Vec<&[u8]> = data.split(|&b| b == b'\n').collect();
    if data.ends_with(b"\n") {
        lines.pop();
    }
    lines
}

fn is_c_space(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\n' | b'\x0b' | b'\x0c' | b'\r')
}

// One `sscanf %lf` token: full-consumption double, `_`/hex rejected like
// [`parse_double`] (same `strtod`-vs-`parse` gap, same loud error).
fn parse_double_token(token: &[u8]) -> Option<f64> {
    if token.is_empty() || token.contains(&b'_') {
        return None;
    }
    let text = std::str::from_utf8(token).ok()?;
    let body = text.trim_start_matches(['+', '-']);
    if body.starts_with("0x") || body.starts_with("0X") {
        return None;
    }
    text.parse::<f64>().ok()
}

// Write an error line whose middle is raw (possibly non-UTF8) file bytes,
// mirroring `cerr << ... << line << ...` byte-for-byte.
fn eprint_raw(parts: &[&[u8]]) {
    let mut err = std::io::stderr().lock();
    for part in parts {
        let _ = err.write_all(part);
    }
    let _ = err.write_all(b"\n");
    let _ = err.flush();
}

// Guide/feature polyline file: one "x y z" point per line, blank lines
// separate polylines, "#" starts a comment. Mirrors `parseGuidesFile`
// exactly, including the post-`#`-strip echo in error messages and the
// trailing-garbage rule (anything past the third double but [ \t\r]
// fails, while any C whitespace may separate the numbers).
fn parse_guides_file(path: &str, guides: &mut Vec<Vec<Vector3>>, flag_label: &str) -> bool {
    guides.clear();
    let data = match std::fs::read(path) {
        Ok(data) => data,
        Err(_) => {
            eprintln!("Error: cannot open {flag_label} file '{path}'");
            return false;
        }
    };
    let mut current: Vec<Vector3> = Vec::new();
    let mut failed = false;
    for (index, raw) in split_lines(&data).iter().enumerate() {
        let line_no = index + 1;
        let mut line = *raw;
        if let Some(hash) = line.iter().position(|&b| b == b'#') {
            line = &line[..hash];
        }
        if !line.iter().any(|&b| b != b' ' && b != b'\t' && b != b'\r') {
            if current.len() >= 2 {
                guides.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
            continue;
        }
        // Tokenize on C whitespace; the tail past the last token must be
        // [ \t\r] only (a trailing vertical-tab fails in `sscanf` too).
        let mut tokens: Vec<&[u8]> = Vec::new();
        let mut i = 0;
        while i < line.len() {
            while i < line.len() && is_c_space(line[i]) {
                i += 1;
            }
            let start = i;
            while i < line.len() && !is_c_space(line[i]) {
                i += 1;
            }
            if i > start {
                tokens.push(&line[start..i]);
            }
        }
        let mut values = [0.0f64; 3];
        let mut ok = tokens.len() == 3;
        if ok {
            for (k, token) in tokens.iter().enumerate() {
                match parse_double_token(token) {
                    Some(v) => values[k] = v,
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
        }
        if ok {
            // Find the end of the third token to check the tail.
            let mut seen = 0;
            let mut j = 0;
            let mut tail_ok = true;
            while j < line.len() {
                if is_c_space(line[j]) {
                    j += 1;
                    continue;
                }
                while j < line.len() && !is_c_space(line[j]) {
                    j += 1;
                }
                seen += 1;
                if seen == 3 {
                    for &b in &line[j..] {
                        if b != b' ' && b != b'\t' && b != b'\r' {
                            tail_ok = false;
                            break;
                        }
                    }
                    break;
                }
            }
            ok = tail_ok;
        }
        if !ok {
            let prefix =
                format!("Error: {flag_label} file '{path}' line {line_no} expects 'x y z', got '");
            eprint_raw(&[prefix.as_bytes(), line, b"'"]);
            failed = true;
            break;
        }
        current.push(Vector3::new(values[0], values[1], values[2]));
    }
    if failed {
        return false;
    }
    if current.len() >= 2 {
        guides.push(std::mem::take(&mut current));
    }
    if guides.is_empty() {
        eprintln!(
            "Error: {flag_label} file '{path}' holds no usable polyline (need 2+ points per polyline)"
        );
        return false;
    }
    true
}

// Density file: one multiplier per line ("#" starts a comment, blank
// lines skipped). Mirrors `parseDensityFile`.
fn parse_density_file(path: &str, multipliers: &mut Vec<f64>) -> bool {
    multipliers.clear();
    let data = match std::fs::read(path) {
        Ok(data) => data,
        Err(_) => {
            eprintln!("Error: cannot open --density file '{path}'");
            return false;
        }
    };
    for (index, raw) in split_lines(&data).iter().enumerate() {
        let line_no = index + 1;
        let mut line = *raw;
        if let Some(hash) = line.iter().position(|&b| b == b'#') {
            line = &line[..hash];
        }
        if !line.iter().any(|&b| b != b' ' && b != b'\t' && b != b'\r') {
            continue;
        }
        let mut tokens: Vec<&[u8]> = Vec::new();
        let mut i = 0;
        while i < line.len() {
            while i < line.len() && is_c_space(line[i]) {
                i += 1;
            }
            let start = i;
            while i < line.len() && !is_c_space(line[i]) {
                i += 1;
            }
            if i > start {
                tokens.push(&line[start..i]);
            }
        }
        let mut ok = tokens.len() == 1;
        let mut value = 0.0;
        if ok {
            match parse_double_token(tokens[0]) {
                Some(v) => value = v,
                None => ok = false,
            }
        }
        if ok {
            let mut j = 0;
            while j < line.len() {
                if is_c_space(line[j]) {
                    j += 1;
                    continue;
                }
                while j < line.len() && !is_c_space(line[j]) {
                    j += 1;
                }
                for &b in &line[j..] {
                    if b != b' ' && b != b'\t' && b != b'\r' {
                        ok = false;
                        break;
                    }
                }
                break;
            }
        }
        if !ok {
            let prefix =
                format!("Error: --density file '{path}' line {line_no} expects a number, got '");
            eprint_raw(&[prefix.as_bytes(), line, b"'"]);
            return false;
        }
        multipliers.push(value);
    }
    if multipliers.is_empty() {
        eprintln!("Error: --density file '{path}' holds no multipliers");
        return false;
    }
    true
}

fn matches(arg: &str, long_flag: &str, short_flag: Option<char>) -> bool {
    if let Some(short) = short_flag {
        let mut buf = [0u8; 4];
        let short_opt = format!("-{}", short.encode_utf8(&mut buf));
        if arg == short_opt {
            return true;
        }
    }
    arg.starts_with(long_flag)
        && (arg.len() == long_flag.len() || arg.as_bytes().get(long_flag.len()) == Some(&b'='))
}

fn parse_args(args: &[String], params: &mut Params) -> bool {
    let mut i = 1;
    while i < args.len() {
        let arg = args[i].clone();
        if matches(&arg, "--help", Some('h')) {
            print_usage(&args[0]);
            flush_stdout();
            std::process::exit(0);
        } else if matches(&arg, "--version", Some('v')) {
            println!("retopoforge {RETOPO_VERSION}");
            flush_stdout();
            std::process::exit(0);
        } else if matches(&arg, "--input", Some('i')) {
            match take_value(args, &mut i, "--input") {
                Some(value) => params.input_path = value,
                None => return false,
            }
        } else if matches(&arg, "--output", Some('o')) {
            match take_value(args, &mut i, "--output") {
                Some(value) => params.output_path = value,
                None => return false,
            }
        } else if matches(&arg, "--report", None) {
            match take_value(args, &mut i, "--report") {
                Some(value) => params.report_path = value,
                None => return false,
            }
        } else if matches(&arg, "--target-quads", None) {
            let value = match take_value(args, &mut i, "--target-quads") {
                Some(value) => value,
                None => return false,
            };
            match parse_int(&value, "--target-quads") {
                Some(v) => params.target_quads = v,
                None => return false,
            }
        } else if matches(&arg, "--lods", None) {
            let value = match take_value(args, &mut i, "--lods") {
                Some(value) => value,
                None => return false,
            };
            if !parse_lods(&value, &mut params.lod_targets) {
                return false;
            }
        } else if matches(&arg, "--edge-scaling", None) {
            let value = match take_value(args, &mut i, "--edge-scaling") {
                Some(value) => value,
                None => return false,
            };
            match parse_double(&value, "--edge-scaling") {
                Some(v) => params.edge_scaling = v,
                None => return false,
            }
        } else if matches(&arg, "--sharp-edge", None) {
            let value = match take_value(args, &mut i, "--sharp-edge") {
                Some(value) => value,
                None => return false,
            };
            match parse_double(&value, "--sharp-edge") {
                Some(v) => params.sharp_edge_degrees = v,
                None => return false,
            }
        } else if matches(&arg, "--smooth-normal", None) {
            let value = match take_value(args, &mut i, "--smooth-normal") {
                Some(value) => value,
                None => return false,
            };
            match parse_double(&value, "--smooth-normal") {
                Some(v) => params.smooth_normal_degrees = v,
                None => return false,
            }
        } else if matches(&arg, "--adaptivity", None) {
            let value = match take_value(args, &mut i, "--adaptivity") {
                Some(value) => value,
                None => return false,
            };
            match parse_double(&value, "--adaptivity") {
                Some(v) => params.adaptivity = v,
                None => return false,
            }
        } else if matches(&arg, "--anisotropy", None) {
            let value = match take_value(args, &mut i, "--anisotropy") {
                Some(value) => value,
                None => return false,
            };
            match parse_double(&value, "--anisotropy") {
                Some(v) => params.anisotropy = v,
                None => return false,
            }
        } else if matches(&arg, "--uvs", None) {
            let value = match take_value(args, &mut i, "--uvs") {
                Some(value) => value,
                None => return false,
            };
            if value == "on" {
                params.emit_uvs = true;
            } else if value == "off" {
                params.emit_uvs = false;
            } else {
                eprintln!("Error: --uvs expects 'on' or 'off', got '{value}'");
                return false;
            }
        } else if matches(&arg, "--quiet", None) {
            params.quiet = true;
        } else if matches(&arg, "--symmetry", None) {
            let value = match take_value(args, &mut i, "--symmetry") {
                Some(value) => value,
                None => return false,
            };
            if value == "off" {
                params.symmetry_enabled = false;
                params.symmetry_axis = -1;
            } else if value == "auto" {
                params.symmetry_enabled = true;
                params.symmetry_axis = -1;
            } else if value == "x" || value == "X" {
                params.symmetry_enabled = true;
                params.symmetry_axis = 0;
            } else if value == "y" || value == "Y" {
                params.symmetry_enabled = true;
                params.symmetry_axis = 1;
            } else if value == "z" || value == "Z" {
                params.symmetry_enabled = true;
                params.symmetry_axis = 2;
            } else {
                eprintln!(
                    "Error: --symmetry expects 'off', 'auto', 'x', 'y' or 'z', got '{value}'"
                );
                return false;
            }
        } else if matches(&arg, "--density", None) {
            match take_value(args, &mut i, "--density") {
                Some(value) => params.density_path = value,
                None => return false,
            }
        } else if matches(&arg, "--guides", None) {
            match take_value(args, &mut i, "--guides") {
                Some(value) => params.guides_path = value,
                None => return false,
            }
        } else if matches(&arg, "--features", None) {
            match take_value(args, &mut i, "--features") {
                Some(value) => params.features_path = value,
                None => return false,
            }
        } else if matches(&arg, "--model-type", None) {
            let value = match take_value(args, &mut i, "--model-type") {
                Some(value) => value,
                None => return false,
            };
            if value == "organic" {
                params.model_type = ModelType::Organic;
            } else if value == "hardsurface" || value == "hard-surface" || value == "hard_surface" {
                params.model_type = ModelType::HardSurface;
            } else {
                eprintln!("Error: --model-type expects 'organic' or 'hardsurface', got '{value}'");
                return false;
            }
        } else {
            eprintln!("Error: unknown option '{arg}'");
            print_usage(&args[0]);
            return false;
        }
        i += 1;
    }
    if params.input_path.is_empty() || params.output_path.is_empty() {
        eprintln!("Error: --input and --output are required");
        print_usage(&args[0]);
        return false;
    }
    true
}

struct ProgressState {
    last_percent: i32,
    last_status: String,
}

impl ProgressState {
    fn new() -> Self {
        Self {
            last_percent: -1,
            last_status: String::new(),
        }
    }
}

fn report_progress(tag: *mut c_void, progress: f32, status: &str) {
    let state = unsafe { &*(tag as *const Mutex<ProgressState>) };
    let mut state = state.lock().unwrap();
    let percent = (progress * 100.0) as i32;
    if percent == state.last_percent && status == state.last_status {
        return;
    }
    state.last_percent = percent;
    state.last_status = status.to_string();
    let mut out = std::io::stdout().lock();
    if status.is_empty() {
        let _ = writeln!(out, "{percent}% done.");
    } else {
        let _ = writeln!(out, "{percent}% done. {status}");
    }
    let _ = out.flush();
}

struct LoadedMesh {
    vertices: Vec<Vector3>,
    triangles: Vec<Vec<usize>>,
    pre_weld_vertices: usize,
    pre_weld_triangles: usize,
    weld_stats: WeldStats,
}

fn finish_load(
    positions: Vec<f32>,
    loaded_triangles: Vec<Vec<usize>>,
    pre_weld_vertices: usize,
    pre_weld_triangles: usize,
    weld_stats: WeldStats,
) -> LoadedMesh {
    let mut vertices = Vec::with_capacity(positions.len() / 3);
    for i in 0..positions.len() / 3 {
        vertices.push(Vector3::new(
            positions[3 * i] as f64,
            positions[3 * i + 1] as f64,
            positions[3 * i + 2] as f64,
        ));
    }
    LoadedMesh {
        vertices,
        triangles: loaded_triangles,
        pre_weld_vertices,
        pre_weld_triangles,
        weld_stats,
    }
}

fn load_obj(filename: &str) -> Option<LoadedMesh> {
    let mut positions: Vec<f32> = Vec::new();
    let mut loaded_triangles: Vec<Vec<usize>> = Vec::new();
    let mut warn = String::new();
    let mut err = String::new();
    let ok = obj_reader::load_obj_positions_and_triangles(
        Path::new(filename),
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
    Some(finish_load(
        positions,
        loaded_triangles,
        pre_weld_vertices,
        pre_weld_triangles,
        weld_stats,
    ))
}

fn load_glb(filename: &str) -> Option<LoadedMesh> {
    let mut positions: Vec<f32> = Vec::new();
    let mut loaded_triangles: Vec<Vec<usize>> = Vec::new();
    let mut warn = String::new();
    let mut err = String::new();
    let ok = glb_io::load_glb_positions_and_triangles(
        Path::new(filename),
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
    Some(finish_load(
        positions,
        loaded_triangles,
        pre_weld_vertices,
        pre_weld_triangles,
        weld_stats,
    ))
}

fn load_mesh(filename: &str) -> Option<LoadedMesh> {
    if glb_io::has_glb_extension(filename) {
        load_glb(filename)
    } else {
        load_obj(filename)
    }
}

fn warn_dropped_non_finite(non_finite_dropped: usize) {
    if non_finite_dropped > 0 {
        eprintln!(
            "Warning: dropped {non_finite_dropped} input triangles with non-finite corners (NaN or infinity)"
        );
    }
}

fn save_obj(filename: &str, vertices: &[Vector3], quads: &[Vec<usize>]) -> bool {
    let mut file = match File::create(filename) {
        Ok(file) => file,
        Err(_) => return false,
    };
    if writeln!(file, "# retopoforge {RETOPO_VERSION}").is_err() {
        return false;
    }
    if writeln!(file, "# https://github.com/chris-straka/retopoforge").is_err() {
        return false;
    }
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
    for face in quads {
        if write!(file, "f").is_err() {
            return false;
        }
        for index in face {
            if write!(file, " {}", 1 + index).is_err() {
                return false;
            }
        }
        if writeln!(file).is_err() {
            return false;
        }
    }
    file.flush().is_ok()
}

fn save_obj_with_uvs(
    filename: &str,
    vertices: &[Vector3],
    quads: &[Vec<usize>],
    uvs: &[Vector2],
) -> bool {
    let mut file = match File::create(filename) {
        Ok(file) => file,
        Err(_) => return false,
    };
    if writeln!(file, "# retopoforge {RETOPO_VERSION}").is_err() {
        return false;
    }
    if writeln!(file, "# https://github.com/chris-straka/retopoforge").is_err() {
        return false;
    }
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
    for uv in uvs {
        if writeln!(file, "vt {} {}", g_format(uv.x()), g_format(uv.y())).is_err() {
            return false;
        }
    }
    for face in quads {
        if write!(file, "f").is_err() {
            return false;
        }
        for index in face {
            if write!(file, " {}/{}", 1 + index, 1 + index).is_err() {
                return false;
            }
        }
        if writeln!(file).is_err() {
            return false;
        }
    }
    file.flush().is_ok()
}

fn save_mesh(
    filename: &str,
    vertices: &[Vector3],
    quads: &[Vec<usize>],
    uvs: Option<&[Vector2]>,
) -> bool {
    if vertices.is_empty() {
        return false;
    }
    let have_uvs = uvs.map(|u| u.len() == vertices.len()).unwrap_or(false);
    if glb_io::has_glb_extension(filename) {
        let generator = format!("retopoforge {RETOPO_VERSION}");
        if have_uvs {
            return glb_io::save_glb_with_uvs(
                Path::new(filename),
                &generator,
                vertices,
                quads,
                uvs.unwrap(),
            );
        }
        return glb_io::save_glb(Path::new(filename), &generator, vertices, quads);
    }
    if have_uvs {
        return save_obj_with_uvs(filename, vertices, quads, uvs.unwrap());
    }
    save_obj(filename, vertices, quads)
}

// `stem()`/`extension()` per `std::filesystem::path` rules (verified
// against a probe of the C++ calls): dotfiles (`.obj`) and `.`/`..`
// have no extension, a trailing dot (`foo.`) yields the `.` extension,
// otherwise the extension runs from the last dot inclusive. `ext` comes
// back with the `.obj` default applied when empty, matching both C++
// use sites (`lodOutputPath` and batch LOD naming).
fn cpp_stem_ext(file_name: &str) -> (String, String) {
    if file_name.is_empty() || file_name == "." || file_name == ".." {
        return (file_name.to_string(), ".obj".to_string());
    }
    match file_name.rfind('.') {
        None => (file_name.to_string(), ".obj".to_string()),
        Some(0) => (file_name.to_string(), ".obj".to_string()),
        Some(dot) => (file_name[..dot].to_string(), file_name[dot..].to_string()),
    }
}

fn lod_output_path(base_output: &str, lod_index: usize) -> String {
    let (parent, file_name) = match base_output.rfind('/') {
        Some(slash) => (&base_output[..slash], &base_output[slash + 1..]),
        None => ("", base_output),
    };
    let (stem, ext) = cpp_stem_ext(file_name);
    let name = format!("{stem}_lod{lod_index}{ext}");
    if parent.is_empty() {
        if base_output.starts_with('/') {
            format!("/{name}")
        } else {
            name
        }
    } else {
        format!("{parent}/{name}")
    }
}

struct RungResult {
    ok: bool,
    quad_count: usize,
    non_quad_count: usize,
    vertex_count: usize,
    island_count: usize,
    failed_islands: usize,
    elapsed_seconds: f64,
    error: String,
}

impl RungResult {
    fn failed() -> Self {
        Self {
            ok: false,
            quad_count: 0,
            non_quad_count: 0,
            vertex_count: 0,
            island_count: 0,
            failed_islands: 0,
            elapsed_seconds: 0.0,
            error: String::new(),
        }
    }
}

fn count_islands_without_output(
    islands: &[Vec<Vec<usize>>],
    input_vertices: &[Vector3],
    output_vertices: &[Vector3],
) -> usize {
    let mut failed = 0;
    for island in islands {
        let mut first = true;
        let (mut min_x, mut min_y, mut min_z) = (0.0, 0.0, 0.0);
        let (mut max_x, mut max_y, mut max_z) = (0.0, 0.0, 0.0);
        for face in island {
            for &index in face {
                let v = &input_vertices[index];
                if first {
                    min_x = v.x();
                    max_x = v.x();
                    min_y = v.y();
                    max_y = v.y();
                    min_z = v.z();
                    max_z = v.z();
                    first = false;
                } else {
                    if v.x() < min_x {
                        min_x = v.x();
                    }
                    if v.x() > max_x {
                        max_x = v.x();
                    }
                    if v.y() < min_y {
                        min_y = v.y();
                    }
                    if v.y() > max_y {
                        max_y = v.y();
                    }
                    if v.z() < min_z {
                        min_z = v.z();
                    }
                    if v.z() > max_z {
                        max_z = v.z();
                    }
                }
            }
        }
        if first {
            failed += 1;
            continue;
        }
        let dx = max_x - min_x;
        let dy = max_y - min_y;
        let dz = max_z - min_z;
        let pad = (dx * dx + dy * dy + dz * dz).sqrt() * 0.01 + 1e-6;
        let mut found = false;
        for v in output_vertices {
            if v.x() >= min_x - pad
                && v.x() <= max_x + pad
                && v.y() >= min_y - pad
                && v.y() <= max_y + pad
                && v.z() >= min_z - pad
                && v.z() <= max_z + pad
            {
                found = true;
                break;
            }
        }
        if !found {
            failed += 1;
        }
    }
    failed
}

fn dropped_island_count(
    engine_quad_counts: &[usize],
    islands: &[Vec<Vec<usize>>],
    input_vertices: &[Vector3],
    output_vertices: &[Vector3],
) -> usize {
    if engine_quad_counts.len() == islands.len() {
        return engine_quad_counts.iter().filter(|&&q| q == 0).count();
    }
    count_islands_without_output(islands, input_vertices, output_vertices)
}

fn remesh_loaded_mesh(
    params: &Params,
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    guides: Vec<Vec<Vector3>>,
    features: Vec<Vec<Vector3>>,
    density: &[f64],
    target_quads: i64,
    output_path: &str,
) -> RungResult {
    let mut result = RungResult::failed();
    let start_time = Instant::now();

    if !density.is_empty() && density.len() != vertices.len() {
        result.error = format!(
            "--density file holds {} multipliers, input has {} vertices",
            density.len(),
            vertices.len()
        );
        return result;
    }

    let mut remesher = AutoRemesher::new(vertices, triangles);
    remesher.set_target_triangle_count((target_quads as usize).wrapping_mul(2));
    remesher.set_symmetry_enabled(params.symmetry_enabled);
    remesher.set_symmetry_plane(params.symmetry_axis);
    remesher.set_guide_polylines(guides);
    remesher.set_sharp_polylines(features);
    remesher.set_density_multipliers(density);
    if params.edge_scaling > 0.0 {
        remesher.set_scaling(params.edge_scaling);
    }
    remesher.set_model_type(params.model_type);
    remesher.set_gradient_adaptivity(params.adaptivity);
    remesher.set_anisotropy(params.anisotropy);
    remesher.set_sharp_edge_degrees(params.sharp_edge_degrees);
    remesher.set_smooth_normal_degrees(params.smooth_normal_degrees);
    remesher.set_compute_remeshed_uvs(params.emit_uvs);
    remesher.set_quiet(params.quiet);
    let progress_state = Mutex::new(ProgressState::new());
    if !params.quiet {
        remesher.set_tag(&progress_state as *const Mutex<ProgressState> as *mut c_void);
        remesher.set_progress_handler(Some(report_progress));
    }

    if !remesher.remesh() {
        result.error = "remeshing produced no result".to_string();
        return result;
    }

    if !params.quiet {
        for line in remesher.phase_report() {
            eprintln!("  {line}");
        }
    }

    let remeshed_vertices = remesher.remeshed_vertices();
    let remeshed_quads = remesher.remeshed_quads();

    for face in remeshed_quads {
        if face.len() == 4 {
            result.quad_count += 1;
        } else {
            result.non_quad_count += 1;
        }
    }
    result.vertex_count = remeshed_vertices.len();

    let mut input_islands: Vec<Vec<Vec<usize>>> = Vec::new();
    MeshSeparator::split_to_islands(triangles, &mut input_islands);
    result.island_count = input_islands.len();
    result.failed_islands = dropped_island_count(
        remesher.island_output_quad_counts(),
        &input_islands,
        vertices,
        remeshed_vertices,
    );

    let uvs = if params.emit_uvs {
        Some(remesher.remeshed_vertex_uvs())
    } else {
        None
    };
    if !save_mesh(output_path, remeshed_vertices, remeshed_quads, uvs) {
        result.error = format!("failed to write {output_path}");
        return result;
    }

    result.elapsed_seconds = start_time.elapsed().as_secs_f64();
    result.ok = true;
    result
}

fn print_rung_line(label: &str, output_path: &str, target_quads: i64, result: &RungResult) {
    println!(
        "{label}target-quads={target_quads} output={output_path} quads={} non-quads={} vertices={} time={} seconds",
        result.quad_count,
        result.non_quad_count,
        result.vertex_count,
        g_format(result.elapsed_seconds),
    );
}

fn file_name_of(path: &str) -> String {
    Path::new(path)
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or(path)
        .to_string()
}

fn run_multi_mode(params: &Params, batch: bool) -> i32 {
    if batch && !params.guides_path.is_empty() {
        eprintln!("Error: --guides needs a single input mesh, not a batch directory");
        return 1;
    }
    if batch && !params.features_path.is_empty() {
        eprintln!("Error: --features needs a single input mesh, not a batch directory");
        return 1;
    }
    if batch && !params.density_path.is_empty() {
        eprintln!("Error: --density needs a single input mesh, not a batch directory");
        return 1;
    }
    let mut guides: Vec<Vec<Vector3>> = Vec::new();
    if !params.guides_path.is_empty()
        && !parse_guides_file(&params.guides_path, &mut guides, "--guides")
    {
        return 1;
    }
    let mut features: Vec<Vec<Vector3>> = Vec::new();
    if !params.features_path.is_empty()
        && !parse_guides_file(&params.features_path, &mut features, "--features")
    {
        return 1;
    }
    let mut density: Vec<f64> = Vec::new();
    if !params.density_path.is_empty() && !parse_density_file(&params.density_path, &mut density) {
        return 1;
    }
    let mut inputs: Vec<String> = Vec::new();
    if batch {
        let entries = match std::fs::read_dir(&params.input_path) {
            Ok(entries) => entries,
            Err(_) => {
                eprintln!("Error: cannot read directory {}", params.input_path);
                return 1;
            }
        };
        for entry in entries {
            let entry = match entry {
                Ok(entry) => entry,
                Err(_) => {
                    eprintln!("Error: cannot read directory {}", params.input_path);
                    return 1;
                }
            };
            // `metadata` follows symlinks, like the C++ `is_regular_file`
            // (which stats, not lstats): a symlink to a mesh counts.
            let path = entry.path();
            match std::fs::metadata(&path) {
                Ok(meta) if meta.is_file() => {}
                _ => continue,
            }
            let name = path.to_string_lossy().to_string();
            if glb_io::is_supported_input_extension(&name) {
                inputs.push(name);
            }
        }
        inputs.sort();
        if inputs.is_empty() {
            eprintln!("Error: no .obj/.glb files in {}", params.input_path);
            return 1;
        }
        let out_meta = std::fs::metadata(&params.output_path);
        if let Ok(meta) = out_meta {
            if !meta.is_dir() {
                eprintln!("Error: --output must be a directory when --input is a directory");
                return 1;
            }
        }
        if std::fs::create_dir_all(&params.output_path).is_err() {
            eprintln!(
                "Error: cannot create output directory {}",
                params.output_path
            );
            return 1;
        }
    } else {
        inputs.push(params.input_path.clone());
    }

    let targets: Vec<i64> = if params.lod_targets.is_empty() {
        vec![params.target_quads as i64]
    } else {
        params.lod_targets.clone()
    };
    let lod_mode = !params.lod_targets.is_empty();

    let mut report: Option<File> = None;
    let mut report_ok = true;
    if !params.report_path.is_empty() {
        let mut file = match File::create(&params.report_path) {
            Ok(file) => file,
            Err(_) => {
                eprintln!("Error: failed to write {}", params.report_path);
                return 1;
            }
        };
        let mut header_ok = true;
        header_ok &= writeln!(file, "retopoforge Report").is_ok();
        header_ok &= writeln!(file, "==================").is_ok();
        header_ok &= writeln!(file).is_ok();
        header_ok &= writeln!(file, "Edge scaling: {}", g_format(params.edge_scaling)).is_ok();
        header_ok &= writeln!(
            file,
            "Sharp edge degrees: {}",
            g_format(params.sharp_edge_degrees)
        )
        .is_ok();
        header_ok &= writeln!(
            file,
            "Smooth normal degrees: {}",
            g_format(params.smooth_normal_degrees)
        )
        .is_ok();
        header_ok &= writeln!(file, "Adaptivity: {}", g_format(params.adaptivity)).is_ok();
        header_ok &= writeln!(file, "Anisotropy: {}", g_format(params.anisotropy)).is_ok();
        header_ok &= writeln!(
            file,
            "Model type: {}",
            if params.model_type == ModelType::Organic {
                "organic"
            } else {
                "hardsurface"
            }
        )
        .is_ok();
        header_ok &= writeln!(file).is_ok();
        report_ok &= header_ok;
        report = Some(file);
    }

    let mut failed_files: Vec<String> = Vec::new();

    for input_path in &inputs {
        let file_label = if batch {
            let name = file_name_of(input_path);
            if lod_mode {
                format!("FILE {name} ")
            } else {
                format!("FILE {name}: ")
            }
        } else {
            String::new()
        };
        let fail_name = if batch {
            file_name_of(input_path)
        } else {
            input_path.clone()
        };

        let loaded = match load_mesh(input_path) {
            Some(loaded) => loaded,
            None => {
                eprintln!("Error: failed to load {input_path}");
                println!("{file_label}FAILED to load {input_path}");
                if !failed_files.contains(&fail_name) {
                    failed_files.push(fail_name.clone());
                }
                continue;
            }
        };
        if !params.quiet {
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
        warn_dropped_non_finite(loaded.weld_stats.non_finite_dropped);

        for (rung, target) in targets.iter().enumerate() {
            let output_path = if batch {
                let in_file = Path::new(input_path);
                let raw_name = in_file
                    .file_name()
                    .and_then(|n| n.to_str())
                    .unwrap_or(input_path);
                let (stem, lod_ext) = cpp_stem_ext(raw_name);
                let out_dir = PathBuf::from(&params.output_path);
                if lod_mode {
                    out_dir
                        .join(format!("{stem}_lod{rung}{lod_ext}"))
                        .to_string_lossy()
                        .to_string()
                } else {
                    out_dir.join(raw_name).to_string_lossy().to_string()
                }
            } else {
                lod_output_path(&params.output_path, rung)
            };
            let mut label = file_label.clone();
            if lod_mode {
                label.push_str(&format!("LOD {rung}: "));
            }

            let result = remesh_loaded_mesh(
                params,
                &loaded.vertices,
                &loaded.triangles,
                guides.clone(),
                features.clone(),
                &density,
                *target,
                &output_path,
            );
            if !result.ok {
                eprintln!("Error: {} ({output_path})", result.error);
                println!("{label}FAILED {}", result.error);
                if !failed_files.contains(&fail_name) {
                    failed_files.push(fail_name.clone());
                }
                continue;
            }
            if result.failed_islands > 0 {
                if batch {
                    let name = file_name_of(input_path);
                    eprintln!(
                        "Warning: FILE {name}: {} of {} islands produced no output and were dropped from the mesh",
                        result.failed_islands, result.island_count
                    );
                } else {
                    eprintln!(
                        "Warning: {} of {} islands produced no output and were dropped from the mesh",
                        result.failed_islands, result.island_count
                    );
                }
            }
            print_rung_line(&label, &output_path, *target, &result);
            if let Some(file) = report.as_mut() {
                report_ok &= writeln!(file, "Input file: {input_path}").is_ok();
                report_ok &= writeln!(file, "Output file: {output_path}").is_ok();
                report_ok &= writeln!(file, "Target quads: {target}").is_ok();
                report_ok &= writeln!(file, "Results:").is_ok();
                report_ok &= writeln!(file, "  Quads: {}", result.quad_count).is_ok();
                report_ok &= writeln!(file, "  Non-quads: {}", result.non_quad_count).is_ok();
                report_ok &= writeln!(file, "  Vertices: {}", result.vertex_count).is_ok();
                report_ok &= writeln!(
                    file,
                    "  Total time: {} seconds",
                    g_format(result.elapsed_seconds)
                )
                .is_ok();
                report_ok &= writeln!(file).is_ok();
            }
        }
    }

    if report.is_some() {
        let mut file = report.take().unwrap();
        report_ok &= file.flush().is_ok();
        // The C++ checks `fail()` after `close()`; Rust surfaces write +
        // flush errors here, while a close-time-only failure is
        // unobservable (drop closes; see DECISIONS in the lane report).
        if !report_ok {
            eprintln!("Error: failed to write {}", params.report_path);
            return 1;
        }
    }

    if batch {
        if failed_files.is_empty() {
            println!("Failed files: none");
        } else {
            print!("Failed files ({}):", failed_files.len());
            for name in &failed_files {
                print!(" {name}");
            }
            println!();
        }
    }
    if failed_files.is_empty() { 0 } else { 1 }
}

fn run_single_mode(params: &Params) -> i32 {
    let start_time = Instant::now();

    let loaded = match load_mesh(&params.input_path) {
        Some(loaded) => loaded,
        None => {
            eprintln!("Error: failed to load {}", params.input_path);
            return 1;
        }
    };
    if !params.quiet {
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
    warn_dropped_non_finite(loaded.weld_stats.non_finite_dropped);

    let mut guides: Vec<Vec<Vector3>> = Vec::new();
    if !params.guides_path.is_empty() {
        if !parse_guides_file(&params.guides_path, &mut guides, "--guides") {
            return 1;
        }
        if !params.quiet {
            eprintln!("Guide polylines: {}", guides.len());
        }
    }
    let mut features: Vec<Vec<Vector3>> = Vec::new();
    if !params.features_path.is_empty() {
        if !parse_guides_file(&params.features_path, &mut features, "--features") {
            return 1;
        }
        if !params.quiet {
            eprintln!("Feature polylines: {}", features.len());
        }
    }
    let mut density: Vec<f64> = Vec::new();
    if !params.density_path.is_empty() {
        if !parse_density_file(&params.density_path, &mut density) {
            return 1;
        }
        if density.len() != loaded.vertices.len() {
            eprintln!(
                "Error: --density file holds {} multipliers, input has {} vertices",
                density.len(),
                loaded.vertices.len()
            );
            return 1;
        }
        if !params.quiet {
            eprintln!("Density multipliers: {}", density.len());
        }
    }

    let mut remesher = AutoRemesher::new(&loaded.vertices, &loaded.triangles);
    remesher.set_target_triangle_count((params.target_quads as usize).wrapping_mul(2));
    remesher.set_symmetry_enabled(params.symmetry_enabled);
    remesher.set_symmetry_plane(params.symmetry_axis);
    remesher.set_guide_polylines(guides);
    remesher.set_sharp_polylines(features);
    remesher.set_density_multipliers(&density);
    if params.edge_scaling > 0.0 {
        remesher.set_scaling(params.edge_scaling);
    }
    remesher.set_model_type(params.model_type);
    remesher.set_gradient_adaptivity(params.adaptivity);
    remesher.set_anisotropy(params.anisotropy);
    remesher.set_sharp_edge_degrees(params.sharp_edge_degrees);
    remesher.set_smooth_normal_degrees(params.smooth_normal_degrees);
    remesher.set_compute_remeshed_uvs(params.emit_uvs);
    remesher.set_quiet(params.quiet);
    let progress_state = Mutex::new(ProgressState::new());
    if !params.quiet {
        remesher.set_tag(&progress_state as *const Mutex<ProgressState> as *mut c_void);
        remesher.set_progress_handler(Some(report_progress));
    }

    if !remesher.remesh() {
        eprintln!("Error: remeshing produced no result");
        return 1;
    }

    if !params.quiet {
        for line in remesher.phase_report() {
            eprintln!("  {line}");
        }
    }

    let remeshed_vertices = remesher.remeshed_vertices();
    let remeshed_quads = remesher.remeshed_quads();

    let mut input_islands: Vec<Vec<Vec<usize>>> = Vec::new();
    MeshSeparator::split_to_islands(&loaded.triangles, &mut input_islands);
    let failed_islands = dropped_island_count(
        remesher.island_output_quad_counts(),
        &input_islands,
        &loaded.vertices,
        remeshed_vertices,
    );
    if failed_islands > 0 {
        eprintln!(
            "Warning: {failed_islands} of {} islands produced no output and were dropped from the mesh",
            input_islands.len()
        );
    }

    let mut quad_count = 0usize;
    let mut non_quad_count = 0usize;
    for face in remeshed_quads {
        if face.len() == 4 {
            quad_count += 1;
        } else {
            non_quad_count += 1;
        }
    }

    let uvs = if params.emit_uvs {
        Some(remesher.remeshed_vertex_uvs())
    } else {
        None
    };
    if !save_mesh(&params.output_path, remeshed_vertices, remeshed_quads, uvs) {
        eprintln!("Error: failed to write {}", params.output_path);
        return 1;
    }

    let elapsed_seconds = start_time.elapsed().as_secs_f64();

    println!("=== retopoforge Report ===");
    println!("Input: {}", params.input_path);
    println!("Output: {}", params.output_path);
    println!("Islands: {}", input_islands.len());
    println!("Failed islands: {failed_islands}");
    println!("Quads: {quad_count}");
    println!("Non-quads: {non_quad_count}");
    println!("Vertices: {}", remeshed_vertices.len());
    println!("Time: {} seconds", g_format(elapsed_seconds));
    println!("==========================");

    if !params.report_path.is_empty() {
        let mut report = match File::create(&params.report_path) {
            Ok(report) => report,
            Err(_) => {
                eprintln!("Error: failed to write {}", params.report_path);
                return 1;
            }
        };
        let mut ok = true;
        ok &= writeln!(report, "retopoforge Report").is_ok();
        ok &= writeln!(report, "==================").is_ok();
        ok &= writeln!(report).is_ok();
        ok &= writeln!(report, "Input file: {}", params.input_path).is_ok();
        ok &= writeln!(report, "Output file: {}", params.output_path).is_ok();
        ok &= writeln!(report, "Target quads: {}", params.target_quads).is_ok();
        ok &= writeln!(report, "Edge scaling: {}", g_format(params.edge_scaling)).is_ok();
        ok &= writeln!(
            report,
            "Sharp edge degrees: {}",
            g_format(params.sharp_edge_degrees)
        )
        .is_ok();
        ok &= writeln!(
            report,
            "Smooth normal degrees: {}",
            g_format(params.smooth_normal_degrees)
        )
        .is_ok();
        ok &= writeln!(report, "Adaptivity: {}", g_format(params.adaptivity)).is_ok();
        ok &= writeln!(report, "Anisotropy: {}", g_format(params.anisotropy)).is_ok();
        ok &= writeln!(
            report,
            "Model type: {}",
            if params.model_type == ModelType::Organic {
                "organic"
            } else {
                "hardsurface"
            }
        )
        .is_ok();
        ok &= writeln!(report).is_ok();
        ok &= writeln!(report, "Results:").is_ok();
        ok &= writeln!(report, "  Islands: {}", input_islands.len()).is_ok();
        ok &= writeln!(report, "  Failed islands: {failed_islands}").is_ok();
        ok &= writeln!(report, "  Quads: {quad_count}").is_ok();
        ok &= writeln!(report, "  Non-quads: {non_quad_count}").is_ok();
        ok &= writeln!(report, "  Vertices: {}", remeshed_vertices.len()).is_ok();
        ok &= writeln!(
            report,
            "  Total time: {} seconds",
            g_format(elapsed_seconds)
        )
        .is_ok();
        ok &= report.flush().is_ok();
        if !ok {
            eprintln!("Error: failed to write {}", params.report_path);
            return 1;
        }
    }

    0
}

fn main() {
    // `args_os` + lossy conversion: `env::args` panics on non-UTF8 argv
    // while the C++ side handles raw bytes; lossy keeps every UTF-8 case
    // byte-exact and degrades gracefully otherwise (see DECISIONS).
    let args: Vec<String> = std::env::args_os()
        .map(|a| a.to_string_lossy().into_owned())
        .collect();
    let mut params = Params::new();
    if !parse_args(&args, &mut params) {
        flush_stdout();
        std::process::exit(1);
    }

    let batch = Path::new(&params.input_path).is_dir();
    let code = if !params.lod_targets.is_empty() || batch {
        run_multi_mode(&params, batch)
    } else {
        run_single_mode(&params)
    };
    flush_stdout();
    std::process::exit(code);
}
