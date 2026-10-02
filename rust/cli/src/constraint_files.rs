//! Constraint files: `--guides`/`--features` polylines and `--density`
//! multipliers.
//!
//! Guide files hold one `x y z` point per line, blank lines separate
//! polylines, `#` starts a comment. Density files hold one multiplier
//! per line with the same comment/blank rules. Error messages echo the
//! offending line's raw bytes (it may not be UTF-8), verbatim plus a
//! newline. All bytes are pinned by the CLI contract goldens.

use crate::error::CliError;
use crate::error::os_reason;
use retopo_core::vector3::Vector3;
use std::path::Path;

/// The loaded `--guides` / `--features` / `--density` inputs.
#[derive(Default)]
pub(crate) struct Constraints {
    pub(crate) guides: Vec<Vec<Vector3>>,
    pub(crate) features: Vec<Vec<Vector3>>,
    pub(crate) density: Vec<f64>,
}

/// Split file bytes into `getline`-style lines: split on `\n`, drop the
/// phantom segment a trailing newline would add, keep `\r` (blank-line
/// detection treats it as blank, and error echoes keep it).
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

fn is_blank_line(line: &[u8]) -> bool {
    !line.iter().any(|&b| b != b' ' && b != b'\t' && b != b'\r')
}

/// Split a comment-stripped line on C whitespace.
fn split_c_tokens(line: &[u8]) -> Vec<&[u8]> {
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
    tokens
}

/// Trailing-garbage rule: past the Nth whitespace-separated token only
/// `[ \t\r]` may follow — any other byte (including vertical tab)
/// fails the line.
fn trailing_blank_after(line: &[u8], tokens: usize) -> bool {
    let mut seen = 0;
    let mut j = 0;
    while j < line.len() {
        if is_c_space(line[j]) {
            j += 1;
            continue;
        }
        while j < line.len() && !is_c_space(line[j]) {
            j += 1;
        }
        seen += 1;
        if seen == tokens {
            return line[j..]
                .iter()
                .all(|&b| b == b' ' || b == b'\t' || b == b'\r');
        }
    }
    // Unreachable: callers only ask after checking the arity.
    true
}

/// One full-consumption double token: `_` and hex rejected, same as
/// argv number parsing.
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

fn strip_comment(line: &[u8]) -> &[u8] {
    match line.iter().position(|&b| b == b'#') {
        Some(hash) => &line[..hash],
        None => line,
    }
}

fn raw_error(prefix: String, line: &[u8]) -> CliError {
    let mut bytes = prefix.into_bytes();
    bytes.extend_from_slice(line);
    bytes.extend_from_slice(b"'");
    CliError::UsageRaw(bytes)
}

/// Parse a `--guides`/`--features` polyline file. Short (< 2 point)
/// fragments between blank lines are dropped; at least one surviving
/// polyline is required.
pub(crate) fn parse_guides_file(
    path: &Path,
    guides: &mut Vec<Vec<Vector3>>,
    flag_label: &str,
) -> Result<(), CliError> {
    guides.clear();
    let path = path.display().to_string();
    let data = std::fs::read(&path).map_err(|err| {
        CliError::usage(format!(
            "retopo: error: cannot open {flag_label} file '{path}': {}.",
            os_reason(&err)
        ))
    })?;
    let mut current: Vec<Vector3> = Vec::new();
    for (index, raw) in split_lines(&data).iter().enumerate() {
        let line_no = index + 1;
        let line = strip_comment(raw);
        if is_blank_line(line) {
            if current.len() >= 2 {
                guides.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
            continue;
        }
        let tokens = split_c_tokens(line);
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
            ok = trailing_blank_after(line, 3);
        }
        if !ok {
            let prefix = format!(
                "retopo: error: {flag_label} file '{path}' line {line_no} expects 'x y z', got '"
            );
            return Err(raw_error(prefix, line));
        }
        current.push(Vector3::new(values[0], values[1], values[2]));
    }
    if current.len() >= 2 {
        guides.push(std::mem::take(&mut current));
    }
    if guides.is_empty() {
        return Err(CliError::usage(format!(
            "retopo: error: {flag_label} file '{path}' holds no usable polyline (need 2+ points per polyline)."
        )));
    }
    Ok(())
}

/// Parse a `--density` multiplier file: one number per line.
pub(crate) fn parse_density_file(path: &Path, multipliers: &mut Vec<f64>) -> Result<(), CliError> {
    multipliers.clear();
    let path = path.display().to_string();
    let data = std::fs::read(&path).map_err(|err| {
        CliError::usage(format!(
            "retopo: error: cannot open --density file '{path}': {}.",
            os_reason(&err)
        ))
    })?;
    for (index, raw) in split_lines(&data).iter().enumerate() {
        let line_no = index + 1;
        let line = strip_comment(raw);
        if is_blank_line(line) {
            continue;
        }
        let tokens = split_c_tokens(line);
        let mut ok = tokens.len() == 1;
        let mut value = 0.0;
        if ok {
            match parse_double_token(tokens[0]) {
                Some(v) => value = v,
                None => ok = false,
            }
        }
        if ok {
            ok = trailing_blank_after(line, 1);
        }
        if !ok {
            let prefix = format!(
                "retopo: error: --density file '{path}' line {line_no} expects a number, got '"
            );
            return Err(raw_error(prefix, line));
        }
        multipliers.push(value);
    }
    if multipliers.is_empty() {
        return Err(CliError::usage(format!(
            "retopo: error: --density file '{path}' holds no multipliers."
        )));
    }
    Ok(())
}
