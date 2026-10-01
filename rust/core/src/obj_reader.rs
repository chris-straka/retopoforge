//! Port of `core/objreader.*`: minimal Wavefront OBJ reader (vertex
//! positions plus triangulated faces) and the weld-on-load helper.
//!
//! Line-by-line structural mirror of `core/objreader.cpp`: same functions in
//! the same order, same thresholds, same error strings. Three deliberate
//! restructures, all proven by the differential oracle
//! (`rust/core/tests/obj_reader_diff.rs`):
//! - The parser works on raw bytes with hand-rolled C-locale `strtod` /
//!   `strtol` prefix scanners (`scan_double`, `scan_long`), since Rust's
//!   `f64::from_str` rejects the hex-float, `nan(payload)` and partial-token
//!   (`1e`, `0x`) subjects the C++ parser accepts. Decimal subjects convert
//!   through `f64::from_str` (correctly rounded, like `strtod`); hex subjects
//!   through the correctly-rounded `hex_to_f64` below.
//! - `weld_positions_and_triangles` reimplements the single meshoptimizer call
//!   (`meshopt_generateVertexRemap` over 12-byte bitwise positions) natively:
//!   first-reference-order remap keyed on position bits. Output order is fully
//!   determined by the index stream, so this is exactly equivalent.
//! - The three `a * b - c * d` sites in the ear clipper use explicit
//!   [`f32::mul_add`] to replicate the C++ build's FP contraction (Clang
//!   fuses these to `fma(a, b, -(c*d))`, verified in the emitted IR; the
//!   point-in-polygon quotient does not fuse). Ear decisions flip on ulps, so
//!   strict IEEE evaluation diverges on degenerate polygons.

use std::collections::HashMap;
use std::path::Path;

fn is_space(b: u8) -> bool {
    b == b' ' || b == b'\t'
}

fn is_new_line(b: u8) -> bool {
    b == b'\r' || b == b'\n' || b == 0
}

/// C `isspace` in the C locale (what `strtod`/`strtol` skip).
fn is_c_space(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\n' | b'\x0b' | b'\x0c' | b'\r')
}

/// Byte at `i`, or NUL past the end: mirrors C `cursor[n]` reads against the
/// line's NUL terminator.
fn at(line: &[u8], i: usize) -> u8 {
    if i < line.len() { line[i] } else { 0 }
}

/// C `strtol(cursor, &end, 10)` equivalent over the NUL-terminated view of
/// `line` at `start`: skips C whitespace, takes an optional sign plus decimal
/// digits, clamps overflow to `i64::{MAX,MIN}` (C `LONG_MAX/MIN` on LP64).
/// Returns the value and the absolute end offset, or `None` when C would leave
/// `end == cursor` (no conversion).
fn scan_long(line: &[u8], start: usize) -> Option<(i64, usize)> {
    let mut i = start;
    while is_c_space(at(line, i)) {
        i += 1;
    }
    let mut neg = false;
    if at(line, i) == b'+' || at(line, i) == b'-' {
        neg = at(line, i) == b'-';
        i += 1;
    }
    let digits = i;
    let mut mag: u64 = 0;
    let mut overflow = false;
    while at(line, i).is_ascii_digit() {
        let d = (at(line, i) - b'0') as u64;
        match mag.checked_mul(10).and_then(|v| v.checked_add(d)) {
            Some(v) => mag = v,
            None => overflow = true,
        }
        i += 1;
    }
    if i == digits {
        return None;
    }
    let limit = i64::MAX as u64 + u64::from(neg);
    if overflow || mag > limit {
        return Some((if neg { i64::MIN } else { i64::MAX }, i));
    }
    let value = mag as i64;
    // `wrapping_neg`: `-i64::MIN` (from magnitude 2^63) would trap in debug.
    Some((if neg { value.wrapping_neg() } else { value }, i))
}

fn match_word_ci(line: &[u8], i: usize, word: &[u8]) -> bool {
    for (k, &c) in word.iter().enumerate() {
        if at(line, i + k).to_ascii_lowercase() != c {
            return false;
        }
    }
    true
}

fn hex_nibble(b: u8) -> u32 {
    match b {
        b'0'..=b'9' => (b - b'0') as u32,
        b'a'..=b'f' => (b - b'a') as u32 + 10,
        b'A'..=b'F' => (b - b'A') as u32 + 10,
        _ => 0,
    }
}

/// `nan(payload)` payload rule, reverse-engineered from the platform libc
/// (see the `nan(...)` battery in the differential fixture): lowercase-`0x`
/// hex, `0`-led octal-strict, or decimal, fully consumed, wrapping to 64 bits
/// (overflow wraps, it does not clamp); anything else is payload 0.
fn nan_payload(seq: &[u8]) -> u64 {
    if seq.is_empty() {
        return 0;
    }
    if seq.len() > 2
        && seq[0] == b'0'
        && seq[1] == b'x'
        && seq[2..].iter().all(|b| b.is_ascii_hexdigit())
    {
        let mut v: u64 = 0;
        for &b in &seq[2..] {
            v = v.wrapping_mul(16).wrapping_add(hex_nibble(b) as u64);
        }
        return v;
    }
    if seq.len() > 1 && seq[0] == b'0' {
        if !seq[1..].iter().all(|b| matches!(b, b'0'..=b'7')) {
            return 0;
        }
        let mut v: u64 = 0;
        for &b in &seq[1..] {
            v = v.wrapping_mul(8).wrapping_add((b - b'0') as u64);
        }
        return v;
    }
    if !seq.iter().all(|b| b.is_ascii_digit()) {
        return 0;
    }
    let mut v: u64 = 0;
    for &b in seq {
        v = v.wrapping_mul(10).wrapping_add((b - b'0') as u64);
    }
    v
}

/// Correctly-rounded hex float to `f64`: the nibble stream (int part then
/// frac part) is the integer `D`, and the value is `D * 2^(pexp - 4*fraclen)`.
/// The top 64 significant bits plus a sticky bit feed round-to-nearest-even,
/// with an explicit subnormal path below 2^-1022.
fn hex_to_f64(neg: bool, int_hex: &[u8], frac_hex: &[u8], pexp: i64) -> f64 {
    let sign = if neg { 0x8000_0000_0000_0000u64 } else { 0 };
    let mut acc: u64 = 0;
    let mut kept = 0usize;
    let mut total_sig = 0usize;
    let mut sticky = false;
    let mut seen_nonzero = false;
    for &b in int_hex.iter().chain(frac_hex.iter()) {
        let n = hex_nibble(b);
        if !seen_nonzero {
            if n == 0 {
                continue;
            }
            seen_nonzero = true;
        }
        total_sig += 1;
        if kept < 16 {
            acc = (acc << 4) | n as u64;
            kept += 1;
        } else if n != 0 {
            sticky = true;
        }
    }
    if !seen_nonzero {
        return f64::from_bits(sign);
    }
    // Normalized: bit 63 is the leading 1; value = acc * 2^(exp - 63).
    let lz = acc.leading_zeros();
    acc <<= lz;
    let exp: i128 = pexp as i128 - 4 * frac_hex.len() as i128 + 63 - lz as i128
        + 4 * total_sig.saturating_sub(16) as i128;
    if exp > 1023 {
        return f64::from_bits(sign | 0x7ff0_0000_0000_0000);
    }
    if exp >= -1022 {
        let mut mant = acc >> 11;
        let guard = (acc >> 10) & 1;
        let round = (acc >> 9) & 1;
        let rest = (acc & 0x1ff) | u64::from(sticky);
        if guard == 1 && (round == 1 || rest != 0 || mant & 1 == 1) {
            mant += 1;
            if mant >> 53 != 0 {
                // Rounded up to 1.0 * 2^(exp+1).
                if exp + 1 > 1023 {
                    return f64::from_bits(sign | 0x7ff0_0000_0000_0000);
                }
                return f64::from_bits(sign | (((exp + 1 + 1023) as u64) << 52));
            }
        }
        return f64::from_bits(
            sign | (((exp + 1023) as u64) << 52) | (mant & 0x000f_ffff_ffff_ffff),
        );
    }
    // Subnormal: quotient in units of 2^-1074, rounded to nearest even.
    let sh = -(exp + 1011);
    if sh >= 128 {
        return f64::from_bits(sign);
    }
    let sh = sh as u32;
    let mut q = if sh >= 64 { 0 } else { acc >> sh };
    let guard = if sh == 0 || sh - 1 >= 64 {
        0
    } else {
        (acc >> (sh - 1)) & 1
    };
    let round = if sh < 2 || sh - 2 >= 64 {
        0
    } else {
        (acc >> (sh - 2)) & 1
    };
    if sh > 2 {
        if sh - 2 >= 64 {
            sticky = true; // acc != 0 always (bit 63 set)
        } else if acc & ((1u64 << (sh - 2)) - 1) != 0 {
            sticky = true;
        }
    }
    if guard == 1 && (round == 1 || sticky || q & 1 == 1) {
        q += 1;
        if q >= 1 << 52 {
            // Rounded up to the smallest normal.
            return f64::from_bits(sign | (1 << 52));
        }
    }
    f64::from_bits(sign | q)
}

/// Hex-float subject just after `0x`/`0X`. Returns `None` when no hex digit
/// follows (the caller falls back to decimal: `0x` alone parses as `0`).
fn scan_hex_float(line: &[u8], start: usize, neg: bool) -> Option<(f64, usize)> {
    let mut j = start;
    while at(line, j).is_ascii_hexdigit() {
        j += 1;
    }
    let int_end = j;
    let mut frac_start = j;
    let mut frac_end = j;
    let mut end = j;
    if at(line, j) == b'.' {
        frac_start = j + 1;
        let mut f = frac_start;
        while at(line, f).is_ascii_hexdigit() {
            f += 1;
        }
        frac_end = f;
        if int_end > start || frac_end > frac_start {
            end = frac_end;
        }
    }
    if int_end == start && frac_end == frac_start {
        return None;
    }
    j = end;
    if at(line, j) == b'p' || at(line, j) == b'P' {
        let mut e = j + 1;
        let mut pneg = false;
        if at(line, e) == b'+' || at(line, e) == b'-' {
            pneg = at(line, e) == b'-';
            e += 1;
        }
        let exp_start = e;
        let mut pexp: i64 = 0;
        while at(line, e).is_ascii_digit() {
            pexp = pexp
                .saturating_mul(10)
                .saturating_add((at(line, e) - b'0') as i64);
            e += 1;
        }
        if e > exp_start {
            end = e;
            let value = hex_to_f64(
                neg,
                &line[start..int_end],
                &line[frac_start..frac_end],
                if pneg { -pexp } else { pexp },
            );
            return Some((value, end));
        }
    }
    let value = hex_to_f64(neg, &line[start..int_end], &line[frac_start..frac_end], 0);
    Some((value, end))
}

/// C `strtod(cursor, &end)` equivalent (C locale) over the NUL-terminated view
/// of `line` at `start`. Returns the value and the absolute end offset, or
/// `None` when C would leave `end == cursor` (no conversion).
fn scan_double(line: &[u8], start: usize) -> Option<(f64, usize)> {
    let mut i = start;
    while is_c_space(at(line, i)) {
        i += 1;
    }
    let num_start = i;
    let mut neg = false;
    if at(line, i) == b'+' || at(line, i) == b'-' {
        neg = at(line, i) == b'-';
        i += 1;
    }
    // Infinity: `inf` plus optional `inity`, case-insensitive.
    if match_word_ci(line, i, b"inf") {
        i += 3;
        if match_word_ci(line, i, b"inity") {
            i += 5;
        }
        return Some((
            if neg {
                f64::NEG_INFINITY
            } else {
                f64::INFINITY
            },
            i,
        ));
    }
    // NaN with optional `(payload)`; the parens are consumed only when closed.
    if match_word_ci(line, i, b"nan") {
        i += 3;
        let mut payload: u64 = 0;
        if at(line, i) == b'(' {
            let mut j = i + 1;
            while at(line, j) != 0 && at(line, j) != b')' {
                j += 1;
            }
            if at(line, j) == b')' {
                payload = nan_payload(&line[i + 1..j]);
                i = j + 1;
            }
        }
        let mut bits = 0x7ff8_0000_0000_0000u64 | (payload & 0x000f_ffff_ffff_ffffu64);
        if neg {
            bits |= 0x8000_0000_0000_0000;
        }
        return Some((f64::from_bits(bits), i));
    }
    // Hex float; on no hex digits fall through to decimal (`0x` parses as 0).
    if at(line, i) == b'0' && (at(line, i + 1) == b'x' || at(line, i + 1) == b'X') {
        if let Some(found) = scan_hex_float(line, i + 2, neg) {
            return Some(found);
        }
    }
    // Decimal: digits [.digits] | .digits, optional exponent with backtrack.
    let int_start = i;
    while at(line, i).is_ascii_digit() {
        i += 1;
    }
    let int_digits = i - int_start;
    let mut end = i;
    if at(line, i) == b'.' {
        let mut f = i + 1;
        while at(line, f).is_ascii_digit() {
            f += 1;
        }
        if int_digits > 0 || f > i + 1 {
            end = f;
        }
    }
    if int_digits == 0 && end == int_start {
        return None;
    }
    i = end;
    if at(line, i) == b'e' || at(line, i) == b'E' {
        let mut e = i + 1;
        if at(line, e) == b'+' || at(line, e) == b'-' {
            e += 1;
        }
        let exp_start = e;
        while at(line, e).is_ascii_digit() {
            e += 1;
        }
        if e > exp_start {
            end = e;
        }
    }
    let subject = &line[num_start..end];
    // The scanned subject always matches Rust's decimal grammar (correctly
    // rounded, like `strtod`); map the impossible failure to no-conversion.
    match std::str::from_utf8(subject).ok()?.parse::<f64>().ok() {
        Some(v) => Some((v, end)),
        None => None,
    }
}

// Zero-based index with relative-index support, mirroring tinyobj's fixIndex:
// positive values are 1-based, negatives are relative to the end, zero is
// invalid. Unlike tinyobj, positive values are also range-checked.
fn fix_index(idx: i32, count: usize) -> Option<usize> {
    if idx > 0 {
        let zero_based = idx as usize - 1;
        if zero_based >= count {
            return None;
        }
        return Some(zero_based);
    }
    if idx == 0 {
        return None;
    }
    // `count` is `positions.len() / 3`, far below `i64::MAX`, so this cannot
    // overflow.
    let resolved = count as i64 + idx as i64;
    if resolved < 0 {
        return None;
    }
    Some(resolved as usize)
}

// Parses the leading vertex index of one face corner (`v`, `v/vt`, `v//vn` or
// `v/vt/vn`); texture and normal parts are ignored.
fn parse_face_corner(line: &[u8], cursor: &mut usize, vertex_count: usize) -> Option<usize> {
    let (idx, end) = scan_long(line, *cursor)?;
    // The index narrows to int below; a value outside int range (or a strtol
    // overflow) must fail instead of wrapping into a valid index.
    if idx > i32::MAX as i64 || idx < i32::MIN as i64 {
        return None;
    }
    *cursor = end;
    while at(line, *cursor) == b'/' {
        *cursor += 1;
    }
    while at(line, *cursor) != 0 && !is_space(at(line, *cursor)) && !is_new_line(at(line, *cursor))
    {
        if at(line, *cursor) == b'/' {
            *cursor += 1;
            continue;
        }
        match scan_long(line, *cursor) {
            Some((_, part_end)) => {
                *cursor = part_end;
            }
            None => {
                *cursor += 1;
            }
        }
    }
    fix_index(idx as i32, vertex_count)
}

// Point-in-polygon test after W. Randolph Franklin's pnpoly
// (https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html).
fn point_in_polygon(nvert: usize, vertx: &[f32], verty: &[f32], testx: f32, testy: f32) -> bool {
    let mut c = false;
    let mut j = nvert - 1;
    for i in 0..nvert {
        if (verty[i] > testy) != (verty[j] > testy)
            && testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]
        {
            c = !c;
        }
        j = i;
    }
    c
}

fn position_valid(vertex_index: usize, component: usize, positions: &[f32]) -> bool {
    vertex_index * 3 + component < positions.len()
}

// Ear-clipping triangulation matching the triangulation tinyobjloader applied
// to polygon faces: project to the dominant plane, then clip ears starting
// from the first corner.
fn triangulate_face(positions: &[f32], corners: &[usize], out: &mut Vec<Vec<usize>>) {
    let corner_count = corners.len();
    if corner_count < 3 {
        return;
    }

    // Find the two axes to work in.
    let mut axes = [1usize, 2usize];
    for k in 0..corner_count {
        let vi0 = corners[(k) % corner_count];
        let vi1 = corners[(k + 1) % corner_count];
        let vi2 = corners[(k + 2) % corner_count];
        if 3 * vi0 + 2 >= positions.len()
            || 3 * vi1 + 2 >= positions.len()
            || 3 * vi2 + 2 >= positions.len()
        {
            continue;
        }
        let v0x = positions[vi0 * 3];
        let v0y = positions[vi0 * 3 + 1];
        let v0z = positions[vi0 * 3 + 2];
        let v1x = positions[vi1 * 3];
        let v1y = positions[vi1 * 3 + 1];
        let v1z = positions[vi1 * 3 + 2];
        let v2x = positions[vi2 * 3];
        let v2y = positions[vi2 * 3 + 1];
        let v2z = positions[vi2 * 3 + 2];
        let e0x = v1x - v0x;
        let e0y = v1y - v0y;
        let e0z = v1z - v0z;
        let e1x = v2x - v1x;
        let e1y = v2y - v1y;
        let e1z = v2z - v1z;
        // FP contraction (see module docs): the C++ build fuses each of
        // these to one rounding (`fma(a, b, c * -d)`).
        let cx = e0y.mul_add(e1z, e0z * -e1y).abs();
        let cy = e0z.mul_add(e1x, e0x * -e1z).abs();
        let cz = e0x.mul_add(e1y, e0y * -e1x).abs();
        let epsilon = f32::EPSILON;
        if cx > epsilon || cy > epsilon || cz > epsilon {
            // Found a corner.
            if cx > cy && cx > cz {
            } else {
                axes[0] = 0;
                if cz > cx && cz > cy {
                    axes[1] = 1;
                }
            }
            break;
        }
    }

    let mut area = 0.0f32;
    for k in 0..corner_count {
        let vi0 = corners[(k) % corner_count];
        let vi1 = corners[(k + 1) % corner_count];
        if !position_valid(vi0, axes[0], positions)
            || !position_valid(vi0, axes[1], positions)
            || !position_valid(vi1, axes[0], positions)
            || !position_valid(vi1, axes[1], positions)
        {
            continue;
        }
        let v0x = positions[vi0 * 3 + axes[0]];
        let v0y = positions[vi0 * 3 + axes[1]];
        let v1x = positions[vi1 * 3 + axes[0]];
        let v1y = positions[vi1 * 3 + axes[1]];
        // FP contraction (see module docs): the C++ build fuses both the
        // inner difference and the `+ area` accumulation.
        area = v0x.mul_add(v1y, v0y * -v1x).mul_add(0.5, area);
    }

    let mut remaining: Vec<usize> = corners.to_vec();
    let mut guess_vert = 0usize;
    let mut remaining_iterations = corner_count;
    let mut previous_remaining_vertices = remaining.len();

    while remaining.len() > 3 && remaining_iterations > 0 {
        let npolys = remaining.len();
        if guess_vert >= npolys {
            guess_vert -= npolys;
        }

        if previous_remaining_vertices != npolys {
            // The number of remaining vertices decreased. Reset counters.
            previous_remaining_vertices = npolys;
            remaining_iterations = npolys;
        } else {
            // We didn't consume a vertex on previous iteration, reduce the
            // available iterations.
            remaining_iterations -= 1;
        }

        let mut ind = [0usize; 3];
        let mut vx = [0.0f32; 3];
        let mut vy = [0.0f32; 3];
        for k in 0..3 {
            ind[k] = remaining[(guess_vert + k) % npolys];
            if !position_valid(ind[k], axes[0], positions)
                || !position_valid(ind[k], axes[1], positions)
            {
                vx[k] = 0.0;
                vy[k] = 0.0;
            } else {
                vx[k] = positions[ind[k] * 3 + axes[0]];
                vy[k] = positions[ind[k] * 3 + axes[1]];
            }
        }
        let e0x = vx[1] - vx[0];
        let e0y = vy[1] - vy[0];
        let e1x = vx[2] - vx[1];
        let e1y = vy[2] - vy[1];
        // FP contraction (see module docs): the C++ build fuses this.
        let cross = e0x.mul_add(e1y, e0y * -e1x);
        // If an internal angle.
        if cross * area < 0.0 {
            guess_vert += 1;
            continue;
        }

        // Check all other verts in case they are inside this triangle.
        let mut overlap = false;
        for other_vert in 3..npolys {
            let idx = (guess_vert + other_vert) % npolys;
            if idx >= remaining.len() {
                continue;
            }
            let ovi = remaining[idx];
            if !position_valid(ovi, axes[0], positions) || !position_valid(ovi, axes[1], positions)
            {
                continue;
            }
            let tx = positions[ovi * 3 + axes[0]];
            let ty = positions[ovi * 3 + axes[1]];
            if point_in_polygon(3, &vx, &vy, tx, ty) {
                overlap = true;
                break;
            }
        }

        if overlap {
            guess_vert += 1;
            continue;
        }

        // This triangle is an ear.
        out.push(vec![ind[0], ind[1], ind[2]]);

        // Remove v1 from the list.
        remaining.remove((guess_vert + 1) % npolys);
    }

    if remaining.len() == 3 {
        out.push(vec![remaining[0], remaining[1], remaining[2]]);
    }
}

fn parse_obj_bytes(
    bytes: &[u8],
    positions: &mut Vec<f32>,
    triangles: &mut Vec<Vec<usize>>,
) -> Result<(), String> {
    let mut local_positions: Vec<f32> = Vec::new();
    let mut local_triangles: Vec<Vec<usize>> = Vec::new();
    let mut line_number = 0usize;
    // Like C++ `getline` (delimiter `\n` extracted). A trailing `\n` yields one
    // phantom empty line, which is a no-op below (line numbers only matter for
    // error messages, and an empty line cannot fail).
    for line in bytes.split(|&b| b == b'\n') {
        line_number += 1;
        let mut cursor = 0usize;
        while at(line, cursor) == b' ' || at(line, cursor) == b'\t' {
            cursor += 1;
        }
        if is_new_line(at(line, cursor)) || at(line, cursor) == b'#' {
            continue;
        }

        // Vertex position (`v` only; `vn`, `vt`, `vp` are ignored).
        if at(line, cursor) == b'v' && is_space(at(line, cursor + 1)) {
            cursor += 2;
            let mut xyz = [0.0f32; 3];
            for i in 0..3 {
                while at(line, cursor) == b' ' || at(line, cursor) == b'\t' {
                    cursor += 1;
                }
                if is_new_line(at(line, cursor)) || at(line, cursor) == b'#' {
                    break;
                }
                let (value, end) = match scan_double(line, cursor) {
                    Some(found) => found,
                    None => break,
                };
                xyz[i] = value as f32;
                cursor = end;
            }
            local_positions.push(xyz[0]);
            local_positions.push(xyz[1]);
            local_positions.push(xyz[2]);
            continue;
        }

        // Face.
        if at(line, cursor) == b'f' && is_space(at(line, cursor + 1)) {
            cursor += 2;
            let mut corners: Vec<usize> = Vec::new();
            let mut failed = false;
            loop {
                while at(line, cursor) == b' ' || at(line, cursor) == b'\t' {
                    cursor += 1;
                }
                if is_new_line(at(line, cursor)) || at(line, cursor) == b'#' {
                    break;
                }
                match parse_face_corner(line, &mut cursor, local_positions.len() / 3) {
                    Some(index) => corners.push(index),
                    None => {
                        failed = true;
                        break;
                    }
                }
            }
            if failed {
                return Err(format!(
                    "Failed parse `f' line(e.g. zero value for face index. line {}.)\n",
                    line_number
                ));
            }
            triangulate_face(&local_positions, &corners, &mut local_triangles);
        }
    }

    *positions = local_positions;
    *triangles = local_triangles;
    Ok(())
}

/// Minimal Wavefront OBJ reader: vertex positions plus triangulated faces
/// only (texture coordinates, normals, materials and groups are ignored).
/// Mirrors the tinyobjloader behavior the callers relied on: positions are
/// parsed as doubles rounded to float, face indices are 1-based with negative
/// relative indices supported, and polygons are ear-clip triangulated.
///
/// On failure the outputs are left untouched; `warn`/`err` mirror the C++
/// out-params (pass `None` to ignore one).
pub fn load_obj_positions_and_triangles(
    filename: &Path,
    positions: &mut Vec<f32>,
    triangles: &mut Vec<Vec<usize>>,
    warn: Option<&mut String>,
    err: Option<&mut String>,
) -> bool {
    let mut file = match std::fs::File::open(filename) {
        Ok(file) => file,
        Err(_) => {
            if let Some(w) = warn {
                w.clear();
            }
            if let Some(e) = err {
                *e = format!("Cannot open file [{}]\n", filename.display());
            }
            return false;
        }
    };
    // Mirror C++: read errors just end the `getline` loop (partial success);
    // only open failure is an error. (A directory opens fine and yields zero
    // lines: success with empty outputs.)
    let mut bytes: Vec<u8> = Vec::new();
    let _ = std::io::Read::read_to_end(&mut file, &mut bytes);
    match parse_obj_bytes(&bytes, positions, triangles) {
        Ok(()) => {
            if let Some(w) = warn {
                w.clear();
            }
            if let Some(e) = err {
                e.clear();
            }
            true
        }
        Err(message) => {
            if let Some(w) = warn {
                w.clear();
            }
            if let Some(e) = err {
                *e = message;
            }
            false
        }
    }
}

/// Per-reason drop counts from [`weld_positions_and_triangles`] (optional out
/// param; both are zero for clean input, which takes the identity path).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WeldStats {
    /// Index-degenerate triangles (two corners sharing an index), counted
    /// both before the remap and after it (welding can collapse
    /// distinct-but-coincident corners onto one vertex).
    pub degenerate_dropped: usize,
    /// Triangles with a NaN or infinite corner, which would otherwise poison
    /// area-weighted sizing for the whole run.
    pub non_finite_dropped: usize,
}

/// Weld coincident vertices and drop degenerate triangles in place.
/// AI exporters emit non-indexed triangle soup (one vertex copy per face
/// corner); without welding, the island splitter sees thousands of islands
/// and the remesh degrades. Vertices are merged by bitwise position equality;
/// a triangle is degenerate when two of its corners share an index, checked
/// both before and after the remap (welding can collapse
/// distinct-but-coincident corners). Faces with a non-finite (NaN/inf) corner
/// are dropped as well: one NaN corner makes the mesh area NaN, which used to
/// fail every island of the run. Already-welded input is left untouched
/// (identity fast path), unreferenced vertices are removed, and non-triangle
/// faces pass through unchanged.
pub fn weld_positions_and_triangles(
    positions: &mut Vec<f32>,
    triangles: &mut Vec<Vec<usize>>,
    mut stats: Option<&mut WeldStats>,
) {
    if let Some(s) = stats.as_deref_mut() {
        *s = WeldStats::default();
    }

    // Any corner whose position is NaN or infinite poisons the face. Faces
    // with out-of-range corners cannot be judged here and are kept: the
    // flatten step below bails out on them, and engine input validation
    // rejects the mesh loudly instead of indexing out of bounds.
    let face_has_non_finite_corner = |face: &[usize], positions: &[f32]| -> bool {
        if positions.len() % 3 != 0 {
            return false;
        }
        let vertex_count = positions.len() / 3;
        for &corner in face {
            if corner >= vertex_count {
                return false;
            }
            for component in 0..3 {
                if !positions[corner * 3 + component].is_finite() {
                    return true;
                }
            }
        }
        false
    };

    // Drop index-degenerate triangles first: two corners sharing an index can
    // never form an area, before or after welding. Non-finite-cornered
    // triangles go in the same pass: one NaN corner makes the whole mesh area
    // NaN, which used to fail every island of the run.
    {
        let mut kept: Vec<Vec<usize>> = Vec::with_capacity(triangles.len());
        let mut degenerate_dropped = 0usize;
        let mut non_finite_dropped = 0usize;
        for face in triangles.iter() {
            if face.len() == 3 && (face[0] == face[1] || face[1] == face[2] || face[0] == face[2]) {
                degenerate_dropped += 1;
                continue;
            }
            if face.len() == 3 && face_has_non_finite_corner(face, positions) {
                non_finite_dropped += 1;
                continue;
            }
            kept.push(face.clone());
        }
        *triangles = kept;
        if let Some(s) = stats.as_deref_mut() {
            s.degenerate_dropped = degenerate_dropped;
            s.non_finite_dropped = non_finite_dropped;
        }
    }

    if triangles.is_empty() || positions.is_empty() || positions.len() % 3 != 0 {
        return;
    }
    let vertex_count = positions.len() / 3;
    if vertex_count > u32::MAX as usize {
        return;
    }

    // Flatten the triangle faces to a 32-bit index buffer, bailing out
    // (degenerates already dropped) on any corner the remapper cannot see:
    // non-triangle faces stay in place and out-of-range indices are left for
    // the caller to deal with.
    let mut indices: Vec<u32> = Vec::with_capacity(triangles.len() * 3);
    for face in triangles.iter() {
        if face.len() != 3 {
            return;
        }
        for &corner in face {
            if corner >= vertex_count {
                return;
            }
            indices.push(corner as u32);
        }
    }
    if indices.is_empty() {
        return;
    }

    // Native replacement for `meshopt_generateVertexRemap` over 12-byte
    // positions plus the two remap calls: new ids are assigned in
    // first-reference order over the index stream, keyed on bitwise position
    // equality, which is exactly meshopt's observable behavior (its hash only
    // affects collision probing, never the result). Unreferenced vertices keep
    // the `u32::MAX` sentinel and are dropped, as in meshopt.
    let mut remap = vec![u32::MAX; vertex_count];
    let mut new_id_of_bits: HashMap<[u32; 3], u32> = HashMap::new();
    let mut welded_positions: Vec<f32> = Vec::new();
    for &index in indices.iter() {
        let old = index as usize;
        if remap[old] != u32::MAX {
            continue;
        }
        let key = [
            positions[old * 3].to_bits(),
            positions[old * 3 + 1].to_bits(),
            positions[old * 3 + 2].to_bits(),
        ];
        let new_id = *new_id_of_bits.entry(key).or_insert_with(|| {
            let id = (welded_positions.len() / 3) as u32;
            welded_positions.push(positions[old * 3]);
            welded_positions.push(positions[old * 3 + 1]);
            welded_positions.push(positions[old * 3 + 2]);
            id
        });
        remap[old] = new_id;
    }
    let welded_vertex_count = new_id_of_bits.len();

    let mut identity = welded_vertex_count == vertex_count;
    for (i, &r) in remap.iter().enumerate() {
        if !identity {
            break;
        }
        identity = r == i as u32;
    }
    if identity {
        return;
    }
    for index in indices.iter_mut() {
        *index = remap[*index as usize];
    }

    // Welding can collapse a triangle's distinct corners onto one vertex.
    let mut welded_triangles: Vec<Vec<usize>> = Vec::with_capacity(indices.len() / 3);
    let mut collapsed_dropped = 0usize;
    let mut i = 0usize;
    while i + 2 < indices.len() {
        let a = indices[i] as usize;
        let b = indices[i + 1] as usize;
        let c = indices[i + 2] as usize;
        if a == b || b == c || a == c {
            collapsed_dropped += 1;
        } else {
            welded_triangles.push(vec![a, b, c]);
        }
        i += 3;
    }
    if let Some(s) = stats.as_deref_mut() {
        s.degenerate_dropped += collapsed_dropped;
    }

    *positions = welded_positions;
    *triangles = welded_triangles;
}
