//! Number formatting for CLI-owned output (report lines, OBJ vertices).
//!
//! Doubles print with `%.6g` semantics: 6 significant digits, `%e` style
//! when the decimal exponent is < -4 or >= 6, trailing zeros stripped.
//! The 6 rounded digits come from Rust's `{:.5e}` (correctly rounded), so
//! only the style selection and zero-stripping are hand-rolled. NaN
//! prints signless `nan`; infinities print `inf`/`-inf`; -0.0 keeps its
//! sign (`-0`), as `%g` does.

/// Format a double with `%.6g` semantics.
pub(crate) fn g_format(value: f64) -> String {
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
