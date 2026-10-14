//! Port of `cli/glb.*`: GLB input/output for the Qt-free CLI.
//!
//! Line-by-line structural mirror: same functions in the same order, same
//! warning/error strings, same true/false contract. Two deliberate
//! restructures, both proven by the differential oracle
//! (`rust/core/tests/glb_diff.rs`):
//! - The reader replaces cgltf with a std-only minimal GLB reader covering
//!   exactly the subset `loadGlbPositionsAndTriangles` exercises: the GLB
//!   container (magic/version/length, JSON chunk, optional BIN chunk, plus
//!   cgltf's raw-JSON fallback for non-magic files), the JSON sections it
//!   touches (accessors, bufferViews, buffers, meshes with
//!   primitives/attributes/indices/mode/targets, nodes with
//!   mesh/children/matrix/TRS, scenes, scene), float VEC3 POSITION and
//!   u8/u16/u32 index accessors (with cgltf's conversion tables mirrored for
//!   the other component kinds), node world transforms, scene flattening
//!   with the unreferenced-mesh fallback, and every warn/err string.
//! - The writer's `%g` min/max formatting (C++ ostream defaults) is
//!   reimplemented exactly (`format_g_f32` below), so output bytes are
//!   identical.
//!
//! Deliberate boundaries (all unreachable from the CLI's self-contained GLB
//!   use case; each would need a file no exporter writes):
//! - Buffers with a `uri` (external `.bin` or base64 data URI) fail with the
//!   load-buffers string; C++ loads the referenced bytes when present.
//! - Materials, textures, skins, animations, images, and their index fixups
//!   are unparsed: dangling refs there fail the C++ parse but are ignored
//!   here. Attribute/target values ARE validated as required indices.
//! - The JSON parser is strict except for `inf`/`nan` bare words (any
//!   case, optional sign), which the writer itself emits for non-finite
//!   min/max: other inputs jsmn tolerates but JSON forbids (`+1`, partial
//!   exponents, trailing commas, ...) fail here.
//! - FP fusion: Clang fuses the transform/apply multiply-add chains at -O3
//!   (53 `llvm.fmuladd` sites, TRS compose vectorized), so C++ itself has no
//!   stable bitwise contract on inexact intermediates (-O0 differs too).
//!   This port evaluates in C++ abstract-machine (source) order: inputs
//!   whose intermediates are exactly representable match bitwise, general
//!   inputs may differ by ~1 ulp (the oracle pins exact cases bitwise and
//!   inexact-transform cases within tolerance).
//! - Out-of-bounds reads are UB in C++ (cgltf never bounds-checks); this
//!   port fails loudly with the same read-error strings instead.
//! - Cyclic parenting hangs C++ (`while (parent)`); this port reports the
//!   parse-error string instead.

use crate::{vector2::Vector2, vector3::Vector3};
use std::path::Path;

// ---------------------------------------------------------------------------
// Extension checks (mirror lowerExtension/hasGlbExtension/
// isSupportedInputExtension; the backslash counts as a separator like in C++).
// ---------------------------------------------------------------------------

fn lower_extension(path: &str) -> String {
    let dot = match path.rfind('.') {
        Some(dot) => dot,
        None => return String::new(),
    };
    // A trailing dot or a dot in a directory component is not an extension.
    if let Some(slash) = path.rfind(['/', '\\'])
        && dot < slash
    {
        return String::new();
    }
    path[dot..].to_ascii_lowercase()
}

/// Case-insensitive ".glb" extension check (".GLB" from AI exporters counts).
#[must_use]
pub fn has_glb_extension(path: &str) -> bool {
    lower_extension(path) == ".glb"
}

/// Case-insensitive ".obj"/".glb" check for batch-mode input scanning.
#[must_use]
pub fn is_supported_input_extension(path: &str) -> bool {
    let ext = lower_extension(path);
    ext == ".obj" || ext == ".glb"
}

// ---------------------------------------------------------------------------
// Minimal strict JSON parser (std-only).
//
// Strings keep their RAW inner text (escapes validated but not decoded):
// cgltf compares keys and attribute names against the raw token bytes, so
// an escaped spelling never matches anywhere.
// Numbers keep their raw text; int/float fields convert with C atoi/atof
// semantics (`c_atoll`, `prim_float`), so `1e3` counts as 1 in an int field
// and `true`/`null` count as 0, exactly like the C++ converters.
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
enum Json {
    Null,
    Bool(bool),
    /// Raw number text (validated against the strict JSON number grammar).
    Num(String),
    /// Raw string contents (escapes validated, NOT decoded).
    Str(String),
    Array(Vec<Json>),
    Object(Vec<(String, Json)>),
}

impl Json {
    /// Last-wins member lookup (cgltf re-assigns duplicate keys in order).
    fn member(&self, key: &str) -> Option<&Json> {
        if let Json::Object(members) = self {
            members.iter().rev().find(|(k, _)| k == key).map(|(_, v)| v)
        } else {
            None
        }
    }
    fn as_array(&self) -> Option<&Vec<Json>> {
        if let Json::Array(items) = self {
            Some(items)
        } else {
            None
        }
    }
    fn as_object(&self) -> Option<&Vec<(String, Json)>> {
        if let Json::Object(members) = self {
            Some(members)
        } else {
            None
        }
    }
    fn as_str(&self) -> Option<&str> {
        if let Json::Str(s) = self {
            Some(s)
        } else {
            None
        }
    }
    /// Raw primitive text for the C converters: numbers verbatim, bools and
    /// null by spelling (what jsmn hands cgltf, which atoi/atof then read).
    fn prim_text(&self) -> Option<&str> {
        match self {
            Json::Num(s) => Some(s),
            Json::Bool(true) => Some("true"),
            Json::Bool(false) => Some("false"),
            Json::Null => Some("null"),
            _ => None,
        }
    }
}

struct Parser<'a> {
    bytes: &'a [u8],
    pos: usize,
    depth: usize,
}

const MAX_JSON_DEPTH: usize = 1000;

fn is_json_space(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\n' | b'\r')
}

fn is_json_digit(b: u8) -> bool {
    b.is_ascii_digit()
}

impl<'a> Parser<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            pos: 0,
            depth: 0,
        }
    }

    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    fn skip_space(&mut self) {
        while let Some(b) = self.peek() {
            if is_json_space(b) {
                self.pos += 1;
            } else {
                break;
            }
        }
    }

    fn expect_byte(&mut self, want: u8) -> Result<(), ()> {
        if self.peek() == Some(want) {
            self.pos += 1;
            Ok(())
        } else {
            Err(())
        }
    }

    fn parse_value(&mut self) -> Result<Json, ()> {
        self.skip_space();
        match self.peek() {
            Some(b'{') => self.parse_object(),
            Some(b'[') => self.parse_array(),
            Some(b'"') => Ok(Json::Str(self.parse_string()?)),
            Some(b't') => self.parse_literal("true", Json::Bool(true)),
            Some(b'f') => self.parse_literal("false", Json::Bool(false)),
            Some(b'n') | Some(b'N') => {
                // `null`, or the non-standard `nan` the writer emits for
                // NaN min/max (jsmn reads it as a primitive either way).
                let save = self.pos;
                if self.parse_word("nan") {
                    let raw = self.bytes[save..self.pos].to_vec();
                    return Ok(Json::Num(String::from_utf8(raw).map_err(|_| ())?));
                }
                self.pos = save;
                self.parse_literal("null", Json::Null)
            }
            Some(b'-') | Some(b'+') | Some(b'0'..=b'9') | Some(b'i') | Some(b'I') => {
                // Optional sign + inf/infinity/nan (any case): the writer
                // emits these for non-finite min/max, and jsmn accepts any
                // bare word as a primitive. Anything else falls through to
                // the strict number grammar.
                let save = self.pos;
                if matches!(self.peek(), Some(b'-' | b'+')) {
                    self.pos += 1;
                }
                if self.parse_word("inf") || self.parse_word("infinity") || self.parse_word("nan") {
                    let raw = self.bytes[save..self.pos].to_vec();
                    // Infallible: ASCII sign + ASCII letters by construction.
                    return Ok(Json::Num(String::from_utf8(raw).map_err(|_| ())?));
                }
                self.pos = save;
                Ok(Json::Num(self.parse_number()?))
            }
            _ => Err(()),
        }
    }

    /// Match an ASCII word case-insensitively at the cursor; on success the
    /// cursor moves past it. The caller checks the following delimiter via
    /// the normal value-end rules (strict: a letter must not follow).
    fn parse_word(&mut self, word: &str) -> bool {
        let end = self.pos + word.len();
        if self.bytes.get(self.pos..end).is_none() {
            return false;
        }
        let matches = self.bytes[self.pos..end]
            .iter()
            .zip(word.bytes())
            .all(|(a, b)| a.to_ascii_lowercase() == b);
        if !matches {
            return false;
        }
        // A letter immediately after means a longer word (`info`, `nanx`).
        if let Some(next) = self.bytes.get(end)
            && next.is_ascii_alphabetic()
        {
            return false;
        }
        self.pos = end;
        true
    }

    fn parse_literal(&mut self, word: &str, value: Json) -> Result<Json, ()> {
        if self.bytes.get(self.pos..self.pos + word.len()) == Some(word.as_bytes()) {
            self.pos += word.len();
            Ok(value)
        } else {
            Err(())
        }
    }

    /// Strict JSON number; the raw text is kept for C-style conversion.
    fn parse_number(&mut self) -> Result<String, ()> {
        let start = self.pos;
        if self.peek() == Some(b'-') {
            self.pos += 1;
        }
        match self.peek() {
            Some(b'0') => {
                self.pos += 1;
            }
            Some(b'1'..=b'9') => {
                while matches!(self.peek(), Some(b) if is_json_digit(b)) {
                    self.pos += 1;
                }
            }
            _ => return Err(()),
        }
        if self.peek() == Some(b'.') {
            self.pos += 1;
            if !matches!(self.peek(), Some(b) if is_json_digit(b)) {
                return Err(());
            }
            while matches!(self.peek(), Some(b) if is_json_digit(b)) {
                self.pos += 1;
            }
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            self.pos += 1;
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.pos += 1;
            }
            if !matches!(self.peek(), Some(b) if is_json_digit(b)) {
                return Err(());
            }
            while matches!(self.peek(), Some(b) if is_json_digit(b)) {
                self.pos += 1;
            }
        }
        // Infallible: the token is ASCII digits/punctuation by construction.
        String::from_utf8(self.bytes[start..self.pos].to_vec()).map_err(|_| ())
    }

    /// Raw inner text with escape validation (no decoding: section keys
    /// compare raw, like cgltf's strcmp on the token bytes).
    fn parse_string(&mut self) -> Result<String, ()> {
        self.expect_byte(b'"')?;
        let start = self.pos;
        while let Some(b) = self.peek() {
            match b {
                b'"' => {
                    let raw = self.bytes[start..self.pos].to_vec();
                    self.pos += 1;
                    return String::from_utf8(raw).map_err(|_| ());
                }
                b'\\' => {
                    self.pos += 1;
                    match self.peek() {
                        Some(b'"') | Some(b'\\') | Some(b'/') | Some(b'b') | Some(b'f')
                        | Some(b'n') | Some(b'r') | Some(b't') => {
                            self.pos += 1;
                        }
                        Some(b'u') => {
                            self.pos += 1;
                            for _ in 0..4 {
                                match self.peek() {
                                    Some(b) if b.is_ascii_hexdigit() => self.pos += 1,
                                    _ => return Err(()),
                                }
                            }
                        }
                        _ => return Err(()),
                    }
                }
                0x00..=0x1F => return Err(()),
                _ => self.pos += 1,
            }
        }
        Err(())
    }

    fn parse_array(&mut self) -> Result<Json, ()> {
        self.depth += 1;
        if self.depth > MAX_JSON_DEPTH {
            return Err(());
        }
        self.expect_byte(b'[')?;
        let mut items = Vec::new();
        self.skip_space();
        if self.peek() == Some(b']') {
            self.pos += 1;
            self.depth -= 1;
            return Ok(Json::Array(items));
        }
        loop {
            items.push(self.parse_value()?);
            self.skip_space();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                }
                Some(b']') => {
                    self.pos += 1;
                    self.depth -= 1;
                    return Ok(Json::Array(items));
                }
                _ => return Err(()),
            }
        }
    }

    fn parse_object(&mut self) -> Result<Json, ()> {
        self.depth += 1;
        if self.depth > MAX_JSON_DEPTH {
            return Err(());
        }
        self.expect_byte(b'{')?;
        let mut members = Vec::new();
        self.skip_space();
        if self.peek() == Some(b'}') {
            self.pos += 1;
            self.depth -= 1;
            return Ok(Json::Object(members));
        }
        loop {
            self.skip_space();
            if self.peek() != Some(b'"') {
                return Err(());
            }
            let key = self.parse_string()?;
            self.skip_space();
            self.expect_byte(b':')?;
            let value = self.parse_value()?;
            members.push((key, value));
            self.skip_space();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                    continue;
                }
                Some(b'}') => {
                    self.pos += 1;
                    self.depth -= 1;
                    return Ok(Json::Object(members));
                }
                _ => return Err(()),
            }
        }
    }
}

fn parse_json(bytes: &[u8]) -> Result<Json, ()> {
    let mut parser = Parser::new(bytes);
    let value = parser.parse_value()?;
    parser.skip_space();
    if parser.pos != bytes.len() {
        return Err(());
    }
    if matches!(value, Json::Object(_)) {
        Ok(value)
    } else {
        Err(())
    }
}

/// C `atoll` over ASCII text (base 10, leading space skipped, empty or
/// digit-less prefix converts to 0, overflow clamps to `i64::MIN/MAX`).
fn c_atoll(text: &str) -> i64 {
    let bytes = text.as_bytes();
    let mut i = 0;
    while i < bytes.len() && matches!(bytes[i], b' ' | b'\t' | b'\n' | b'\x0b' | b'\x0c' | b'\r') {
        i += 1;
    }
    let mut neg = false;
    if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
        neg = bytes[i] == b'-';
        i += 1;
    }
    let mut mag: u64 = 0;
    let mut any = false;
    let mut overflow = false;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        any = true;
        let d = u64::from(bytes[i] - b'0');
        match mag.checked_mul(10).and_then(|v| v.checked_add(d)) {
            Some(v) => mag = v,
            None => overflow = true,
        }
        i += 1;
    }
    if !any {
        return 0;
    }
    let limit = i64::MAX as u64 + u64::from(neg);
    if overflow || mag > limit {
        return if neg { i64::MIN } else { i64::MAX };
    }
    let value = mag as i64;
    if neg { value.wrapping_neg() } else { value }
}

// ---------------------------------------------------------------------------
// glTF document subset (mirror of the cgltf structs + fixups the loader
// touches). Index fields follow the PTRFIXUP/PTRINDEX rules exactly:
// - optional indices (`indices`, accessor `bufferView`, node `mesh`,
//   top-level `scene`): missing -> None; -1 -> None (a -1+1 null pointer);
//   below -1 or >= count -> parse error;
// - required indices (attribute values, node `children`, view `buffer`,
//   sparse views, scene nodes): missing -> parse error; any negative or
//   >= count -> parse error.
// Non-primitive JSON values convert like `cgltf_json_to_int` failing its
// type check (-1) or `cgltf_json_to_size` failing its (0); the one
// exception is node `mesh`, whose parser checks the type explicitly.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ComponentType {
    I8,
    U8,
    I16,
    U16,
    U32,
    F32,
    Invalid,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum AccessorType {
    Scalar,
    Vec2,
    Vec3,
    Vec4,
    Mat2,
    Mat3,
    Mat4,
    Invalid,
}

fn component_type_from_int(v: i64) -> ComponentType {
    match v {
        5120 => ComponentType::I8,
        5121 => ComponentType::U8,
        5122 => ComponentType::I16,
        5123 => ComponentType::U16,
        5125 => ComponentType::U32,
        5126 => ComponentType::F32,
        _ => ComponentType::Invalid,
    }
}

fn accessor_type_from_str(raw: &str) -> AccessorType {
    // Raw comparison: cgltf strcmps the token bytes without decoding.
    match raw {
        "SCALAR" => AccessorType::Scalar,
        "VEC2" => AccessorType::Vec2,
        "VEC3" => AccessorType::Vec3,
        "VEC4" => AccessorType::Vec4,
        "MAT2" => AccessorType::Mat2,
        "MAT3" => AccessorType::Mat3,
        "MAT4" => AccessorType::Mat4,
        _ => AccessorType::Invalid,
    }
}

/// `cgltf_num_components` (invalid counts as 1, like scalar).
fn num_components(t: AccessorType) -> usize {
    match t {
        AccessorType::Vec2 => 2,
        AccessorType::Vec3 => 3,
        AccessorType::Vec4 | AccessorType::Mat2 => 4,
        AccessorType::Mat3 => 9,
        AccessorType::Mat4 => 16,
        AccessorType::Scalar | AccessorType::Invalid => 1,
    }
}

/// `cgltf_component_size` (invalid is 0 wide: reads convert nothing).
fn component_size(c: ComponentType) -> usize {
    match c {
        ComponentType::I8 | ComponentType::U8 => 1,
        ComponentType::I16 | ComponentType::U16 => 2,
        ComponentType::U32 | ComponentType::F32 => 4,
        ComponentType::Invalid => 0,
    }
}

/// `cgltf_calc_size` (packed stride incl. the matrix alignment cases).
fn calc_size(t: AccessorType, c: ComponentType) -> usize {
    let size = component_size(c);
    if t == AccessorType::Mat2 && size == 1 {
        return 8 * size;
    }
    if t == AccessorType::Mat3 && (size == 1 || size == 2) {
        return 12 * size;
    }
    size * num_components(t)
}

struct Accessor {
    buffer_view: Option<usize>,
    byte_offset: usize,
    component: ComponentType,
    count: usize,
    acc_type: AccessorType,
    normalized: bool,
    sparse: bool,
    stride: usize,
}

struct BufferView {
    buffer: usize,
    offset: usize,
    // Validation-only: parsed (cgltf would reject a malformed field) but
    // never used, since the C++ load path ignores view lengths as well.
    #[allow(dead_code)]
    length: usize,
    stride: usize, // 0 = tightly packed
}

struct Buffer {
    length: usize,
    has_uri: bool,
}

struct Primitive {
    position: Option<usize>,
    indices: Option<usize>,
    is_triangles: bool,
}

struct Mesh {
    primitives: Vec<Primitive>,
}

struct Node {
    mesh: Option<usize>,
    children: Vec<usize>,
    matrix: Option<[f32; 16]>,
    translation: [f32; 3],
    rotation: [f32; 4],
    scale: [f32; 3],
}

struct Doc {
    accessors: Vec<Accessor>,
    views: Vec<BufferView>,
    buffers: Vec<Buffer>,
    meshes: Vec<Mesh>,
    nodes: Vec<Node>,
    // Validation-only: scene refs can fail the C++ parse, but the loader
    // flattens every node regardless of scene membership.
    #[allow(dead_code)]
    scenes: Vec<Vec<usize>>,
    #[allow(dead_code)]
    scene: Option<usize>,
}

/// `cgltf_json_to_int` value semantics: primitives convert with atoi,
/// anything else yields -1 (the failed type check returned as a value).
fn prim_int(value: &Json) -> i64 {
    match value.prim_text() {
        Some(text) => c_atoll(text),
        None => -1,
    }
}

/// `cgltf_json_to_size` value semantics: primitives convert with atoll
/// clamped at 0, anything else yields 0.
fn prim_size(value: &Json) -> usize {
    match value.prim_text() {
        Some(text) => c_atoll(text).max(0) as usize,
        None => 0,
    }
}

/// `cgltf_json_to_float` value semantics (atof; bools/null convert to 0).
/// `None` means the token is not a primitive (the caller's explicit type
/// check failed), except float arrays check before calling.
fn prim_float(value: &Json) -> Option<f32> {
    match value {
        Json::Num(text) => text.parse::<f32>().ok(),
        Json::Bool(_) | Json::Null => Some(0.0),
        _ => None,
    }
}

/// `cgltf_json_to_bool`: true iff the token is exactly `true`.
fn prim_bool(value: &Json) -> bool {
    matches!(value, Json::Bool(true))
}

/// Optional index (CGLTF_PTRFIXUP): -1 is the null pointer.
fn opt_index(value: &Json, count: usize) -> Result<Option<usize>, ()> {
    let v = prim_int(value);
    if v == -1 {
        return Ok(None);
    }
    if v < -1 || v as u64 >= count as u64 {
        return Err(());
    }
    Ok(Some(v as usize))
}

/// Required index (CGLTF_PTRFIXUP_REQ).
fn req_index(value: &Json, count: usize) -> Result<usize, ()> {
    let v = prim_int(value);
    if v < 0 || v as u64 >= count as u64 {
        return Err(());
    }
    Ok(v as usize)
}

/// `cgltf_parse_json_float_array`: the value must be an array of exactly
/// `n` primitives.
fn float_array(value: &Json, out: &mut [f32]) -> Result<(), ()> {
    let items = value.as_array().ok_or(())?;
    if items.len() != out.len() {
        return Err(());
    }
    for (slot, item) in out.iter_mut().zip(items.iter()) {
        *slot = prim_float(item).ok_or(())?;
    }
    Ok(())
}

/// Attribute-name match (`POSITION` or `POSITION_<n>`, compared raw:
/// cgltf never decodes attribute names, so an escaped spelling matches
/// nothing; a negative index disqualifies the name entirely).
fn is_position_attribute(raw_name: &str) -> bool {
    let (head, tail) = match raw_name.split_once('_') {
        Some((head, tail)) => (head, Some(tail)),
        None => (raw_name, None),
    };
    if head != "POSITION" {
        return false;
    }
    match tail {
        Some(index) => c_atoll(index) >= 0,
        None => true,
    }
}

/// Validate every value of an attribute object as a required accessor
/// index, returning the FIRST POSITION one (cgltf keeps attributes as an
/// ordered list and the loader takes the first POSITION entry; every value
/// still validates, including custom and indexed names).
fn parse_attribute_list(
    value: &Json,
    accessor_count: usize,
    position: &mut Option<usize>,
) -> Result<(), ()> {
    let members = value.as_object().ok_or(())?;
    for (name, data) in members {
        let index = req_index(data, accessor_count)?;
        if position.is_none() && is_position_attribute(name) {
            *position = Some(index);
        }
    }
    Ok(())
}

fn parse_accessor(value: &Json, views_count: usize) -> Result<Accessor, ()> {
    let _ = value.as_object().ok_or(())?;
    let buffer_view = match value.member("bufferView") {
        Some(v) => opt_index(v, views_count)?,
        None => None,
    };
    let byte_offset = value.member("byteOffset").map_or(0, prim_size);
    let component = value
        .member("componentType")
        .map_or(ComponentType::Invalid, |v| {
            component_type_from_int(prim_int(v))
        });
    let count = value.member("count").map_or(0, prim_size);
    let acc_type = value
        .member("type")
        .and_then(Json::as_str)
        .map_or(AccessorType::Invalid, accessor_type_from_str);
    let normalized = value.member("normalized").is_some_and(prim_bool);
    // Sparse contents are validated like cgltf (both views required) but
    // never read: every sparse read fails, mirroring cgltf_accessor_read_*.
    let mut sparse = false;
    if let Some(s) = value.member("sparse") {
        let _ = s.as_object().ok_or(())?;
        let indices = s.member("indices").ok_or(())?;
        let _ = indices.as_object().ok_or(())?;
        req_index(indices.member("bufferView").ok_or(())?, views_count)?;
        let values = s.member("values").ok_or(())?;
        let _ = values.as_object().ok_or(())?;
        req_index(values.member("bufferView").ok_or(())?, views_count)?;
        sparse = true;
    }
    Ok(Accessor {
        buffer_view,
        byte_offset,
        component,
        count,
        acc_type,
        normalized,
        sparse,
        stride: 0, // fixed up below, after views exist
    })
}

fn parse_buffer_view(value: &Json, buffers_count: usize) -> Result<BufferView, ()> {
    let _ = value.as_object().ok_or(())?;
    // No explicit type check: a non-primitive yields -1, failing required.
    let buffer = req_index(value.member("buffer").ok_or(())?, buffers_count)?;
    Ok(BufferView {
        buffer,
        offset: value.member("byteOffset").map_or(0, prim_size),
        length: value.member("byteLength").map_or(0, prim_size),
        stride: value.member("byteStride").map_or(0, prim_size),
    })
}

fn parse_buffer(value: &Json) -> Result<Buffer, ()> {
    let _ = value.as_object().ok_or(())?;
    Ok(Buffer {
        length: value.member("byteLength").map_or(0, prim_size),
        has_uri: value.member("uri").is_some(),
    })
}

fn parse_primitive(value: &Json, accessor_count: usize) -> Result<Primitive, ()> {
    let _ = value.as_object().ok_or(())?;
    // Default triangles; unknown modes (incl. -1 from bad JSON) count as
    // non-triangles downstream, exactly like cgltf_primitive_type_invalid.
    let is_triangles = value.member("mode").is_none_or(|v| prim_int(v) == 4);
    let indices = match value.member("indices") {
        Some(v) => opt_index(v, accessor_count)?,
        None => None,
    };
    let mut position = None;
    if let Some(attributes) = value.member("attributes") {
        parse_attribute_list(attributes, accessor_count, &mut position)?;
    }
    // Morph targets carry attribute lists with the same required rule;
    // their values are never read by the loader.
    if let Some(targets) = value.member("targets") {
        let items = targets.as_array().ok_or(())?;
        let mut ignored = None;
        for target in items {
            parse_attribute_list(target, accessor_count, &mut ignored)?;
        }
    }
    // NOTE: `material` (and every other section: textures, skins,
    // animations, images, extensions) is deliberately unparsed; see the
    // module docs for the boundary.
    Ok(Primitive {
        position,
        indices,
        is_triangles,
    })
}

fn parse_mesh(value: &Json, accessor_count: usize) -> Result<Mesh, ()> {
    let _ = value.as_object().ok_or(())?;
    let mut primitives = Vec::new();
    if let Some(list) = value.member("primitives") {
        let items = list.as_array().ok_or(())?;
        for item in items {
            primitives.push(parse_primitive(item, accessor_count)?);
        }
    }
    Ok(Mesh { primitives })
}

fn parse_node(value: &Json, meshes_count: usize, nodes_count: usize) -> Result<Node, ()> {
    let _ = value.as_object().ok_or(())?;
    // cgltf checks this type explicitly: a non-primitive mesh is an error.
    let mesh = match value.member("mesh") {
        Some(v) => {
            if v.prim_text().is_none() {
                return Err(());
            }
            opt_index(v, meshes_count)?
        }
        None => None,
    };
    let mut children = Vec::new();
    if let Some(list) = value.member("children") {
        let items = list.as_array().ok_or(())?;
        for item in items {
            children.push(req_index(item, nodes_count)?);
        }
    }
    let mut node = Node {
        mesh,
        children,
        matrix: None,
        translation: [0.0, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
        scale: [1.0, 1.0, 1.0],
    };
    if let Some(m) = value.member("matrix") {
        let mut matrix = [0.0f32; 16];
        float_array(m, &mut matrix)?;
        node.matrix = Some(matrix);
    }
    if let Some(t) = value.member("translation") {
        float_array(t, &mut node.translation)?;
    }
    if let Some(r) = value.member("rotation") {
        float_array(r, &mut node.rotation)?;
    }
    if let Some(s) = value.member("scale") {
        float_array(s, &mut node.scale)?;
    }
    Ok(node)
}

fn parse_doc(root: &Json) -> Result<Doc, ()> {
    let _ = root.as_object().ok_or(())?;
    let get_array = |key: &str| -> Result<Vec<Json>, ()> {
        match root.member(key) {
            Some(v) => Ok(v.as_array().ok_or(())?.clone()),
            None => Ok(Vec::new()),
        }
    };
    // Lengths first: indices count against the full arrays.
    let accessor_values = get_array("accessors")?;
    let view_values = get_array("bufferViews")?;
    let buffer_values = get_array("buffers")?;
    let mesh_values = get_array("meshes")?;
    let node_values = get_array("nodes")?;
    let scene_values = get_array("scenes")?;
    let (accessor_count, view_count, buffer_count, mesh_count, node_count, scene_count) = (
        accessor_values.len(),
        view_values.len(),
        buffer_values.len(),
        mesh_values.len(),
        node_values.len(),
        scene_values.len(),
    );

    let mut accessors = Vec::with_capacity(accessor_count);
    for v in &accessor_values {
        accessors.push(parse_accessor(v, view_count)?);
    }
    let mut views = Vec::with_capacity(view_count);
    for v in &view_values {
        views.push(parse_buffer_view(v, buffer_count)?);
    }
    let mut buffers = Vec::with_capacity(buffer_count);
    for v in &buffer_values {
        buffers.push(parse_buffer(v)?);
    }
    let mut meshes = Vec::with_capacity(mesh_count);
    for v in &mesh_values {
        meshes.push(parse_mesh(v, accessor_count)?);
    }
    let mut nodes = Vec::with_capacity(node_count);
    for v in &node_values {
        nodes.push(parse_node(v, mesh_count, node_count)?);
    }

    // Stride fixup (cgltf_fixup_pointers): the view stride wins, else the
    // packed size.
    for accessor in &mut accessors {
        let packed = calc_size(accessor.acc_type, accessor.component);
        accessor.stride = accessor.buffer_view.map_or(packed, |i| views[i].stride);
        if accessor.stride == 0 {
            accessor.stride = packed;
        }
    }

    // Parent fixup: children are required indices and a node can only be
    // adopted once, exactly like the C++ pointer fixup.
    let mut parents: Vec<Option<usize>> = vec![None; node_count];
    for (i, node) in nodes.iter().enumerate() {
        for &child in &node.children {
            if parents[child].is_some() {
                return Err(());
            }
            parents[child] = Some(i);
        }
    }

    // Scene fixup: scene roots are required indices and must be roots.
    let mut scenes = Vec::with_capacity(scene_count);
    for v in &scene_values {
        let _ = v.as_object().ok_or(())?;
        let mut roots = Vec::new();
        if let Some(list) = v.member("nodes") {
            let items = list.as_array().ok_or(())?;
            for item in items {
                let n = req_index(item, node_count)?;
                if parents[n].is_some() {
                    return Err(());
                }
                roots.push(n);
            }
        }
        scenes.push(roots);
    }
    let scene = match root.member("scene") {
        Some(v) => opt_index(v, scene_count)?,
        None => None,
    };

    // Cycle guard: C++ spins `while (parent)` forever here; fail loudly.
    for start in 0..node_count {
        let mut slow = start;
        let mut fast = start;
        loop {
            slow = match parents[slow] {
                Some(p) => p,
                None => break,
            };
            fast = match parents[fast].and_then(|p| parents[p]) {
                Some(p) => p,
                None => break,
            };
            if slow == fast {
                return Err(());
            }
        }
    }

    Ok(Doc {
        accessors,
        views,
        buffers,
        meshes,
        nodes,
        scenes,
        scene,
    })
}

// ---------------------------------------------------------------------------
// Transforms (mirror cgltf_node_transform_local/world + applyMatrix in C++
// abstract-machine order; see the FP-fusion note in the module docs).
// ---------------------------------------------------------------------------

/// Column-major local matrix: explicit `matrix` wins, else TRS composition.
fn node_transform_local(node: &Node, out: &mut [f32; 16]) {
    if let Some(matrix) = node.matrix {
        *out = matrix;
        return;
    }
    let tx = node.translation[0];
    let ty = node.translation[1];
    let tz = node.translation[2];
    let qx = node.rotation[0];
    let qy = node.rotation[1];
    let qz = node.rotation[2];
    let qw = node.rotation[3];
    let sx = node.scale[0];
    let sy = node.scale[1];
    let sz = node.scale[2];

    out[0] = (1.0 - 2.0 * qy * qy - 2.0 * qz * qz) * sx;
    out[1] = (2.0 * qx * qy + 2.0 * qz * qw) * sx;
    out[2] = (2.0 * qx * qz - 2.0 * qy * qw) * sx;
    out[3] = 0.0;

    out[4] = (2.0 * qx * qy - 2.0 * qz * qw) * sy;
    out[5] = (1.0 - 2.0 * qx * qx - 2.0 * qz * qz) * sy;
    out[6] = (2.0 * qy * qz + 2.0 * qx * qw) * sy;
    out[7] = 0.0;

    out[8] = (2.0 * qx * qz + 2.0 * qy * qw) * sz;
    out[9] = (2.0 * qy * qz - 2.0 * qx * qw) * sz;
    out[10] = (1.0 - 2.0 * qx * qx - 2.0 * qy * qy) * sz;
    out[11] = 0.0;

    out[12] = tx;
    out[13] = ty;
    out[14] = tz;
    out[15] = 1.0;
}

/// Root-to-node composition over the parent chain (acyclic by validation).
fn node_transform_world(
    nodes: &[Node],
    parents: &[Option<usize>],
    node: usize,
    out: &mut [f32; 16],
) {
    node_transform_local(&nodes[node], out);
    let mut parent = parents[node];
    while let Some(p) = parent {
        let mut pm = [0.0f32; 16];
        node_transform_local(&nodes[p], &mut pm);
        for i in 0..4 {
            let l0 = out[i * 4];
            let l1 = out[i * 4 + 1];
            let l2 = out[i * 4 + 2];
            let r0 = l0 * pm[0] + l1 * pm[4] + l2 * pm[8];
            let r1 = l0 * pm[1] + l1 * pm[5] + l2 * pm[9];
            let r2 = l0 * pm[2] + l1 * pm[6] + l2 * pm[10];
            out[i * 4] = r0;
            out[i * 4 + 1] = r1;
            out[i * 4 + 2] = r2;
        }
        out[12] += pm[12];
        out[13] += pm[13];
        out[14] += pm[14];
        parent = parents[p];
    }
}

/// `out = M * [x y z 1]` for column-major M (mirror applyMatrix).
fn apply_matrix(m: &[f32; 16], v: &mut [f32; 3]) {
    let x = v[0];
    let y = v[1];
    let z = v[2];
    v[0] = m[0] * x + m[4] * y + m[8] * z + m[12];
    v[1] = m[1] * x + m[5] * y + m[9] * z + m[13];
    v[2] = m[2] * x + m[6] * y + m[10] * z + m[14];
}

// ---------------------------------------------------------------------------
// Accessor reads (mirror cgltf_accessor_read_float/uint + the element and
// component readers, including the no-view zeros and the float-index zeros).
// Bounds are checked per byte actually dereferenced; C++ never checks (UB),
// so overruns fail loudly here instead.
// ---------------------------------------------------------------------------

fn read_le(bytes: &[u8], at: usize, width: usize) -> Option<u64> {
    let slice = bytes.get(at..at.checked_add(width)?)?;
    let mut raw: u64 = 0;
    for (i, &b) in slice.iter().enumerate() {
        raw |= u64::from(b) << (8 * i);
    }
    Some(raw)
}

/// `cgltf_component_read_float` for one component.
fn component_to_float(
    data: &[u8],
    at: usize,
    component: ComponentType,
    normalized: bool,
) -> Option<f32> {
    if component == ComponentType::F32 {
        return Some(f32::from_bits(read_le(data, at, 4)? as u32));
    }
    if normalized {
        return match component {
            ComponentType::I16 => Some((read_le(data, at, 2)? as u16 as i16) as f32 / 32767.0),
            ComponentType::U16 => Some((read_le(data, at, 2)? as u16) as f32 / 65535.0),
            ComponentType::I8 => Some((read_le(data, at, 1)? as u8 as i8) as f32 / 127.0),
            ComponentType::U8 => Some((read_le(data, at, 1)? as u8) as f32 / 255.0),
            _ => Some(0.0),
        };
    }
    // `cgltf_component_read_integer`, converted (float/invalid read nothing).
    let value: i64 = match component {
        ComponentType::I16 => i64::from(read_le(data, at, 2)? as u16 as i16),
        ComponentType::U16 => i64::from(read_le(data, at, 2)? as u16),
        ComponentType::U32 => i64::from(read_le(data, at, 4)? as u32),
        ComponentType::I8 => i64::from(read_le(data, at, 1)? as u8 as i8),
        ComponentType::U8 => i64::from(read_le(data, at, 1)? as u8),
        _ => return Some(0.0),
    };
    Some(value as f32)
}

/// `cgltf_component_read_uint` for one component (signed kinds
/// sign-extend; float/invalid read 0 successfully).
fn component_to_uint(data: &[u8], at: usize, component: ComponentType) -> Option<u32> {
    match component {
        ComponentType::I8 => Some((read_le(data, at, 1)? as u8 as i8) as u32),
        ComponentType::U8 => Some(u32::from(read_le(data, at, 1)? as u8)),
        ComponentType::I16 => Some((read_le(data, at, 2)? as u16 as i16) as u32),
        ComponentType::U16 => Some(u32::from(read_le(data, at, 2)? as u16)),
        ComponentType::U32 => Some(read_le(data, at, 4)? as u32),
        _ => Some(0),
    }
}

struct Buffers<'a> {
    /// Bound data per buffer (`buffers[0]` is the BIN chunk when bound).
    data: Vec<Option<&'a [u8]>>,
}

/// Mirror `cgltf_accessor_read_float(accessor, i, v, 3)`; `v` starts zeroed
/// and short types leave the tail zeroed, exactly like the C++ caller.
fn accessor_read_float3(
    doc: &Doc,
    buffers: &Buffers<'_>,
    accessor: usize,
    index: usize,
    out: &mut [f32; 3],
) -> bool {
    let acc = &doc.accessors[accessor];
    if acc.sparse {
        return false;
    }
    let Some(view_index) = acc.buffer_view else {
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = 0.0;
        return true;
    };
    let view = &doc.views[view_index];
    let Some(data) = buffers.data[view.buffer] else {
        return false;
    };
    // Element-size check (VEC4+ POSITION fails); matrix packing needs no
    // special case: every matrix kind exceeds 3 components.
    if num_components(acc.acc_type) > 3 {
        return false;
    }
    let base = match view
        .offset
        .checked_add(acc.byte_offset)
        .and_then(|o| o.checked_add(acc.stride.checked_mul(index)?))
    {
        Some(base) => base,
        None => return false,
    };
    let width = component_size(acc.component);
    for k in 0..num_components(acc.acc_type) {
        let at = match width.checked_mul(k).and_then(|o| base.checked_add(o)) {
            Some(at) => at,
            None => return false,
        };
        match component_to_float(data, at, acc.component, acc.normalized) {
            Some(v) => out[k] = v,
            None => return false,
        }
    }
    true
}

/// Mirror `cgltf_accessor_read_uint(accessor, i, &idx, 1)`.
fn accessor_read_uint(
    doc: &Doc,
    buffers: &Buffers<'_>,
    accessor: usize,
    index: usize,
) -> Option<u32> {
    let acc = &doc.accessors[accessor];
    if acc.sparse {
        return None;
    }
    let Some(view_index) = acc.buffer_view else {
        return Some(0);
    };
    let view = &doc.views[view_index];
    let data = buffers.data[view.buffer]?;
    if matches!(
        acc.acc_type,
        AccessorType::Mat2 | AccessorType::Mat3 | AccessorType::Mat4
    ) {
        return None;
    }
    if num_components(acc.acc_type) > 1 {
        return None;
    }
    let at = view
        .offset
        .checked_add(acc.byte_offset)?
        .checked_add(acc.stride.checked_mul(index)?)?;
    component_to_uint(data, at, acc.component)
}

// ---------------------------------------------------------------------------
// GLB container (mirror cgltf_parse's container half + the raw-JSON
// fallback for files without the magic).
// ---------------------------------------------------------------------------

const GLB_MAGIC: [u8; 4] = *b"glTF";
const GLB_JSON_TYPE: u32 = 0x4E4F534A;
const GLB_BIN_TYPE: u32 = 0x004E4942;

fn read_u32_le(bytes: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_le_bytes(bytes.get(at..at + 4)?.try_into().ok()?))
}

struct Container<'a> {
    json: &'a [u8],
    bin: Option<&'a [u8]>,
}

fn parse_container(file: &[u8]) -> Result<Container<'_>, ()> {
    if file.len() < 12 {
        return Err(());
    }
    if file[0..4] != GLB_MAGIC {
        // cgltf falls back to parsing the whole file as glTF JSON.
        return Ok(Container {
            json: file,
            bin: None,
        });
    }
    let version = read_u32_le(file, 4).ok_or(())?;
    if version != 2 {
        return Err(());
    }
    let total = read_u32_le(file, 8).ok_or(())? as usize;
    if total > file.len() {
        return Err(());
    }
    if 12 + 8 > file.len() {
        return Err(());
    }
    let json_length = read_u32_le(file, 12).ok_or(())? as usize;
    if json_length > file.len() - 12 - 8 {
        return Err(());
    }
    if read_u32_le(file, 16).ok_or(())? != GLB_JSON_TYPE {
        return Err(());
    }
    let json_start = 20;
    let json_end = json_start + json_length;
    let rest = file.len() - json_end;
    let bin = if rest >= 8 {
        let bin_length = read_u32_le(file, json_end).ok_or(())? as usize;
        if bin_length > rest - 8 {
            return Err(());
        }
        if read_u32_le(file, json_end + 4).ok_or(())? != GLB_BIN_TYPE {
            return Err(());
        }
        Some(&file[json_end + 8..json_end + 8 + bin_length])
    } else {
        None
    };
    Ok(Container {
        json: &file[json_start..json_end],
        bin,
    })
}

// ---------------------------------------------------------------------------
// Loader (mirror loadGlbPositionsAndTriangles + appendMesh, same strings).
// ---------------------------------------------------------------------------

struct PrimitiveStats {
    skipped_non_triangles: usize,
    skipped_no_position: usize,
}

fn append_mesh(
    doc: &Doc,
    buffers: &Buffers<'_>,
    mesh: usize,
    world: &[f32; 16],
    positions: &mut Vec<f32>,
    triangles: &mut Vec<Vec<usize>>,
    stats: &mut PrimitiveStats,
) -> Result<(), String> {
    for prim in &doc.meshes[mesh].primitives {
        if !prim.is_triangles {
            stats.skipped_non_triangles += 1;
            continue;
        }
        let Some(pos_accessor) = prim.position else {
            stats.skipped_no_position += 1;
            continue;
        };
        let vert_count = doc.accessors[pos_accessor].count;
        if vert_count == 0 {
            stats.skipped_no_position += 1;
            continue;
        }
        let base = positions.len() / 3;
        positions.reserve(vert_count * 3);
        for i in 0..vert_count {
            let mut v = [0.0f32; 3];
            if !accessor_read_float3(doc, buffers, pos_accessor, i, &mut v) {
                return Err("failed to read POSITION accessor".to_string());
            }
            apply_matrix(world, &mut v);
            positions.push(v[0]);
            positions.push(v[1]);
            positions.push(v[2]);
        }
        let Some(index_accessor) = prim.indices else {
            // Non-indexed soup: every three vertices are one triangle.
            if !vert_count.is_multiple_of(3) {
                return Err("non-indexed primitive vertex count is not a multiple of 3".to_string());
            }
            for i in (0..vert_count).step_by(3) {
                triangles.push(vec![base + i, base + i + 1, base + i + 2]);
            }
            continue;
        };
        let index_count = doc.accessors[index_accessor].count;
        if !index_count.is_multiple_of(3) {
            return Err("index count is not a multiple of 3".to_string());
        }
        for i in (0..index_count).step_by(3) {
            let mut idx = [0u32; 3];
            for k in 0..3 {
                match accessor_read_uint(doc, buffers, index_accessor, i + k) {
                    Some(v) => idx[k] = v,
                    None => return Err("failed to read index accessor".to_string()),
                }
                if idx[k] as u64 >= vert_count as u64 {
                    return Err(format!("index out of range in {}-th index", i + k));
                }
            }
            triangles.push(vec![
                base + idx[0] as usize,
                base + idx[1] as usize,
                base + idx[2] as usize,
            ]);
        }
    }
    Ok(())
}

fn fail_load(err: &mut Option<&mut String>, message: String) -> bool {
    if let Some(e) = err {
        **e = message;
    }
    false
}

fn rebuild_parents(doc: &Doc) -> Vec<Option<usize>> {
    let mut parents: Vec<Option<usize>> = vec![None; doc.nodes.len()];
    for (i, node) in doc.nodes.iter().enumerate() {
        for &child in &node.children {
            parents[child] = Some(i);
        }
    }
    parents
}

/// Parse every mesh primitive in a `.glb` file into flat positions +
/// triangles, mirroring `loadObjPositionsAndTriangles` (positions as xyz
/// triples, one index vector per triangle; warn/err strings, false on
/// fatal errors). Node world transforms are applied; non-triangle
/// primitives are skipped with a warning; out-of-range indices fail.
pub fn load_glb_positions_and_triangles(
    filename: &Path,
    positions: &mut Vec<f32>,
    triangles: &mut Vec<Vec<usize>>,
    warn: Option<&mut String>,
    err: Option<&mut String>,
) -> bool {
    // Mirror C++: outputs are cleared up front (failures leave them empty,
    // not sentinel); warn/err are only assigned where C++ assigns them.
    positions.clear();
    triangles.clear();
    let mut err = err;

    let file = match std::fs::read(filename) {
        Ok(file) => file,
        Err(_) => {
            return fail_load(
                &mut err,
                format!("failed to parse GLB file {}", filename.display()),
            );
        }
    };
    let container = match parse_container(&file) {
        Ok(container) => container,
        Err(()) => {
            return fail_load(
                &mut err,
                format!("failed to parse GLB file {}", filename.display()),
            );
        }
    };
    let root = match parse_json(container.json) {
        Ok(root) => root,
        Err(()) => {
            return fail_load(
                &mut err,
                format!("failed to parse GLB file {}", filename.display()),
            );
        }
    };
    let doc = match parse_doc(&root) {
        Ok(doc) => doc,
        Err(()) => {
            return fail_load(
                &mut err,
                format!("failed to parse GLB file {}", filename.display()),
            );
        }
    };
    // Buffer binding (cgltf_load_buffers): buffer 0 takes the BIN chunk
    // when it fits; anything with a uri fails (external/data-URI bytes are
    // outside the subset); uri-less buffers without data stay unbound and
    // fail later at the read, like cgltf's null buffer data.
    let mut bound: Vec<Option<&[u8]>> = vec![None; doc.buffers.len()];
    if !doc.buffers.is_empty()
        && !doc.buffers[0].has_uri
        && let Some(bin) = container.bin
    {
        if bin.len() < doc.buffers[0].length {
            return fail_load(
                &mut err,
                format!("failed to load GLB buffers from {}", filename.display()),
            );
        }
        bound[0] = Some(bin);
    }
    for (i, buffer) in doc.buffers.iter().enumerate() {
        if bound[i].is_some() {
            continue;
        }
        if buffer.has_uri {
            return fail_load(
                &mut err,
                format!("failed to load GLB buffers from {}", filename.display()),
            );
        }
    }
    let buffers = Buffers { data: bound };

    if doc.meshes.is_empty() {
        return fail_load(
            &mut err,
            format!("GLB file has no meshes: {}", filename.display()),
        );
    }

    let parents = rebuild_parents(&doc);
    let mut stats = PrimitiveStats {
        skipped_non_triangles: 0,
        skipped_no_position: 0,
    };
    // Scene flattening: every node carrying a mesh contributes its mesh
    // once, transformed to world space (instanced meshes are duplicated,
    // which is what a remesher wants). Meshes no node references are still
    // loaded once, untransformed, so node-less exports are not silently
    // empty.
    let mut referenced = vec![false; doc.meshes.len()];
    for (n, node) in doc.nodes.iter().enumerate() {
        let Some(mesh) = node.mesh else {
            continue;
        };
        referenced[mesh] = true;
        let mut world = [0.0f32; 16];
        node_transform_world(&doc.nodes, &parents, n, &mut world);
        if let Err(message) = append_mesh(
            &doc, &buffers, mesh, &world, positions, triangles, &mut stats,
        ) {
            positions.clear();
            triangles.clear();
            return fail_load(&mut err, message);
        }
    }
    let identity: [f32; 16] = [
        1.0, 0.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, 0.0, //
        0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ];
    for mesh in 0..doc.meshes.len() {
        if referenced[mesh] {
            continue;
        }
        if let Err(message) = append_mesh(
            &doc, &buffers, mesh, &identity, positions, triangles, &mut stats,
        ) {
            positions.clear();
            triangles.clear();
            return fail_load(&mut err, message);
        }
    }
    if triangles.is_empty() {
        positions.clear();
        return fail_load(
            &mut err,
            format!("GLB file has no triangle geometry: {}", filename.display()),
        );
    }
    if (stats.skipped_non_triangles > 0 || stats.skipped_no_position > 0)
        && let Some(w) = warn
    {
        *w = format!(
            "skipped {} non-triangle and {} position-less primitives in {}",
            stats.skipped_non_triangles,
            stats.skipped_no_position,
            filename.display()
        );
    }
    true
}

// ---------------------------------------------------------------------------
// `%.6g` for f32 (C++ ostream default float formatting, used for the writer
// min/max). Exact decimal expansion (Rust's `{:.200}` is exact for f32:
// 149 fraction digits suffice and it prints the terminating expansion,
// verified against known expansions), rounded half-even to 6 significant
// digits, then fixed style for -4 <= X < 6 else scientific with a 2+ digit
// exponent. NaN prints signless `nan` (macOS libc++ behavior, pinned by the
// committed fixture); -0.0 prints `-0`.
// ---------------------------------------------------------------------------

fn format_g_f32(v: f32) -> String {
    if v.is_nan() {
        return "nan".to_string();
    }
    if v.is_infinite() {
        return if v > 0.0 {
            "inf".to_string()
        } else {
            "-inf".to_string()
        };
    }
    let neg = v.is_sign_negative();
    if v == 0.0 {
        return if neg {
            "-0".to_string()
        } else {
            "0".to_string()
        };
    }
    // Exact digits: strip the point, then leading zeros; `exp10` is the
    // decimal exponent of the first surviving digit.
    let exact = format!("{:.200}", v.abs());
    let mut digits: Vec<u8> = exact.bytes().filter(|&b| b != b'.').collect();
    let point = exact.find('.').unwrap_or(exact.len()) as i64;
    let mut first = 0;
    while first < digits.len() && digits[first] == b'0' {
        first += 1;
    }
    digits.drain(..first);
    while digits.last() == Some(&b'0') {
        digits.pop();
    }
    debug_assert!(!digits.is_empty());
    let mut exp10 = point - first as i64 - 1;

    // Round to 6 significant digits, half-even on the exact tail.
    const PREC: usize = 6;
    if digits.len() > PREC {
        let seventh = digits[PREC];
        let sticky = digits[PREC + 1..].iter().any(|&d| d != b'0');
        let odd = (digits[PREC - 1] - b'0') % 2 == 1;
        let round_up = seventh > b'5' || (seventh == b'5' && (sticky || odd));
        digits.truncate(PREC);
        if round_up {
            let mut i = PREC;
            while i > 0 {
                if digits[i - 1] == b'9' {
                    digits[i - 1] = b'0';
                    i -= 1;
                } else {
                    digits[i - 1] += 1;
                    break;
                }
            }
            if i == 0 {
                // 999999 + 1: renormalize to 100000 with the exponent up.
                digits[0] = b'1';
                exp10 += 1;
            }
        }
    }
    while digits.len() > 1 && digits.last() == Some(&b'0') {
        digits.pop();
    }

    let mut out = String::new();
    if neg {
        out.push('-');
    }
    if (-4..6).contains(&exp10) {
        // Fixed style.
        if exp10 >= 0 {
            let int_len = exp10 as usize + 1;
            for i in 0..int_len {
                out.push(digits.get(i).copied().unwrap_or(b'0') as char);
            }
            if digits.len() > int_len {
                out.push('.');
                for &d in &digits[int_len..] {
                    out.push(d as char);
                }
            }
        } else {
            out.push('0');
            out.push('.');
            for _ in 0..(-exp10 - 1) {
                out.push('0');
            }
            for &d in &digits {
                out.push(d as char);
            }
        }
    } else {
        // Scientific style with a 2+ digit exponent.
        out.push(digits[0] as char);
        if digits.len() > 1 {
            out.push('.');
            for &d in &digits[1..] {
                out.push(d as char);
            }
        }
        out.push('e');
        out.push(if exp10 < 0 { '-' } else { '+' });
        let mag = exp10.unsigned_abs().to_string();
        if mag.len() < 2 {
            out.push('0');
        }
        out.push_str(&mag);
    }
    out
}

// ---------------------------------------------------------------------------
// Writer (mirror writeGlb/saveGlb, byte-identical output).
// ---------------------------------------------------------------------------

fn push_u32_le(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn write_glb_bytes(
    generator: &str,
    vertices: &[Vector3],
    faces: &[Vec<usize>],
    uvs: Option<&[Vector2]>,
) -> Option<Vec<u8>> {
    // Fan-triangulate: quads become (0,1,2)+(0,2,3), matching the winding
    // the OBJ writer emits. Out-of-range indices fail loudly instead of
    // writing a corrupt file.
    let mut indices: Vec<u32> = Vec::new();
    for face in faces {
        if face.len() < 3 {
            continue;
        }
        for &index in face {
            if index >= vertices.len() {
                return None;
            }
        }
        for i in 1..face.len() - 1 {
            indices.push(face[0] as u32);
            indices.push(face[i] as u32);
            indices.push(face[i + 1] as u32);
        }
    }
    if vertices.is_empty() || indices.is_empty() {
        return None;
    }
    let have_uvs = uvs.is_some_and(|u| u.len() == vertices.len());

    // f32 min/max with f64 comparisons, exactly like the C++ loop (a NaN
    // coordinate never wins a comparison, but verts[0] seeds the range).
    let mut min_pos = [
        vertices[0].x() as f32,
        vertices[0].y() as f32,
        vertices[0].z() as f32,
    ];
    let mut max_pos = min_pos;
    for v in vertices {
        if v.x() < f64::from(min_pos[0]) {
            min_pos[0] = v.x() as f32;
        }
        if v.y() < f64::from(min_pos[1]) {
            min_pos[1] = v.y() as f32;
        }
        if v.z() < f64::from(min_pos[2]) {
            min_pos[2] = v.z() as f32;
        }
        if v.x() > f64::from(max_pos[0]) {
            max_pos[0] = v.x() as f32;
        }
        if v.y() > f64::from(max_pos[1]) {
            max_pos[1] = v.y() as f32;
        }
        if v.z() > f64::from(max_pos[2]) {
            max_pos[2] = v.z() as f32;
        }
    }

    let vert_bytes = vertices.len() * 3 * 4;
    let index_bytes = indices.len() * 4;
    let uv_bytes = if have_uvs { vertices.len() * 2 * 4 } else { 0 };
    // All sections are multiples of 4 by construction (3 floats, uint32,
    // 2 floats), so no inter-section padding is needed.
    let bin_length = vert_bytes + index_bytes + uv_bytes;

    // JSON assembly in the exact C++ ostringstream order (the generator is
    // interpolated raw, without escaping, like the C++).
    let mut json = String::new();
    json.push_str("{\"asset\":{\"version\":\"2.0\",\"generator\":\"");
    json.push_str(generator);
    json.push_str("\"}");
    json.push_str(",\"scene\":0");
    json.push_str(",\"scenes\":[{\"nodes\":[0]}]");
    json.push_str(",\"nodes\":[{\"mesh\":0,\"name\":\"retopoforge\"}]");
    json.push_str(",\"meshes\":[{\"name\":\"retopoforge\",\"primitives\":[");
    json.push_str("{\"attributes\":{\"POSITION\":0");
    if have_uvs {
        json.push_str(",\"TEXCOORD_0\":2");
    }
    json.push_str("},\"indices\":1,\"mode\":4}]}]");
    json.push_str(",\"accessors\":[");
    json.push_str("{\"bufferView\":0,\"componentType\":5126,\"count\":");
    json.push_str(&vertices.len().to_string());
    json.push_str(",\"type\":\"VEC3\",\"min\":[");
    json.push_str(&format_g_f32(min_pos[0]));
    json.push(',');
    json.push_str(&format_g_f32(min_pos[1]));
    json.push(',');
    json.push_str(&format_g_f32(min_pos[2]));
    json.push_str("],\"max\":[");
    json.push_str(&format_g_f32(max_pos[0]));
    json.push(',');
    json.push_str(&format_g_f32(max_pos[1]));
    json.push(',');
    json.push_str(&format_g_f32(max_pos[2]));
    json.push_str("]}");
    json.push_str(",{\"bufferView\":1,\"componentType\":5125,\"count\":");
    json.push_str(&indices.len().to_string());
    json.push_str(",\"type\":\"SCALAR\"}");
    if have_uvs {
        json.push_str(",{\"bufferView\":2,\"componentType\":5126,\"count\":");
        json.push_str(&vertices.len().to_string());
        json.push_str(",\"type\":\"VEC2\"}");
    }
    json.push(']');
    json.push_str(",\"bufferViews\":[");
    json.push_str("{\"buffer\":0,\"byteOffset\":0,\"byteLength\":");
    json.push_str(&vert_bytes.to_string());
    json.push('}');
    json.push_str(",{\"buffer\":0,\"byteOffset\":");
    json.push_str(&vert_bytes.to_string());
    json.push_str(",\"byteLength\":");
    json.push_str(&index_bytes.to_string());
    json.push('}');
    if have_uvs {
        json.push_str(",{\"buffer\":0,\"byteOffset\":");
        json.push_str(&(vert_bytes + index_bytes).to_string());
        json.push_str(",\"byteLength\":");
        json.push_str(&uv_bytes.to_string());
        json.push('}');
    }
    json.push(']');
    json.push_str(",\"buffers\":[{\"byteLength\":");
    json.push_str(&bin_length.to_string());
    json.push_str("}]}");
    while !json.len().is_multiple_of(4) {
        json.push(' ');
    }

    let mut blob: Vec<u8> = Vec::with_capacity(12 + 8 + json.len() + 8 + bin_length);
    // GLB header: magic 'glTF', version 2, total length.
    blob.extend_from_slice(b"glTF");
    push_u32_le(&mut blob, 2);
    push_u32_le(&mut blob, (12 + 8 + json.len() + 8 + bin_length) as u32);
    // JSON chunk (type 0x4E4F534A "JSON").
    push_u32_le(&mut blob, json.len() as u32);
    push_u32_le(&mut blob, 0x4E4F534A);
    blob.extend_from_slice(json.as_bytes());
    // BIN chunk (type 0x004E4942 "BIN\0"). UVs append after indices, so the
    // UV-less prefix (positions + indices) keeps identical bytes and offsets.
    push_u32_le(&mut blob, bin_length as u32);
    push_u32_le(&mut blob, 0x004E4942);
    for v in vertices {
        blob.extend_from_slice(&(v.x() as f32).to_le_bytes());
        blob.extend_from_slice(&(v.y() as f32).to_le_bytes());
        blob.extend_from_slice(&(v.z() as f32).to_le_bytes());
    }
    for &index in &indices {
        blob.extend_from_slice(&index.to_le_bytes());
    }
    if have_uvs {
        // `have_uvs` implies `uvs` is `Some` with the right length.
        for uv in uvs.unwrap_or(&[]) {
            blob.extend_from_slice(&(uv.x() as f32).to_le_bytes());
            blob.extend_from_slice(&(uv.y() as f32).to_le_bytes());
        }
    }
    Some(blob)
}

/// Write vertices + faces (quads, tris, any n-gon) as a single-mesh `.glb`
/// file. Faces are fan-triangulated; degenerate (<3 verts) faces are
/// skipped. Returns false when there is nothing to write or the write fails.
pub fn save_glb(
    filename: &Path,
    generator: &str,
    vertices: &[Vector3],
    faces: &[Vec<usize>],
) -> bool {
    match write_glb_bytes(generator, vertices, faces, None) {
        Some(blob) => std::fs::write(filename, &blob).is_ok(),
        None => false,
    }
}

/// `--uvs` on twin: same mesh plus a TEXCOORD_0 accessor (one VEC2 per
/// vertex; `uvs.len()` must equal `vertices.len()`). The 4-argument
/// [`save_glb`] is untouched so default output stays byte-identical.
pub fn save_glb_with_uvs(
    filename: &Path,
    generator: &str,
    vertices: &[Vector3],
    faces: &[Vec<usize>],
    uvs: &[Vector2],
) -> bool {
    if uvs.len() != vertices.len() {
        return false;
    }
    match write_glb_bytes(generator, vertices, faces, Some(uvs)) {
        Some(blob) => std::fs::write(filename, &blob).is_ok(),
        None => false,
    }
}
