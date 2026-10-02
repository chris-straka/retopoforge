//! Command-line parsing: every flag, hand-rolled, std-only.
//!
//! Each `--long` flag also accepts the `--flag=value` form; `-i`, `-o`,
//! `-h`, `-v` are exact-match shorts. Parsing returns [`Action`] (run the
//! pipeline, or print help/version) or an [`ArgsError`] carrying the
//! `retopo: error:`-convention text, its exit code (usage errors exit 2),
//! and whether the `--help` pointer follows it. All messages and exit
//! codes are pinned by the CLI contract goldens.
//!
//! Number syntax: leading whitespace, `+` signs, and integer-overflow
//! clamping stay (harmless, pinned); empty strings, `inf`/`nan`, `_`,
//! and hex floats are usage errors. Documented ranges clamp with a
//! one-line warning each (collected into [`Config::warnings`], emitted
//! by `main` before anything else runs).

use crate::error::CliError;
use retopo_core::auto_remesher::ModelType;
use retopo_core::quad_parameterizer::DipoleConfig;
use std::path::PathBuf;

/// Mirror-symmetry constraint mode (`--symmetry`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub(crate) enum Symmetry {
    #[default]
    Off,
    Auto,
    X,
    Y,
    Z,
}

impl Symmetry {
    pub(crate) fn enabled(self) -> bool {
        !matches!(self, Self::Off)
    }

    /// Engine plane selector: 0/1/2 pin an axis, -1 leaves it to the
    /// engine (auto-detect, or unused when disabled).
    pub(crate) fn plane(self) -> i32 {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
            Self::Off | Self::Auto => -1,
        }
    }
}

/// The fully parsed command line. Optional paths are `None` when their
/// flag was absent (previously empty-string sentinels).
pub(crate) struct Config {
    pub(crate) input: PathBuf,
    pub(crate) output: PathBuf,
    pub(crate) report: Option<PathBuf>,
    pub(crate) lod_targets: Vec<i64>,
    pub(crate) target_quads: i32,
    pub(crate) edge_scaling: f64,
    pub(crate) sharp_edge_degrees: f64,
    pub(crate) smooth_normal_degrees: f64,
    pub(crate) adaptivity: f64,
    pub(crate) anisotropy: f64,
    pub(crate) model_type: ModelType,
    pub(crate) symmetry: Symmetry,
    pub(crate) guides: Option<PathBuf>,
    pub(crate) features: Option<PathBuf>,
    pub(crate) density: Option<PathBuf>,
    pub(crate) dipoles: DipoleConfig,
    pub(crate) emit_uvs: bool,
    pub(crate) quiet: bool,
    /// Clamp warnings (`retopo: warning: …`, help order), emitted by
    /// `main` before the pipeline runs.
    pub(crate) warnings: Vec<String>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            input: PathBuf::new(),
            output: PathBuf::new(),
            report: None,
            lod_targets: Vec::new(),
            target_quads: 50000,
            edge_scaling: 1.0,
            sharp_edge_degrees: 90.0,
            smooth_normal_degrees: 0.0,
            adaptivity: 1.0,
            anisotropy: 1.0,
            model_type: ModelType::Organic,
            symmetry: Symmetry::Off,
            guides: None,
            features: None,
            density: None,
            dipoles: DipoleConfig::automatic(),
            emit_uvs: false,
            quiet: false,
            warnings: Vec::new(),
        }
    }
}

/// What `main` should do after parsing.
pub(crate) enum Action {
    Help,
    Version,
    Run(Config),
}

/// A parse failure: the usage error plus whether the `--help`
/// pointer line follows it (unknown options, positionals, and missing
/// flags/values point at `--help`; bad values name the flag, value,
/// and expectation already, so they don't).
pub(crate) struct ArgsError {
    pub(crate) error: CliError,
    pub(crate) show_pointer: bool,
}

impl ArgsError {
    fn value_error(error: CliError) -> Self {
        Self {
            error,
            show_pointer: false,
        }
    }

    fn with_pointer(text: impl Into<String>) -> Self {
        Self {
            error: CliError::usage(text),
            show_pointer: true,
        }
    }
}

impl From<CliError> for ArgsError {
    fn from(error: CliError) -> Self {
        Self::value_error(error)
    }
}

/// Split `--flag=value` into its name and inline value. Only `--long`
/// flags accept `=`; anything else (short flags, positionals, stray
/// text) is returned whole so it matches literally or falls into
/// "unknown option", exactly as before.
fn split_flag(arg: &str) -> (&str, Option<&str>) {
    if let Some((head, value)) = arg.split_once('=')
        && head.starts_with("--")
        && head.len() > 2
    {
        return (head, Some(value));
    }
    (arg, None)
}

/// A flag's value: the `--flag=value` tail when present, else the next
/// argv element, else a "requires a value" usage error (with pointer).
fn take_value<'a>(
    inline: Option<&'a str>,
    rest: &mut impl Iterator<Item = &'a String>,
    flag: &str,
) -> Result<String, ArgsError> {
    if let Some(value) = inline {
        return Ok(value.to_string());
    }
    match rest.next() {
        Some(value) => Ok(value.clone()),
        None => Err(ArgsError::with_pointer(format!(
            "retopo: error: {flag} requires a value."
        ))),
    }
}

/// Double syntax: leading C-locale whitespace skipped, full
/// consumption required; empty strings, `inf`/`nan`, `_`, and hex
/// floats are usage errors.
fn parse_double(text: &str, flag: &str) -> Result<f64, CliError> {
    let stripped = text.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    let ok = !stripped.is_empty()
        && !stripped.as_bytes().contains(&b'_')
        && !stripped.trim_start_matches(['+', '-']).starts_with("0x")
        && !stripped.trim_start_matches(['+', '-']).starts_with("0X");
    if !ok {
        return Err(CliError::usage(format!(
            "retopo: error: {flag} expects a number, got '{text}'."
        )));
    }
    match stripped.parse::<f64>() {
        Ok(value) if value.is_finite() => Ok(value),
        _ => Err(CliError::usage(format!(
            "retopo: error: {flag} expects a number, got '{text}'."
        ))),
    }
}

/// Base-10 integer syntax, same shape as [`parse_double`]: leading
/// whitespace, optional sign, full consumption; empty strings are
/// usage errors. Overflow clamps to `i64::MAX`/`i64::MIN`, then the
/// negativity check runs on the clamped value and the `i32` cast wraps.
fn parse_int(text: &str, flag: &str) -> Result<i32, CliError> {
    let stripped = text.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    if stripped.is_empty() || stripped.as_bytes().contains(&b'_') {
        return Err(CliError::usage(format!(
            "retopo: error: {flag} expects a non-negative integer, got '{text}'."
        )));
    }
    let value: i64 = match stripped.parse::<i64>() {
        Ok(value) => value,
        Err(err) => {
            use std::num::IntErrorKind::*;
            match err.kind() {
                PosOverflow => i64::MAX,
                NegOverflow => i64::MIN,
                _ => {
                    return Err(CliError::usage(format!(
                        "retopo: error: {flag} expects a non-negative integer, got '{text}'."
                    )));
                }
            }
        }
    };
    if value < 0 {
        return Err(CliError::usage(format!(
            "retopo: error: {flag} expects a non-negative integer, got '{text}'."
        )));
    }
    Ok(value as i32)
}

/// `--lods`: comma-separated positive integers, `parse_int` per token.
/// Empty rungs (leading/trailing/doubled commas) name the whole input
/// and hint the fix.
fn parse_lods(text: &str, out: &mut Vec<i64>) -> Result<(), CliError> {
    out.clear();
    let mut start = 0usize;
    while start <= text.len() {
        let end = text[start..]
            .find(',')
            .map(|p| start + p)
            .unwrap_or(text.len());
        let token = text[start..end].trim_matches([' ', '\t']);
        if token.is_empty() {
            return Err(CliError::usage(format!(
                "retopo: error: --lods expects positive integers, got '' in '{text}' (empty rung — drop the empty entry)."
            )));
        }
        let value = parse_int(token, "--lods")?;
        if value <= 0 {
            return Err(CliError::usage(format!(
                "retopo: error: --lods expects positive integers, got '{token}'."
            )));
        }
        out.push(value as i64);
        start = end + 1;
    }
    if out.is_empty() {
        return Err(CliError::usage(format!(
            "retopo: error: --lods expects a comma-separated list, got '{text}'."
        )));
    }
    Ok(())
}

fn parse_on_off(text: &str, flag: &str) -> Result<bool, CliError> {
    match text {
        "on" => Ok(true),
        "off" => Ok(false),
        _ => Err(CliError::usage(format!(
            "retopo: error: {flag} expects 'on' or 'off', got '{text}'."
        ))),
    }
}

fn parse_symmetry(text: &str) -> Result<Symmetry, CliError> {
    match text {
        "off" => Ok(Symmetry::Off),
        "auto" => Ok(Symmetry::Auto),
        "x" | "X" => Ok(Symmetry::X),
        "y" | "Y" => Ok(Symmetry::Y),
        "z" | "Z" => Ok(Symmetry::Z),
        _ => Err(CliError::usage(format!(
            "retopo: error: --symmetry expects 'off', 'auto', 'x', 'y' or 'z', got '{text}'."
        ))),
    }
}

fn parse_model_type(text: &str) -> Result<ModelType, CliError> {
    match text {
        "organic" => Ok(ModelType::Organic),
        "hardsurface" | "hard-surface" | "hard_surface" => Ok(ModelType::HardSurface),
        _ => Err(CliError::usage(format!(
            "retopo: error: --model-type expects 'organic' or 'hardsurface', got '{text}'."
        ))),
    }
}

/// Long-flag names (without `--`) for did-you-mean.
const LONG_FLAGS: &[&str] = &[
    "adaptivity",
    "anisotropy",
    "density",
    "dipole-every",
    "dipole-ratio",
    "dipoles",
    "edge-scaling",
    "features",
    "guides",
    "help",
    "input",
    "lods",
    "model-type",
    "output",
    "quiet",
    "report",
    "sharp-edge",
    "smooth-normal",
    "symmetry",
    "target-quads",
    "uvs",
    "version",
];

/// Edit distance (Levenshtein) over chars.
fn edit_distance(a: &str, b: &str) -> usize {
    let (a, b): (Vec<char>, Vec<char>) = (a.chars().collect(), b.chars().collect());
    let mut row: Vec<usize> = (0..=b.len()).collect();
    for i in 1..=a.len() {
        let mut prev = row[0];
        row[0] = i;
        for j in 1..=b.len() {
            let cost = usize::from(a[i - 1] != b[j - 1]);
            let next = (row[j] + 1).min(row[j - 1] + 1).min(prev + cost);
            prev = row[j];
            row[j] = next;
        }
    }
    row[b.len()]
}

/// Closest long flag within distance 2 (alphabetical tie-break), if any.
fn suggest_flag(name: &str) -> Option<&'static str> {
    LONG_FLAGS
        .iter()
        .copied()
        .map(|candidate| (edit_distance(name, candidate), candidate))
        .filter(|(distance, _)| *distance <= 2)
        .min()
        .map(|(_, candidate)| candidate)
}

/// Clamp documented ranges, collecting one warning per clamped flag
/// (help order). `--target-quads`/`--lods` and the dipole knobs reject
/// instead (parsed strictly), so they never reach this pass.
fn clamp_ranges(config: &mut Config) {
    let mut clamp = |flag: &str, range: &str, value: &mut f64, lo: f64, hi: f64| {
        if *value < lo || *value > hi {
            let clamped = value.clamp(lo, hi);
            config.warnings.push(format!(
                "retopo: warning: {flag} {} outside {range}; clamped to {}.",
                crate::format::g_format(*value),
                crate::format::g_format(clamped)
            ));
            *value = clamped;
        }
    };
    clamp(
        "--edge-scaling",
        "1.0-4.0",
        &mut config.edge_scaling,
        1.0,
        4.0,
    );
    clamp(
        "--sharp-edge",
        "30-180",
        &mut config.sharp_edge_degrees,
        30.0,
        180.0,
    );
    clamp(
        "--smooth-normal",
        "0-180",
        &mut config.smooth_normal_degrees,
        0.0,
        180.0,
    );
    clamp("--adaptivity", "0-1", &mut config.adaptivity, 0.0, 1.0);
    clamp("--anisotropy", "0-1", &mut config.anisotropy, 0.0, 1.0);
}

/// Parse argv (including argv[0]) into an [`Action`]. `--help` and
/// `--version` win immediately wherever they appear; otherwise
/// `--input` and `--output` are required.
pub(crate) fn parse_args(argv: &[String]) -> Result<Action, ArgsError> {
    let mut config = Config::default();
    let mut rest = argv.iter().skip(1);
    while let Some(arg) = rest.next() {
        let (name, inline) = split_flag(arg);
        match name {
            "--help" | "-h" => {
                if let Some(value) = inline {
                    return Err(CliError::usage(format!(
                        "retopo: error: --help takes no value (got '={value}')."
                    ))
                    .into());
                }
                return Ok(Action::Help);
            }
            "--version" | "-v" => {
                if let Some(value) = inline {
                    return Err(CliError::usage(format!(
                        "retopo: error: --version takes no value (got '={value}')."
                    ))
                    .into());
                }
                return Ok(Action::Version);
            }
            "--input" | "-i" => {
                config.input = take_value(inline, &mut rest, "--input")?.into();
            }
            "--output" | "-o" => {
                config.output = take_value(inline, &mut rest, "--output")?.into();
            }
            "--report" => {
                config.report = Some(take_value(inline, &mut rest, "--report")?.into());
            }
            "--target-quads" => {
                let text = take_value(inline, &mut rest, "--target-quads")?;
                config.target_quads = parse_int(&text, "--target-quads")?;
            }
            "--lods" => {
                let text = take_value(inline, &mut rest, "--lods")?;
                parse_lods(&text, &mut config.lod_targets)?;
            }
            "--edge-scaling" => {
                let text = take_value(inline, &mut rest, "--edge-scaling")?;
                config.edge_scaling = parse_double(&text, "--edge-scaling")?;
            }
            "--sharp-edge" => {
                let text = take_value(inline, &mut rest, "--sharp-edge")?;
                config.sharp_edge_degrees = parse_double(&text, "--sharp-edge")?;
            }
            "--smooth-normal" => {
                let text = take_value(inline, &mut rest, "--smooth-normal")?;
                config.smooth_normal_degrees = parse_double(&text, "--smooth-normal")?;
            }
            "--adaptivity" => {
                let text = take_value(inline, &mut rest, "--adaptivity")?;
                config.adaptivity = parse_double(&text, "--adaptivity")?;
            }
            "--anisotropy" => {
                let text = take_value(inline, &mut rest, "--anisotropy")?;
                config.anisotropy = parse_double(&text, "--anisotropy")?;
            }
            "--uvs" => {
                let text = take_value(inline, &mut rest, "--uvs")?;
                config.emit_uvs = parse_on_off(&text, "--uvs")?;
            }
            "--quiet" => {
                if let Some(value) = inline {
                    return Err(CliError::usage(format!(
                        "retopo: error: --quiet takes no value (got '={value}')."
                    ))
                    .into());
                }
                config.quiet = true;
            }
            "--symmetry" => {
                let text = take_value(inline, &mut rest, "--symmetry")?;
                config.symmetry = parse_symmetry(&text)?;
            }
            "--density" => {
                config.density = Some(take_value(inline, &mut rest, "--density")?.into());
            }
            "--dipoles" => {
                let text = take_value(inline, &mut rest, "--dipoles")?;
                match text.as_str() {
                    "off" => config.dipoles.enabled = false,
                    "auto" => config.dipoles.enabled = true,
                    _ => {
                        return Err(CliError::usage(format!(
                            "retopo: error: --dipoles expects 'off' or 'auto', got '{text}'."
                        ))
                        .into());
                    }
                }
            }
            "--dipole-every" => {
                let text = take_value(inline, &mut rest, "--dipole-every")?;
                match parse_int(&text, "--dipole-every")? {
                    v if v >= 0 => config.dipoles.every = v as usize,
                    _ => {
                        return Err(CliError::usage(format!(
                            "retopo: error: --dipole-every expects a non-negative integer, got '{text}'."
                        ))
                        .into());
                    }
                }
            }
            "--dipole-ratio" => {
                let text = take_value(inline, &mut rest, "--dipole-ratio")?;
                match parse_double(&text, "--dipole-ratio")? {
                    v if v >= 0.0 => config.dipoles.ratio = v,
                    _ => {
                        return Err(CliError::usage(format!(
                            "retopo: error: --dipole-ratio expects a non-negative number, got '{text}'."
                        ))
                        .into());
                    }
                }
            }
            "--guides" => {
                config.guides = Some(take_value(inline, &mut rest, "--guides")?.into());
            }
            "--features" => {
                config.features = Some(take_value(inline, &mut rest, "--features")?.into());
            }
            "--model-type" => {
                let text = take_value(inline, &mut rest, "--model-type")?;
                config.model_type = parse_model_type(&text)?;
            }
            _ => {
                if !arg.starts_with('-') {
                    return Err(ArgsError::with_pointer(format!(
                        "retopo: error: unexpected positional argument '{arg}' (this CLI takes flags only — try --input {arg} --output …)."
                    )));
                }
                let mut text = format!("retopo: error: unknown option '{arg}'");
                if let Some(name) = arg.strip_prefix("--").filter(|n| !n.is_empty())
                    && let Some(suggestion) = suggest_flag(name)
                {
                    text.push_str(&format!(" (did you mean --{suggestion}?)"));
                }
                text.push('.');
                return Err(ArgsError::with_pointer(text));
            }
        }
    }
    let missing_input = config.input.as_os_str().is_empty();
    let missing_output = config.output.as_os_str().is_empty();
    if missing_input || missing_output {
        let text = match (missing_input, missing_output) {
            (true, true) => "retopo: error: missing required options --input and --output.",
            (true, false) => "retopo: error: missing required option --input.",
            (false, true) => "retopo: error: missing required option --output.",
            (false, false) => unreachable!("guarded by the outer condition"),
        };
        return Err(ArgsError::with_pointer(text));
    }
    clamp_ranges(&mut config);
    Ok(Action::Run(config))
}

#[cfg(test)]
mod arg_tests {
    use super::*;

    #[test]
    fn did_you_mean_suggests_within_two() {
        assert_eq!(suggest_flag("inputx"), Some("input"));
        assert_eq!(suggest_flag("symetry"), Some("symmetry"));
        assert_eq!(suggest_flag("uv"), Some("uvs"));
        assert_eq!(suggest_flag("bogus"), None);
        assert_eq!(suggest_flag(""), None);
    }

    #[test]
    fn clamp_warns_once_per_flag_in_help_order() {
        let mut config = Config {
            adaptivity: 7.0,
            edge_scaling: 99.0,
            ..Config::default()
        };
        clamp_ranges(&mut config);
        assert_eq!(config.edge_scaling, 4.0);
        assert_eq!(config.adaptivity, 1.0);
        assert_eq!(config.warnings.len(), 2);
        assert!(
            config.warnings[0].contains("--edge-scaling"),
            "help order: {}",
            config.warnings[0]
        );
        assert!(
            config.warnings[1].contains("--adaptivity"),
            "help order: {}",
            config.warnings[1]
        );
        // In-range values pass silently.
        let mut clean = Config::default();
        clamp_ranges(&mut clean);
        assert!(clean.warnings.is_empty());
    }
}

pub(crate) fn print_usage(argv0: &str) {
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
            "  --dipoles <off|auto>         Density-boundary dipole insertion:\n",
            "                              singularity rings along sharp --density\n",
            "                              steps unlock localized refinement\n",
            "                              (default: auto; fires on asks\n",
            "                              above ~2.5x, mild masks unaffected)\n",
            "  --dipole-every <count>       Dipole dose stride override: place\n",
            "                              every k-th ring candidate (default: 0\n",
            "                              = auto line-ending estimate)\n",
            "  --dipole-ratio <value>       Dipole step-sharpness override:\n",
            "                              minimum face-key ratio (default: 0 =\n",
            "                              auto 1.5)\n",
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
