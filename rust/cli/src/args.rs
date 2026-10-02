//! Command-line parsing: every flag, hand-rolled, std-only.
//!
//! Each `--long` flag also accepts the `--flag=value` form; `-i`, `-o`,
//! `-h`, `-v` are exact-match shorts. Parsing returns [`Action`] (run the
//! pipeline, or print help/version) or an [`ArgsError`] carrying the
//! exact historical error text plus whether usage follows it. All
//! messages and exit codes are pinned by the CLI contract goldens.
//!
//! Number syntax keeps its historical quirks (leading whitespace,
//! `+` signs, `inf`/`nan` accepted; empty string yields zero; integer
//! overflow clamps; `_` and hex floats rejected), also pinned by
//! goldens. Revisiting that acceptance set is CLI-UX follow-up work,
//! not this refactor.

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
        }
    }
}

/// What `main` should do after parsing.
pub(crate) enum Action {
    Help,
    Version,
    Run(Config),
}

/// A parse failure: the error plus whether usage is printed after it
/// (unknown options and missing `--input`/`--output` show usage; bad
/// values do not).
pub(crate) struct ArgsError {
    pub(crate) error: CliError,
    pub(crate) show_usage: bool,
}

impl ArgsError {
    fn value_error(error: CliError) -> Self {
        Self {
            error,
            show_usage: false,
        }
    }

    fn with_usage(text: impl Into<String>) -> Self {
        Self {
            error: CliError::message(text),
            show_usage: true,
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
/// argv element, else a "requires a value" error.
fn take_value<'a>(
    inline: Option<&'a str>,
    rest: &mut impl Iterator<Item = &'a String>,
    flag: &str,
) -> Result<String, CliError> {
    if let Some(value) = inline {
        return Ok(value.to_string());
    }
    match rest.next() {
        Some(value) => Ok(value.clone()),
        None => Err(CliError::message(format!("Error: {flag} requires a value"))),
    }
}

/// Historical double syntax: leading C-locale whitespace skipped, full
/// consumption required, empty string yields 0.0, `_` and hex floats
/// rejected.
fn parse_double(text: &str, flag: &str) -> Result<f64, CliError> {
    if text.is_empty() {
        return Ok(0.0);
    }
    let stripped = text.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    let ok = !stripped.is_empty()
        && !stripped.as_bytes().contains(&b'_')
        && !stripped.trim_start_matches(['+', '-']).starts_with("0x")
        && !stripped.trim_start_matches(['+', '-']).starts_with("0X");
    if !ok {
        return Err(CliError::message(format!(
            "Error: {flag} expects a number, got '{text}'"
        )));
    }
    stripped
        .parse::<f64>()
        .map_err(|_| CliError::message(format!("Error: {flag} expects a number, got '{text}'")))
}

/// Historical base-10 integer syntax, same shape as [`parse_double`]:
/// leading whitespace, optional sign, full consumption, empty yields 0.
/// Overflow clamps to `i64::MAX`/`i64::MIN`, then the negativity check
/// runs on the clamped value and the `i32` cast wraps.
fn parse_int(text: &str, flag: &str) -> Result<i32, CliError> {
    if text.is_empty() {
        return Ok(0);
    }
    let stripped = text.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    if stripped.is_empty() || stripped.as_bytes().contains(&b'_') {
        return Err(CliError::message(format!(
            "Error: {flag} expects a non-negative integer, got '{text}'"
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
                    return Err(CliError::message(format!(
                        "Error: {flag} expects a non-negative integer, got '{text}'"
                    )));
                }
            }
        }
    };
    if value < 0 {
        return Err(CliError::message(format!(
            "Error: {flag} expects a non-negative integer, got '{text}'"
        )));
    }
    Ok(value as i32)
}

/// `--lods`: comma-separated positive integers, `parse_int` per token.
fn parse_lods(text: &str, out: &mut Vec<i64>) -> Result<(), CliError> {
    out.clear();
    let mut start = 0usize;
    while start <= text.len() {
        let end = text[start..]
            .find(',')
            .map(|p| start + p)
            .unwrap_or(text.len());
        let token = text[start..end].trim_matches([' ', '\t']);
        let value = parse_int(token, "--lods")?;
        if value <= 0 {
            return Err(CliError::message(format!(
                "Error: --lods expects positive integers, got '{token}'"
            )));
        }
        out.push(value as i64);
        start = end + 1;
    }
    if out.is_empty() {
        return Err(CliError::message(format!(
            "Error: --lods expects a comma-separated list, got '{text}'"
        )));
    }
    Ok(())
}

fn parse_on_off(text: &str, flag: &str) -> Result<bool, CliError> {
    match text {
        "on" => Ok(true),
        "off" => Ok(false),
        _ => Err(CliError::message(format!(
            "Error: {flag} expects 'on' or 'off', got '{text}'"
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
        _ => Err(CliError::message(format!(
            "Error: --symmetry expects 'off', 'auto', 'x', 'y' or 'z', got '{text}'"
        ))),
    }
}

fn parse_model_type(text: &str) -> Result<ModelType, CliError> {
    match text {
        "organic" => Ok(ModelType::Organic),
        "hardsurface" | "hard-surface" | "hard_surface" => Ok(ModelType::HardSurface),
        _ => Err(CliError::message(format!(
            "Error: --model-type expects 'organic' or 'hardsurface', got '{text}'"
        ))),
    }
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
            "--help" | "-h" => return Ok(Action::Help),
            "--version" | "-v" => return Ok(Action::Version),
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
                        return Err(CliError::message(format!(
                            "Error: --dipoles expects 'off' or 'auto', got '{text}'"
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
                        return Err(CliError::message(format!(
                            "Error: --dipole-every expects a non-negative integer, got '{text}'"
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
                        return Err(CliError::message(format!(
                            "Error: --dipole-ratio expects a non-negative number, got '{text}'"
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
                return Err(ArgsError::with_usage(format!(
                    "Error: unknown option '{arg}'"
                )));
            }
        }
    }
    if config.input.as_os_str().is_empty() || config.output.as_os_str().is_empty() {
        return Err(ArgsError::with_usage(
            "Error: --input and --output are required",
        ));
    }
    Ok(Action::Run(config))
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
