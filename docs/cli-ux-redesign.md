# CLI UX redesign (spec — needs owner approval before ANY implementation)

Status: PROPOSAL. Spec only; no product code or test expectations change
under this document. Implementation happens only after the owner approves
the proposal (whole or per-phase).

Context: the Rust CLI (`rust/cli/src/main.rs`) mirrors the 19 upstream
C++ flags byte-for-byte (help text, exit codes, stdout/stderr story) as
port scaffolding. `docs/rust-switch-verdict.md` lists CLI UX redesign as
post-switch superiority work. The C++ tree is frozen as the oracle
reference. This spec proposes the post-scaffolding CLI surface.

Audit basis: every proposal below is grounded in behavior observed on
`lane/cli-ux-spec` (base `26375cd4`) with a debug Rust build, quoted
verbatim. Flag count today: 19 upstream flags + 3 Rust-only
(`--dipoles`, `--dipole-every`, `--dipole-ratio`) = 22.

## 1. Current-state audit (observed, quoted)

### 1.1 Help and version

`--help` (exit 0) prints a 78-line flat flag list (see
`rust/cli/src/main.rs:96-180` for the exact text); the usage line echoes
argv0:

```
Usage: rust/target/debug/retopo --input <file.obj|file.glb|dir> --output <output.obj|output.glb|dir> [options]
```

`--version` prints `retopoforge 0.1.0` — while Homebrew packaging is
0.3.0 (pre-existing wart, also noted in the switch verdict).

Observations:

- Flat structure: 22 flags in one `Options:` block, no grouping. Expert
  overrides (`--dipole-every`, `--dipole-ratio`) sit beside everyday
  flags with internal jargon ("Dose stride override", "minimum face-key
  ratio", "auto line-ending estimate").
- `--density` help cites a stale saturation number ("~2.3x realized for
  4x asks") that predates dipole production (see
  `docs/dipole-production.md`; 4x faceAbs now realizes up to ~2.04x via
  dipoles, and the help never mentions the `--dipoles` interplay).
- `--uvs` help leaks implementation history: "Off keeps every output
  byte identical to before".
- `--model-type` help shows `organic|hardsurface`, but the parser also
  accepts `hard-surface` and `hard_surface` silently.
  `--symmetry` help shows `x|y|z`, but `X|Y|Z` are silently accepted.
- No examples, no exit-code documentation, no mention of the report
  formats, no `--help <topic>`, no shell completion.

### 1.2 Streams: what goes where

| Content | Stream | Example (verbatim) |
|---|---|---|
| Progress lines (~70/run) | stdout | `0% done. Computing voxel size` … `100% done. Done` |
| Single-mode report block | stdout | `=== retopoforge Report ===` … `Quads: 218` … |
| LOD rung lines | stdout | `LOD 0: target-quads=200 output=… quads=218 non-quads=0 vertices=220 time=0.765389 seconds` |
| Batch per-file lines | stdout | `FILE good.obj: target-quads=200 …` |
| Batch failure lines | stdout | `FILE bad.obj: FAILED remeshing produced no result` |
| Batch summary | stdout | `Failed files: none` / `Failed files (1): bad.obj` |
| Usage-on-error dump | stdout | full 78-line help after `Error: unknown option '--bogus'` (stderr) |
| Load/progress info | stderr | `Loaded 882 vertices, 1760 triangles`, `Guide polylines: 2` |
| Engine phase dump | stderr | `Extract connections...`, `Hold singular lines walked 1 of 1 starved cone(s), added 1 connection(s)` |
| Phase timings (~45 lines) | stderr | `  Rounding cover to integers: 2.3 ms` … `  Total: 715.5 ms` |
| Warnings | stderr | `Warning: 1 of 1 islands produced no output and were dropped from the mesh` |
| Loader warnings | stderr | `WARN: …` (distinct prefix) |
| Loader hard errors | stderr | unprefixed `Cannot open file [/tmp/does-not-exist.obj]`, then `Error: failed to load …` |
| Errors | stderr | `Error: …` |

Core finding: **stdout mixes machine-readable results with ~70 lines of
human progress chatter**, so `retopo … | parse` and
`retopo … > results.txt` both capture progress noise. Downstream
parsers work around it today (`bench/run.py:parse_report_stdout`
scans for `Quads:`; `blender/retopoforge/__init__.py:_LOD_RUNG_RE`
regexes `LOD … quads=`; `bench/profile.py` scrapes indented stderr
timings). Conversely, **usage-on-error goes to stdout**, polluting the
results stream precisely when there are no results.

### 1.3 Errors: prefixes, exit codes, strictness

All failures exit **1** (usage errors, missing files, remesh failures,
partial batch). Partial island drops exit **0** (deliberate, keeps
pipelines alive). Observed error lines:

```
Error: --input and --output are required        (+ full usage to stdout)
Error: unknown option '--bogus'                 (+ full usage to stdout)
Error: unknown option '--inputx'                (+ full usage; no did-you-mean)
Error: --input requires a value
Error: --uvs expects 'on' or 'off', got 'maybe'
Error: --symmetry expects 'off', 'auto', 'x', 'y' or 'z', got 'diag'
Error: --target-quads expects a non-negative integer, got 'xyz'
Error: --lods expects positive integers, got ''          (trailing-comma input `100,`)
Error: --dipoles expects 'off' or 'auto', got 'sometimes'
Error: --guides needs a single input mesh, not a batch directory
Error: no .obj/.glb files in /tmp/uxaudit/empty
Error: cannot open --guides file '/tmp/nope.txt'
Error: --guides file '/tmp/…/badguides.txt' line 1 expects 'x y z', got '0 0'
Error: --density file holds 2 multipliers, input has 882 vertices
Error: failed to write /tmp/uxaudit/nodir2/x.obj
```

Strictness gaps (all observed, exit 0 unless noted):

- Documented ranges are **not enforced**: `--edge-scaling 99`,
  `--sharp-edge 5` (doc range 30–180), `--adaptivity 7` (doc range
  0–1) all run silently to `EXIT=0`.
- `--quiet=x` is silently accepted (the `=x` is ignored; run proceeds).
  `--help=x` prints help. `--inputx` is "unknown option", not a typo
  hint.
- C-compat quirks preserved: `--target-quads ""` → 0,
  `--edge-scaling inf|nan` accepted silently, leading whitespace and
  `+5` accepted, `1_0` rejected. (Intentional port scaffolding; see
  `main.rs:15-22`.)
- Duplicate flags last-win silently; `--quiet --quiet` fine.
- Grammar bug: `Warning: 1 of 1 islands produced no output and were
  dropped from the mesh` (plural verb, singular count).
- Prefix salad: `Error:` vs `Warning:` vs `WARN:` vs unprefixed loader
  lines; load failures double-report (`Cannot open file […]` blank
  line, then `Error: failed to load …`).
- Failure attribution bug (tetra target-8 collapse): a run whose only
  island yields nothing prints 70 progress lines, then
  `Warning: 1 of 1 islands … dropped` and `Error: failed to write
  /tmp/uxaudit/tetra.obj` — the write never happened because there was
  no mesh; the user is told the disk failed.

### 1.4 Ordering bugs (fail-fast violations)

Single-file mode loads the mesh **before** validating constraint
files, so a typo'd `--guides` path still prints
`Loaded 882 vertices, 1760 triangles` before
`Error: cannot open --guides file '/tmp/nope.txt'`. Multi/batch mode
validates first. Same split for `--report`: multi-mode creates the
report up front (fail-fast), but single-mode remeshes to completion,
prints the stdout report, and only then reports
`Error: failed to write /tmp/uxaudit/nodir/r.txt` with exit 1 —
wasted minutes on big inputs for a predictable failure.

### 1.5 Batch mode and `--lods`

- Mode selection is implicit: `--input` pointing at a directory means
  batch. Observed footgun: `-i <dir> -o somefile.obj` (a nonexistent,
  file-looking path) **silently creates a directory named
  `somefile.obj/`** and writes outputs inside it (confirmed via
  `ls`/`file`). Only an *existing file* at `--output` errors
  (`--output must be a directory when --input is a directory`).
- Non-mesh files in the input dir are skipped silently (`notes.txt`
  produced no notice). Uppercase extensions work (`UP.OBJ` remeshed —
  keep). Inputs are sorted; symlinks to files are followed.
- `--guides`/`--features`/`--density` are rejected in batch (good
  errors, quoted above).
- Batch + `--report` **omits failed files entirely**: the report for a
  1-good/1-bad batch contained only the good entry — no trace of
  `bad.obj`. And multi-mode report entries lack the `Islands:` /
  `Failed islands:` lines the single-mode report has.
- `--lods` always rewrites the output name: even a single rung
  (`--lods 200 -o s1.obj`) writes `s1_lod0.obj`, ignoring the literal
  `--output` name. LOD+batch names as `<stem>_lod<rung><ext>`.
- Partial batch: exit 1 with `Failed files (1): bad.obj` on stdout;
  each failure is also duplicated on stderr
  (`Error: remeshing produced no result (/path)`).

### 1.6 Progress and phase output (loud mode)

A 0.7 s finger run emits ~70 stdout progress lines
(`3% done. Building bounding volume tree` — note the repeated
percents and stuttered statuses like `Splitting long edges` at 5/6%),
plus ~50 stderr lines: engine bare dump + a full indented phase table
down to 0.0 ms leaves, ending with
`Cores kept busy across the parallel phase: 1.00 (islands are the unit
of parallelism)`. For a sub-second run the diagnostics outweigh the
result 10:1. `--quiet` silences all of it (stdout = report only,
stderr empty) — the one clean contract in the CLI today.
