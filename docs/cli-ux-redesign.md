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

## 2. Goals and non-goals

Goals:

1. **Pipes work**: stdout carries only results; humans read stderr.
2. **Errors teach**: every failure names the flag/file, shows the bad
   value, and hints the fix. No 78-line dumps, no silent acceptance.
3. **Fail fast**: validate everything validatable before the first
   remesh millisecond (constraint files, report path, output
   writability, ranges).
4. **Help scans**: grouped flags, examples, exit codes; expert knobs
   tucked away but discoverable.
5. **Downstream survives**: Blender add-on, `bench/*`, and the e2e
   harness migrate mechanically; parsing gets easier, never harder.

Non-goals: subcommands (`retopo remesh …` — the tool does one thing);
a config file; JSON stdout in phase 1 (proposed as phase 2);
engine/output-mesh changes of any kind; touching the frozen C++.

## 3. Flag surface: renames, grouping, removals

### 3.1 Verdict: rename nothing heavily-used, group everything

Every widely-used flag (`--input`, `--output`, `--target-quads`,
`--lods`, `--report`, `--quiet`, `--guides`, `--features`,
`--density`, `--symmetry`, `--uvs`, `--model-type`, `--adaptivity`,
`--anisotropy`, `--edge-scaling`, `--sharp-edge`, `--smooth-normal`)
is referenced by the Blender add-on, `bench/*`, docs, or user habit.
Renaming them buys elegance at the price of breaking every caller for
no behavior gain. **Proposal: keep all 19 upstream names; change
presentation (grouping, help text), strictness, and a few value
shapes.** The only renames are additive aliases:

| Change | Rationale |
|---|---|
| Add `-q` short for `--quiet` | convention (`-i/-o/-h/-v` exist; `-q` is free and expected) |
| Accept bare `--uvs` (= `on`); keep `--uvs on\|off` | boolean disguised as enum today; bare form is the principle-of-least-surprise spelling, fully backward compatible |
| Keep `--dipole-every` / `--dipole-ratio` names, move to an `Expert:` help group (still accepted, still in `--help --all`) | brand-new flags (Oct 2026); renaming now churns the freshest docs for zero gain. Grouping fixes the discoverability problem without breaking anything |

Removals: **none**. Every flag is either user-facing or (dipole
overrides) expert-legitimate. Deprecations in phase 1: none — the
strictness changes in §5 are error-message/exit-code changes, not
flag removals.

### 3.2 Proposed help groups

```
Usage: retopo --input <file|dir> --output <file|dir> [options]

Input / output:
  -i, --input …      -o, --output …      --report …      --lods …
Quality target:
  --target-quads …   --edge-scaling …    --adaptivity …  --anisotropy …
  --model-type …     --sharp-edge …      --smooth-normal …
Constraints:
  --guides …         --features …        --density …     --dipoles …
  --symmetry …
Output control:
  --uvs …            --quiet, -q         --verbose …
Expert (see --help --all for full detail):
  --dipole-every …   --dipole-ratio …
Help:
  -h, --help [--all] -v, --version
```

New in this layout: `--verbose` (gates the ~45-line phase table and
engine bare dump; see §6) and `--help --all` (appends the Expert
group with full text; default help shows one line per expert flag).
`--quiet` keeps its exact current semantics (report/rung lines only
on stdout, warnings+errors on stderr).

### 3.3 Batch mode: keep implicit, kill the footgun

The implicit dir-detection is documented, tested, and depended on —
but creating a *directory* out of a file-looking `--output` is never
what the user meant. Proposal:

- If `--input` is a dir and `--output` does not exist and its final
  path component has a mesh extension (`.obj`/`.glb`, any case):
  **error, do not mkdir**. E.g.
  `error: --input is a directory, so --output must be a directory:
  'somefile.obj' looks like a file (hint: drop the extension or point
  --output at an existing directory)`.
- Extensionless nonexistent `--output` keeps the current
  create-as-directory behavior (documented in help).
- Existing-file `--output` keeps the current error.
- Skip notice: when batch ignores non-mesh files, print one stderr
  summary (`note: skipped 3 non-mesh files in <dir>`) instead of
  silence. (Lists names only under `--verbose`.)
- No new `--batch` flag in phase 1 — implicit detection plus the
  guard above closes the hole without a compat break. (Owner call:
  an explicit `--batch` requirement could come in phase 2.)

## 4. Proposed help text (full rewrite)

```
Usage: retopo --input <file|dir> --output <file|dir> [options]

Remesh a triangle mesh (.obj/.glb) into clean quad-dominant output.
Point --input at a directory to remesh every mesh in it (batch mode);
--output is then a directory (created if missing).

Input / output:
  -i, --input <file|dir>
      Input mesh (required). A directory remeshes every .obj/.glb in it
      (non-recursive, sorted); symlinks to meshes are followed.
  -o, --output <file|dir>
      Output path (required). .obj writes OBJ, .glb writes GLB (quads
      triangulated). A directory in batch mode (see --input).
  --report <file>
      Also write the stats report to a file. Fails fast if unwritable.
  --lods <q0,q1,...>
      Emit one LOD rung per target, e.g. --lods 10000,5000,2000 writes
      <stem>_lod0.<ext>, <stem>_lod1.<ext>, … next to --output and
      overrides --target-quads. Each rung gets its own result line.

Quality target:
  --target-quads <n>        Target quad count (default 50000; non-negative
                            integer; ignored when --lods is given).
  --edge-scaling <f>        Edge scaling factor (default 1.0, clamped to
                            1.0-4.0; out-of-range values warn).
  --adaptivity <f>          Curvature-adaptive density, 0-1 (default 1.0).
  --anisotropy <f>          Curvature-adaptive elongation, 0-1 (default 1.0).
  --model-type <organic|hardsurface>
                            Model hint (default organic).
  --sharp-edge <deg>        Sharp dihedral threshold, 30-180 (default 90).
  --smooth-normal <deg>     Smooth-normal threshold, 0-180 (default 0).

Constraints (single-file and --lods runs only; rejected in batch mode):
  --guides <file>           Quad-flow polylines (eye/mouth loops): one
                            'x y z' point per line, blank lines separate
                            polylines, '#' starts a comment. Points are in
                            input-mesh coordinates.
  --features <file>         Crisp-edge polylines for hard-surface props.
                            Same file format as --guides.
  --density <file>          Per-vertex density multipliers (OBJ v-line
                            order), 1.0 = unchanged, clamped to 0.25-4.0.
                            '#' starts a comment. Needs --dipoles (default)
                            to realize sharp (>~2.5x) asks.
  --dipoles <off|auto>      Singularity rings along sharp density steps
                            (default auto). Off restores pre-dipole
                            saturation on strong masks.
  --symmetry <off|auto|x|y|z>
                            Mirror-symmetry constraints (default off). auto
                            detects the dominant plane; falls back to
                            unconstrained output below threshold.

Output control:
  --uvs [on|off]            Emit remeshed UVs (OBJ vt + v/vt corners, GLB
                            TEXCOORD_0). Bare --uvs means on. Default off.
  -q, --quiet               Results only: silence progress/info/diagnostics.
                            Warnings, errors, and the report still print.
  --verbose                 Loud diagnostics: full per-phase timing table
                            and engine progress detail on stderr.

Expert (defaults are right for everyone else; full text in --help --all):
  --dipole-every <n>        Place every n-th dipole-ring candidate
                            (default 0 = automatic).
  --dipole-ratio <f>        Minimum density-step sharpness for dipole
                            insertion (default 0 = automatic 1.5).

  -h, --help [--all]        Show this help (--all adds expert detail).
  -v, --version             Show version.

Examples:
  retopo -i hero.obj -o hero_low.obj --target-quads 5000
  retopo -i hero.obj -o hero_lods.obj --lods 10000,5000,2000
  retopo -i assets/ -o remeshed/ --target-quads 5000 --quiet
  retopo -i face.obj -o face.obj --density face_mask.txt --guides flow.txt

Exit codes: 0 success (including partial island drops, which warn);
1 remesh/load/write failure or partial batch; 2 usage error (bad
flags/values, missing files). Errors go to stderr with the failing
flag/value; `retopo --help` never fails.
```

Notes on the rewrite: ranges are stated as enforced behavior (see
§5.3); the stale `~2.3x` claim is replaced with the dipole interplay
pointer; `--uvs` documents the bare form; expert jargon
("face-key", "line-ending", "dose") is replaced with plain effect
language; exit codes and examples are new.

## 5. Error-message conventions (before → after)

### 5.1 The convention

- Shape: `retopo: error: <what failed>: <detail>. <hint>` on stderr.
  One line per failure; no usage dump (see below). The `retopo:`
  prefix disambiguates in pipelines and Blender logs.
- Usage errors (bad flags/values, missing paths) exit **2**; runtime
  failures (load/remesh/write, partial batch) exit **1**; success
  (including warned-about island drops) exits **0**. Today everything
  is 1 — the 2/1 split is the standard `grep`/`diff` convention and
  lets scripts distinguish "you mistyped" from "the mesh failed".
- No full usage on error. Instead, a 2-line pointer:
  `Run 'retopo --help' for usage.` Usage goes to stdout only when
  asked for (`--help`); error context goes to stderr only.
- Unknown flags get a did-you-mean (edit distance ≤ 2 over long-flag
  names): `--inputx` → `did you mean --input?`.
- Strict values: `--flag=x` on a boolean flag is an error, not
  silent acceptance. Empty-string numbers, `inf`/`nan`, and
  out-of-range values warn-or-error (see §5.3), never pass silently.
- Warnings unify on lowercase `warning:`; notes on `note:`.
  `WARN:` and unprefixed loader lines are retired. Each failure is
  reported once (no `Cannot open…` + `failed to load` double).
- Fail fast: parse all flags → validate constraint files → check
  input readability → check output/report writability → then remesh.
  Single mode matches multi mode (fixes §1.4).

### 5.2 Before/after table (every "before" quoted from §1)

| # | Before (observed) | After (proposed) |
|---|---|---|
| 1 | `Error: unknown option '--bogus'` + 78-line usage on stdout, exit 1 | `retopo: error: unknown option '--bogus'. Run 'retopo --help' for usage.` on stderr, exit 2 |
| 2 | `Error: unknown option '--inputx'` + usage, exit 1 | `retopo: error: unknown option '--inputx' (did you mean --input?). Run 'retopo --help' for usage.` exit 2 |
| 3 | `Error: --input and --output are required` + usage on stdout, exit 1 | `retopo: error: missing required options --input and --output. Run 'retopo --help' for usage.` exit 2 (names whichever is actually missing) |
| 4 | `Error: --uvs expects 'on' or 'off', got 'maybe'` exit 1 | `retopo: error: --uvs expects 'on' or 'off', got 'maybe'.` exit 2 |
| 5 | `Error: --lods expects positive integers, got ''` (for `100,`) exit 1 | `retopo: error: --lods expects positive integers, got '' in '100,' (empty rung — drop the trailing comma).` exit 2 |
| 6 | `--quiet=x` silently accepted, run proceeds | `retopo: error: --quiet takes no value (got '=x').` exit 2 |
| 7 | `--edge-scaling 99` / `--adaptivity 7` run silently, exit 0 | `retopo: warning: --edge-scaling 99 outside 1.0-4.0; clamped to 4.0.` then run (see §5.3) |
| 8 | `Cannot open file [x]` (blank line) + `Error: failed to load x` exit 1 | `retopo: error: cannot open input 'x': no such file or directory.` exit 1 (single line, OS reason included) |
| 9 | `Loaded 882 vertices…` then `Error: cannot open --guides file '/tmp/nope.txt'` exit 1 | `retopo: error: cannot open --guides file '/tmp/nope.txt': no such file or directory.` exit 2, printed before any load line |
| 10 | `Error: --density file holds 2 multipliers, input has 882 vertices` | `retopo: error: --density file holds 2 multipliers but the input has 882 vertices (need exactly one per input vertex).` exit 2 for unreadable/malformed files, exit 1 for count mismatch (needs the mesh to judge — still checked before remeshing) |
| 11 | `Warning: 1 of 1 islands produced no output and were dropped from the mesh` | `retopo: warning: 1 of 1 islands produced no output and was dropped from the mesh.` (pluralize correctly) |
| 12 | Tetra collapse: `Warning: 1 of 1 islands…` + `Error: failed to write /tmp/uxaudit/tetra.obj` exit 1 | `retopo: error: remeshing produced an empty mesh (all 1 island(s) dropped); nothing written to '/tmp/uxaudit/tetra.obj'. Try a larger --target-quads.` exit 1 (names the real cause; no false disk-blame) |
| 13 | `Error: failed to write /tmp/uxaudit/nodir2/x.obj` after a full remesh | same text, but checked up front: `retopo: error: cannot write output '/tmp/…/x.obj': no such directory.` exit 1 before remeshing |
| 14 | Single-mode `--report` to bad path: full remesh, stdout report, then `Error: failed to write …` exit 1 | `retopo: error: cannot write --report file '…': ….` exit 1 before remeshing (multi-mode behavior everywhere) |
| 15 | Batch+report silently omits failed files | every input gets a report entry; failures record `Failed: <reason>` (see §6.3) |
| 16 | `-i dir -o somefile.obj` creates directory `somefile.obj/` | `retopo: error: --input is a directory, so --output must be a directory: 'somefile.obj' looks like a file.` exit 2 (see §3.3) |
| 17 | `WARN: …` loader warnings | `retopo: warning: …` (unified prefix) |

### 5.3 Range handling: clamp + warn (not reject)

Today ranges are documented but unenforced (§1.3). Hard-rejecting
would break callers that pass through computed floats (the Blender
add-on passes `repr(float(...))` verbatim). Proposal: **clamp to the
documented range and warn once per flag** (stderr, `warning:`), except
`--target-quads`/`--lods`, which already reject ≤ 0 / negatives and
keep doing so. `inf`/`nan`/empty numerics become usage errors (exit
2) — they are never meaningful, and the C-compat acceptance was port
scaffolding, explicitly called out as divergent-by-mechanism in the
switch verdict. Underscore rejection, `+5`, and leading-whitespace
acceptance stay (harmless, tested).

### 5.4 Positional args

Today `retopo foo.obj bar.obj` says `unknown option 'foo.obj'`.
Proposal: `retopo: error: unexpected positional argument 'foo.obj'
(this CLI takes flags only — try --input foo.obj --output …).`
exit 2. (No positional support added — flags-only stays.)

## 6. Progress and report output cleanup

### 6.1 The stream contract

- **stdout = results only, always parseable**: the single-mode report
  block, `LOD n:` / `FILE …:` rung lines, `Failed files:` summary.
  Nothing else, ever — no progress, no usage-on-error, no failure
  chatter beyond the result lines themselves.
- **stderr = everything human**: progress, info (`Loaded …`),
  `warning:`, `note:`, `error:`, diagnostics.
- `--quiet`: unchanged semantics (results on stdout; warnings+errors
  on stderr; nothing else). It becomes "the default, minus stderr
  info" rather than a special case.
- `--verbose`: the current loud stderr (phase table, engine bare
  dump) moves behind this flag; default loud keeps progress + info
  + one-line phase summary (see §6.2). Rationale: a 0.7 s run does
  not need 50 diagnostic lines by default, and `bench/profile.py`
  (the only consumer of the phase table) can pass `--verbose`.

### 6.2 Progress lines: stderr, throttled, same vocabulary

- Move `N% done. <status>` from stdout to stderr, one line per
  update (no `\r` tricks — logs stay greppable), throttled to
  at most one line per percent-point per status (kills the
  `3% … / 3% …` stutter observed in §1.6).
- Keep the `N% done. <status>` text itself (the e2e progress oracle
  and user habit already know it); only the stream changes.
- Default loud keeps progress + `Loaded …` + ack lines
  (`Guide polylines: 2`) + a new one-line close:
  `done: 218 quads, 0 non-quads, 220 vertices in 0.73 s` (stderr).
  The full phase table + engine bare dump (`Extract connections...`
  etc.) move to `--verbose`. The `Cores kept busy…` line moves with
  them.
- Timestamps: none (keeps output diffable; `--verbose` table keeps
  its ms column).

### 6.3 Report formats: keep shapes, fix gaps

Single-mode stdout block, LOD/FILE rung lines, and the
`Failed files:` summary keep their exact current text — three
independent parsers depend on them (`bench/run.py`,
`blender/__init__.py` `_SUMMARY_RE`/`_LOD_RUNG_RE`,
`tests/test_cli_multimode.cpp` via the frozen C++ side). Changes:

1. Multi-mode `--report` entries gain the missing `Islands:` /
   `Failed islands:` lines (parity with single-mode reports).
2. Failed inputs get report entries (`Input file: …` /
   `Failed: <reason>`) instead of being omitted (§1.5).
3. `Time:`/`Total time:` keep `%.6g seconds`.
4. Phase 2 (not this spec's implementation): `--report-format json`
   emitting one JSON object per entry, so new consumers stop
   regexing. The text formats above stay byte-stable regardless.

### 6.4 What gets quieter: a size comparison (finger, target-200)

| Run | Today | Proposed default | Proposed --verbose |
|---|---|---|---|
| stdout | ~70 progress + 11-line report | 11-line report only | 11-line report only |
| stderr | ~55 lines (info + engine dump + phase table) | ~4 lines (`Loaded`, ack, one-line close; warnings as needed) | today's ~55 lines |
| `--quiet` stdout/stderr | report / empty | unchanged | n/a (`--quiet --verbose` = error: mutually exclusive) |

## 7. Backward-compat and migration plan

### 7.1 Compat summary

- Flag names: all 22 keep working; additions are purely additive
  (`-q`, bare `--uvs`, `--verbose`, `--help --all`). No caller
  changes required for flag spelling.
- Result-line text (report block, `LOD n:`/`FILE …:` rungs,
  `Failed files:`): byte-stable. Regex parsers keep working.
- Breaking by design: error text + `retopo: ` prefix, exit 2 for
  usage errors, progress on stderr, usage-on-error retired,
  `--verbose` gating of the phase table, `--quiet=x`-style
  leniency removed, `inf`/`nan`/empty numerics rejected, ranges
  clamped-with-warning, batch file-looking `--output` refused.
- `--version` string: out of scope for the string itself
  (`RETOPO_VERSION` 0.1.0 vs packaging 0.3.0 is a separate wart,
  flagged — recommend syncing to the package version in the same
  release, owner call).

### 7.2 Which tests change and how

`rust/cli/tests/e2e_diff.rs` (runs both binaries; the CLI-surface
comparisons become Rust-golden instead of differential):

- `arg_matrix` (~50 cases: `help-long`, `help-short`, `help-eq`,
  `help-midparse`, `unknown`, `unknown-after-valid`, `missing-value`,
  `inputx-typo`, `uvs-*`, `symmetry-*`, `model-type-*`,
  `target-quads-*`, `edge-scaling-*`, `adaptivity-*`, `lods-*`,
  `eq-*`, `quiet-*`, …): rewrite expectations to the §4/§5 text and
  exit 2/1 split. The C++ side can no longer match by design — these
  cases stop invoking the C++ binary and assert Rust goldens
  (stdout empty on usage error, stderr exact, exit code exact).
  Delete the dipole-help tier shim (~line 2175) — help is no longer
  compared across binaries at all.
- Remesh cases (37): stdout strict-compare simplifies (progress gone
  from stdout; report/rung lines stay exact). Stderr audit table
  (`audit_stderr`, ~line 261): C++-only line classes stay
  allowlisted; add Rust-new classes (`retopo: …`, `done: …`,
  `note: …`) as expected-Rust lines. Progress oracle
  (`audit_progress_robust`, ~line 553): repoint at stderr.
- IO-failure paths (20): update to single-line errors + exit codes.
- Mesh-byte comparisons: untouched (still differential vs C++ —
  see §8).

`tests/test_cli_*.cpp` (C++ CTest suite): **no changes**. They run
the frozen C++ `retopo` binary (`cli/CMakeLists.txt:
add_executable(retopo …)` via `RETOPO_BINARY`), which this redesign
does not touch. They remain the oracle-side regression net.

Rust unit tests: `island_accounting_tests` in `main.rs` unchanged
(counting logic untouched); add unit tests for did-you-mean,
clamp-and-warn, and the batch output guard.

Other suites to touch:

- `blender/tests/test_headless.py`: asserts `--symmetry` values and
  `--guides`/`--features`/`--density` passthrough — unchanged (flag
  names stable). Add coverage only if the add-on adopts `-q`/bare
  `--uvs` (optional).
- `bench/run.py::parse_report_stdout`: unchanged (report block
  byte-stable). `bench/compare.py`, `bench/profile.py`: profile.py
  must pass `--verbose` (it scrapes the indented phase table that
  moves behind the flag); compare.py unchanged (uses `--target-quads`
  + exit codes 0-vs-nonzero — note: it must treat exit 2 as failure,
  same as 1; verify, don't assume).
- CI (`ci/`, `development.yml`): any step asserting CLI stderr text
  or exit codes needs the new strings; mesh-count gates unaffected.

### 7.3 Docs and downstream callers

- `blender/retopoforge/__init__.py`: `cli_args` works unchanged.
  Optional follow-up: pass `-q` instead of nothing for quiet runs
  (no — the add-on parses loud stdout today via `_SUMMARY_RE` /
  `_LOD_RUNG_RE`, which still match; leave it). Update the LODRUNG
  comment only if rung text changes (it doesn't).
- `blender/README.md`, `README.md`, `docs/face-flow-iterate.md`,
  `docs/lod-strategy.md`, `docs/dipole-production.md`: refresh quoted
  help/error text and the stale `~2.3x` figure where repeated.
- `docs/rust-switch-verdict.md`: append a post-redesign note (CLI
  surface intentionally diverged; oracle scope narrowed to §8).

## 8. What stays frozen for C++ comparability

The C++ binary remains the oracle for everything the redesign does
not own:

- **Output mesh bytes**: OBJ/GLB writers, vertex/face ordering,
  `%.6g` number formatting, `# retopoforge` header comments —
  byte-identical, still differentially tested on every `cargo test`.
- **Remesh success/failure per input**: which inputs produce meshes
  (modulo the accepted EPX/any-mode tiers) — the engine contract.
- **Result counts**: quads/non-quads/vertices in report/rung lines —
  still cross-checked against C++ runs (same values, new streams).
- **Constraint-file acceptance semantics**: guides/features/density
  file formats and their mesh effects — only diagnostics change.
- **C++ CLI tests** (`tests/test_cli_*.cpp`) and the C++ binary
  itself: frozen, still built and run in CI.

Narrowed on purpose: help text, error strings, exit-code granularity
(2 vs 1 — C++ stays all-1), progress/diagnostic streams and volume,
and the C-compat numeric quirks (§5.3). The e2e harness keeps a
narrow "C++ still exits nonzero here" check for the failure classes
so the oracle still guards against Rust accidentally succeeding
where C++ fails (and vice versa).

## 9. Rollout and owner decisions

Proposed implementation phasing (after approval):

- Phase A (safe, no downstream edits): §4 help rewrite + grouping,
  `-q`/bare-`--uvs`/`--verbose`/`--help --all`, error-text +
  exit-2 convention, did-you-mean, strict `--flag=x`, fail-fast
  ordering, grammar/attribution fixes, batch output guard + skip
  notice, report-gap fixes (§6.3 items 1–2).
- Phase B (stream move + downstream): progress → stderr + throttle,
  phase table behind `--verbose`, usage-on-error retired; migrate
  `e2e_diff.rs`, `bench/profile.py`, CI assertions in the same
  commit stack.
- Phase 2 (separate spec): `--report-format json`, explicit
  `--batch`, shell completion, `RETOPO_VERSION` sync.

Owner calls needed:

1. Approve the exit-2 split (vs keeping everything exit 1)?
2. Approve clamp+warn for ranges (vs hard reject vs leave silent)?
3. Approve `--verbose` gating the phase table (vs keeping loud the
   default)?
4. `--version` sync to packaging version in the same release?
5. Phase 2 scope: is `--report-format json` wanted at all?
