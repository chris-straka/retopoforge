# Plan: freehand flow guides in the Blender add-on

Status: planned 2026-10-06, not started.

## Why

Flow guides decide where the remesh puts its edge loops (shoulder rings,
groin, eye and mouth loops). The owner calls them critical. Guides already
work end to end: the add-on's Flow Guides panel turns an Edit Mode edge
selection into a `--guides` file, `gen character guides <asset> FILE`
attaches it to a genforge character, and the repair-topology adapter passes
it to `retopo` (genforge D64-D65, retopoforge 80d3dba).

The gap is drawing them. genforge remeshes from the generated model, often
1M+ triangles (Andras's Tripo mesh is 1.5M). Picking edges on a mesh that
dense is slow, and its edges rarely run where the flow should go. Getting a
genforge character into Blender also needs a manual import with "Guess
Original Bind Pose" off, or the mesh shifts about 1 m and the guides miss.

Goal: open a genforge character, draw guides freehand on the surface, and
send them back, in a few minutes, on any mesh size.

## G1: freehand strokes

- **Draw:** a "Draw Guides" button in the Flow Guides panel switches to
  Blender's Annotate tool with placement set to Surface, on a dedicated
  annotation layer ("RetopoForge Guides", distinct colour). It works in
  Object Mode, so a 1.5M-triangle mesh never enters Edit Mode. The native
  annotation eraser and Ctrl+Z work as usual.
- **Convert:** the exporter reads that layer's strokes and, per stroke:
  1. resamples at a spacing tied to the target edge length (from the
     target quad count and the surface area, not fixed units);
  2. projects each point onto the target mesh (BVH nearest point in the
     object's local space; points farther than a few target edge lengths
     are dropped);
  3. splits the stroke where points were dropped, and drops pieces that
     end up too short;
  4. simplifies the result, keeping deviation under a fraction of an edge
     length.
- **Mirror:** an option, on by default for humanoid and quadruped classes,
  that mirrors strokes across the object's symmetry plane. The plane comes
  from `retopo --symmetry auto` detection, falling back to X. You draw one
  side.
- **Merge:** freehand strokes and an edge selection may both be present;
  the exported file contains both.
- **Preview:** a toggle draws the exact polylines that will be exported
  (GPU overlay), so what you see is what gets sent.
- **Output:** the same `--guides` format as today (local coordinates, one
  `x y z` per line, blank line between polylines), so `retopo`, genforge
  and the adapter are unchanged.
- Check the annotation API on both Blender 4.2 LTS (the manifest minimum)
  and 5.2 before building on it (annotations moved around during the Grease
  Pencil v3 change). If Surface placement is unreliable, fall back to a
  small modal draw operator that raycasts the mouse onto the mesh.

## G2: one click in, one click out (genforge)

- **Open:** "Open genforge Character" takes a GLB from genforge's
  `<asset>/hand-edit/` folder (file browser starts there). It imports with
  "Guess Original Bind Pose" off, hides the armature, selects the mesh to
  remesh, frames it, starts the guides layer, and remembers the asset name.
- **Send:** "Send Guides to genforge" exports the guides next to the GLB and
  runs `gen character guides <asset> <file>`, which already attaches them
  and resumes the stopped run. It shows the command's result in the panel.
  The path to `gen` is an add-on preference that defaults to PATH.
- **Launch from genforge (small genforge change):** a `--open` flag on
  `gen character guides` (or the "Draw guides" answer in a notice) starts
  Blender with the add-on and opens the character directly, so the flow
  starts from the Decisions item.
- **Coordinate check:** the adapter imports the same GLB the same way, so
  local coordinates line up. A test asserts the round trip (draw, export,
  adapter import, `retopo` input) stays within 1 mm.

## G3: helpers (after G1-G2 prove out)

- **Suggest Guides:** generate starting rings from the rig's joints
  (`retopo`'s existing joints-to-guides helper) as editable strokes on the
  guides layer. You fix them up instead of starting from nothing.
- **Several meshes:** one guides file per mesh. Today a character with
  several meshes sends the same file to every mesh. Needs the adapter and
  genforge to key guides by mesh name.
- **Density brush:** pass the existing density vertex group through
  genforge and the adapter (`--density`), so you can paint "more detail
  here" (face, hands) the same way.

## Verification

- **Headless tests:** scripted strokes on fixtures (a cylinder, a torso,
  Andras). Every exported point lies on the surface (under 1 mm), mirrored
  strokes match, gaps split, the guides layer survives save and reload.
  Covers 4.2 and 5.2.
- **Real run:** scripted shoulder rings and groin lines on Andras's 1.5M
  Tripo mesh, then genforge rehearsal (free) with and without them.
  Compare weightforge scores, and before/after edge-flow renders that the
  agent looks at itself.
- **UI:** screenshots of the panel and of strokes on the mesh, for the
  owner.

## Effort and where it runs

- G1 about 1 agent-day, G2 about half a day plus the small genforge flag,
  G3 about a day.
- It needs Blender, so it runs on the Mac until basement has Blender
  (`czcode/ccez/hosts/build-tools-linux.sh`, run with sudo).
- Repos: retopoforge (add-on, adapter), genforge (`--open`, and per-mesh
  guides in G3).
