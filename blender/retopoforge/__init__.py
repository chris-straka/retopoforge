# SPDX-License-Identifier: GPL-3.0-or-later
# RetopoForge Blender extension: quad remeshing via the retopoforge CLI.
#
# Flow per object: export selection to a temp OBJ (identity transform, so
# local == world and no transform bugs are possible), run the `retopo` CLI
# as a subprocess, import the result and swap it onto the original object.
# The user never sees a file; temp files live in the system temp dir and
# are removed afterwards. New topology cannot carry UVs or vertex colors.
#
# The engine stays MIT-licensed: this GPL extension talks to it only as a
# subprocess over OBJ files, never linked, never imported.

import json
import os
import re
import shutil
import subprocess
import tempfile

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from mathutils import Matrix

# Must equal the module name Blender loaded us under: "retopoforge" on the
# legacy/sys.path test path, "bl_ext.<repo>.retopoforge" as an installed
# extension. A hardcoded id unlinks AddonPreferences on one path or the other.
ADDON_ID = __package__

_SUMMARY_RE = re.compile(
    r"Quads:\s*(\d+).*?Non-quads:\s*(\d+).*?Vertices:\s*(\d+).*?Time:\s*([\d.]+)",
    re.DOTALL,
)

# One line per rung in the CLI's multimode stdout, e.g.
# "LOD 0: target-quads=10000 output=/tmp/x_lod0.obj quads=9051 non-quads=3 ...".
_LOD_RUNG_RE = re.compile(
    # The space before quads= skips the "target-quads=" field earlier on
    # the same line (its prefix is '-', not whitespace).
    r"LOD\s+(\d+):.*?\squads=(\d+).*?non-quads=(\d+)",
)


def find_retopo_binary(explicit_path=""):
    """Locate the `retopo` CLI: explicit preference, PATH, then common
    build-tree locations relative to a retopoforge checkout."""
    if explicit_path:
        expanded = bpy.path.abspath(explicit_path)
        if os.path.isfile(expanded) and os.access(expanded, os.X_OK):
            return expanded
    on_path = shutil.which("retopo")
    if on_path:
        return on_path
    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in (
        os.path.join(here, "..", "..", "build", "cli", "retopo"),
        os.path.join(here, "..", "..", "build", "RetopoForge", "retopo"),
    ):
        candidate = os.path.normpath(candidate)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return ""


def parse_summary(stdout_text):
    """Parse the CLI's `=== retopoforge Report ===` block. Returns
    (quads, non_quads, verts, seconds) or None."""
    match = _SUMMARY_RE.search(stdout_text or "")
    if not match:
        return None
    quads, non_quads, verts, seconds = match.groups()
    return int(quads), int(non_quads), int(verts), float(seconds)


def parse_lod_targets(text):
    """Parse a comma-separated LOD target string into a list of positive
    ints. Raises ValueError with a user-facing message on bad input."""
    parts = [p.strip() for p in (text or "").split(",")]
    targets = []
    for part in parts:
        if not part.isdigit() or int(part) <= 0:
            raise ValueError(
                f"LOD targets must be positive integers, got '{part}' "
                f"in '{text}'")
        targets.append(int(part))
    if not targets:
        raise ValueError("LOD targets must list at least one quad count")
    return targets


# Scene params snapshotted into the recall blob, grouped by Blender type.
_RECALL_INT_KEYS = ("target_quads",)
_RECALL_FLOAT_KEYS = ("sharp_edge", "smooth_normal", "edge_scaling",
                      "adaptivity", "anisotropy",
                      "density_min", "density_max")
_RECALL_BOOL_KEYS = ("apply_modifiers", "keep_original", "symmetry_enabled",
                     "guides_enabled", "density_enabled")
_RECALL_STR_KEYS = ("lod_targets", "density_vertex_group")
_RECALL_MODEL_TYPES = ("ORGANIC", "HARDSURFACE")
_RECALL_SYMMETRY_PLANES = ("AUTO", "X", "Y", "Z")


def _params_snapshot(params):
    snap = {k: int(getattr(params, k)) for k in _RECALL_INT_KEYS}
    snap.update({k: float(getattr(params, k)) for k in _RECALL_FLOAT_KEYS})
    snap.update({k: bool(getattr(params, k)) for k in _RECALL_BOOL_KEYS})
    snap.update({k: str(getattr(params, k)) for k in _RECALL_STR_KEYS})
    snap["model_type"] = str(params.model_type)
    snap["symmetry_plane"] = str(params.symmetry_plane)
    return snap


def _read_recall_blob(context):
    try:
        blob = json.loads(context.scene.retopoforge_recall or "{}")
    except (ValueError, TypeError):
        return {}
    return blob if isinstance(blob, dict) else {}


def save_recall_entry(context, obj_name, params):
    """Remember the params just used for obj_name. A corrupt blob is
    discarded, never merged with: the fresh snapshot wins outright."""
    blob = _read_recall_blob(context)
    blob[obj_name] = _params_snapshot(params)
    context.scene.retopoforge_recall = json.dumps(blob, sort_keys=True)


def load_recall_entry(context, obj_name):
    """Copy the saved params for obj_name into the scene params. Returns
    True when an entry existed (even if some keys were invalid and
    skipped); unknown keys and mistyped values never raise."""
    blob = _read_recall_blob(context)
    entry = blob.get(obj_name)
    if not isinstance(entry, dict):
        return False
    params = context.scene.retopoforge_params
    for key in _RECALL_INT_KEYS:
        value = entry.get(key)
        if isinstance(value, int) and not isinstance(value, bool):
            try:
                setattr(params, key, value)
            except (ValueError, TypeError):
                pass
    for key in _RECALL_FLOAT_KEYS:
        value = entry.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            try:
                setattr(params, key, float(value))
            except (ValueError, TypeError):
                pass
    for key in _RECALL_BOOL_KEYS:
        value = entry.get(key)
        if isinstance(value, bool):
            try:
                setattr(params, key, value)
            except (ValueError, TypeError):
                pass
    for key in _RECALL_STR_KEYS:
        value = entry.get(key)
        if isinstance(value, str):
            try:
                setattr(params, key, value)
            except (ValueError, TypeError):
                pass
    if entry.get("model_type") in _RECALL_MODEL_TYPES:
        try:
            params.model_type = entry["model_type"]
        except (ValueError, TypeError):
            pass
    if entry.get("symmetry_plane") in _RECALL_SYMMETRY_PLANES:
        try:
            params.symmetry_plane = entry["symmetry_plane"]
        except (ValueError, TypeError):
            pass
    return True


def _recall_target_name(context, targets):
    # Multi-object remesh recalls the active object's entry (the one the
    # user most likely tuned for); a non-mesh active object falls back to
    # the first target so single-object runs always recall themselves.
    active = context.view_layer.objects.active
    if (active is not None and active.type == "MESH"
            and any(o.name == active.name for o in targets)):
        return active.name
    return targets[0].name


class RETOPOFORGE_PG_params(bpy.types.PropertyGroup):
    """Shared remesh parameters, stored on the scene so the sidebar panel
    edits persistent values (operator properties drawn in panels are
    transient and not reliably editable)."""

    target_quads: IntProperty(
        name="Target Quads",
        description="Approximate output quad count (engine guidance, expect some undershoot)",
        default=5000, min=4, max=1000000,
    )
    model_type: EnumProperty(
        name="Model Type",
        description="Model type hint for the remesher",
        items=[
            ("ORGANIC", "Organic", "Smooth organic shapes"),
            ("HARDSURFACE", "Hardsurface", "Hard-surface / CAD-like shapes"),
        ],
        default="ORGANIC",
    )
    sharp_edge: FloatProperty(
        name="Sharp Edge",
        description="Dihedral angle above which edges survive as sharp features",
        default=90.0, min=30.0, max=180.0,
    )
    smooth_normal: FloatProperty(
        name="Smooth Normal",
        description="Normal angle threshold for smoothing",
        default=0.0, min=0.0, max=180.0,
    )
    edge_scaling: FloatProperty(
        name="Edge Scaling",
        description="Edge length multiplier (higher = coarser mesh)",
        default=1.0, min=1.0, max=4.0,
    )
    adaptivity: FloatProperty(
        name="Adaptivity",
        description="Curvature-adaptive density: 0 = uniform quads, 1 = dense on curves, sparse on flats",
        default=1.0, min=0.0, max=1.0,
    )
    anisotropy: FloatProperty(
        name="Anisotropy",
        description="Curvature-adaptive elongation: 0 = square quads, 1 = stretched along curvature",
        default=1.0, min=0.0, max=1.0,
    )
    apply_modifiers: BoolProperty(
        name="Apply Modifiers",
        description="Remesh the evaluated mesh with modifiers applied",
        default=True,
    )
    keep_original: BoolProperty(
        name="Keep Original",
        description="Spawn a remeshed copy and hide the original instead of replacing its mesh",
        default=False,
    )
    lod_targets: StringProperty(
        name="LOD Targets",
        description="Comma-separated quad counts for Generate LODs (one rung per value)",
        default="10000,5000,2000",
    )
    symmetry_enabled: BoolProperty(
        name="Symmetry",
        description="Mirror-symmetry constraints (passes --symmetry to the CLI; off by default)",
        default=False,
    )
    symmetry_plane: EnumProperty(
        name="Symmetry Plane",
        description="Symmetry plane: auto-detect the dominant plane, or pin an axis",
        items=[
            ("AUTO", "Auto", "Detect the dominant symmetry plane"),
            ("X", "X", "Mirror across the YZ plane (x = 0)"),
            ("Y", "Y", "Mirror across the XZ plane (y = 0)"),
            ("Z", "Z", "Mirror across the XY plane (z = 0)"),
        ],
        default="AUTO",
    )
    bake_size: IntProperty(
        name="Bake Size",
        description="Width/height of baked textures in pixels",
        default=1024, min=64, max=4096,
    )
    bake_extrusion: FloatProperty(
        name="Bake Extrusion",
        description="Ray extrusion distance for selected-to-active bakes",
        default=0.05, min=0.0, max=10.0,
    )
    bake_margin: IntProperty(
        name="Bake Margin",
        description="Baked UV island margin in pixels",
        default=16, min=0, max=64,
    )
    bake_normal: BoolProperty(
        name="Bake Normal",
        description="Also bake a tangent-space normal map alongside diffuse",
        default=True,
    )
    guides_enabled: BoolProperty(
        name="Flow Guides",
        description="Constrain quad flow to the edge selection on each target (passed as --guides)",
        default=False,
    )
    density_enabled: BoolProperty(
        name="Density Mask",
        description="Drive local density from a vertex group (passed as --density)",
        default=False,
    )
    density_vertex_group: StringProperty(
        name="Density Group",
        description="Vertex group whose weights map to density multipliers",
        default="",
    )
    density_min: FloatProperty(
        name="Density Min",
        description="Multiplier for weight 0 (CLI clamps outside 0.25-4.0)",
        default=0.25, min=0.01, max=8.0,
    )
    density_max: FloatProperty(
        name="Density Max",
        description="Multiplier for weight 1 (CLI clamps outside 0.25-4.0)",
        default=4.0, min=0.01, max=8.0,
    )

    def symmetry_value(self):
        """CLI --symmetry value: off unless the toggle is on, else the
        selected plane (auto/x/y/z, lowercased)."""
        if not self.symmetry_enabled:
            return "off"
        return str(self.symmetry_plane).lower()

    def cli_args(self, binary, input_path, output_path):
        return [
            binary,
            "--input", input_path,
            "--output", output_path,
            "--target-quads", str(self.target_quads),
            "--edge-scaling", repr(float(self.edge_scaling)),
            "--sharp-edge", repr(float(self.sharp_edge)),
            "--smooth-normal", repr(float(self.smooth_normal)),
            "--adaptivity", repr(float(self.adaptivity)),
            "--anisotropy", repr(float(self.anisotropy)),
            "--model-type",
            "hardsurface" if self.model_type == "HARDSURFACE" else "organic",
            "--symmetry", self.symmetry_value(),
        ]


class RetopoForgePreferences(bpy.types.AddonPreferences):
    bl_idname = ADDON_ID

    retopo_binary: StringProperty(
        name="Retopo CLI",
        description="Path to the retopoforge `retopo` binary (empty: search PATH, then the build tree)",
        default="",
        subtype="FILE_PATH",
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "retopo_binary")
        found = find_retopo_binary(self.retopo_binary)
        box = layout.box()
        if found:
            box.label(text="Using: " + found, icon="CHECKMARK")
        else:
            box.label(text="No retopo binary found", icon="ERROR")


def _mesh_objects(context):
    return [o for o in context.selected_objects if o.type == "MESH"]


def _ensure_object_mode():
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")


def _export_selection(context, filepath, apply_modifiers):
    bpy.ops.wm.obj_export(
        filepath=filepath,
        export_selected_objects=True,
        apply_modifiers=apply_modifiers,
        # Local coordinates, not world: the exporter runs while the object
        # is under the identity transform, and local coords stay correct
        # even if that ever changes. Identity + local = belt and suspenders.
        apply_transform=False,
        export_uv=False,
        export_normals=False,
        export_materials=False,
        export_triangulated_mesh=False,
    )


def _import_result(filepath):
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=filepath)
    return [o for o in bpy.data.objects
            if o not in before and o.type == "MESH"]


def trace_edge_chains(mesh):
    """Order the mesh's selected edges into polylines for --guides.

    Each chain is a list of vertex indices. Chains start at open ends
    (degree 1) first, then leftovers (loops, branches); a branch vertex
    ends every chain passing through it. Iteration is index-sorted, so
    the output is deterministic for a given selection."""
    selected = sorted(e.index for e in mesh.edges if e.select)
    if not selected:
        return []
    neighbours = {}
    for edge_index in selected:
        edge = mesh.edges[edge_index]
        v0, v1 = edge.vertices
        neighbours.setdefault(v0, []).append(v1)
        neighbours.setdefault(v1, []).append(v0)
    for vert in neighbours:
        neighbours[vert].sort()
    used = set()
    chains = []

    def walk(start):
        chain = [start]
        vert = start
        while True:
            nxt = None
            for cand in neighbours[vert]:
                if (min(vert, cand), max(vert, cand)) not in used:
                    nxt = cand
                    break
            if nxt is None:
                return chain
            used.add((min(vert, nxt), max(vert, nxt)))
            chain.append(nxt)
            vert = nxt
            # An open end or a branch stops the chain; a loop closes it
            # by returning to the start vertex (no unused edge remains).
            if len(neighbours[vert]) != 2:
                return chain
            if vert == start:
                return chain

    endpoints = sorted(v for v, vs in neighbours.items() if len(vs) == 1)
    for vert in endpoints:
        if any((min(vert, c), max(vert, c)) not in used
               for c in neighbours[vert]):
            chains.append(walk(vert))
    for vert in sorted(neighbours):
        if any((min(vert, c), max(vert, c)) not in used
               for c in neighbours[vert]):
            chains.append(walk(vert))
    return [c for c in chains if len(c) >= 2]


def write_guide_chains(obj, chains, filepath):
    """Write vertex-index chains as a --guides file (local coords, one
    'x y z' per line, blank line between polylines). Returns the number
    of polylines written."""
    mesh = obj.data
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(f"# RetopoForge flow guides from '{obj.name}' "
                f"({len(chains)} polylines)\n")
        for index, chain in enumerate(chains):
            if index:
                f.write("\n")
            for vert in chain:
                co = mesh.vertices[vert].co
                f.write(f"{co.x!r} {co.y!r} {co.z!r}\n")
    return len(chains)


def density_multipliers(obj, group_name, lo, hi):
    """Per-vertex density multipliers in mesh-vertex order: group weight
    0..1 maps linearly onto min..max (swapped when min > max). Vertices
    with no group assignment stay neutral at 1.0, so painting a small
    region never sparsifies the unpainted rest. Raises RuntimeError when
    the group does not exist on the object."""
    group = obj.vertex_groups.get(group_name)
    if group is None:
        raise RuntimeError(
            f"Object '{obj.name}' has no vertex group '{group_name}'")
    lo, hi = (lo, hi) if lo <= hi else (hi, lo)
    multipliers = []
    for vert in obj.data.vertices:
        try:
            weight = group.weight(vert.index)
        except RuntimeError:
            multipliers.append(1.0)
        else:
            multipliers.append(lo + weight * (hi - lo))
    return multipliers


def write_density_multipliers(obj, group_name, multipliers, filepath):
    """Write multipliers as a --density file (one per line, mesh-vertex
    order). Returns the number of entries written."""
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(f"# RetopoForge density mask from '{obj.name}' "
                f"group '{group_name}' ({len(multipliers)} vertices)\n")
        for value in multipliers:
            f.write(f"{value!r}\n")
    return len(multipliers)


def count_obj_vertices(filepath):
    """Count the 'v ' lines in an exported OBJ: the exact vertex count
    the CLI will read (before its weld, which is the identity on clean
    Blender exports)."""
    count = 0
    with open(filepath, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("v "):
                count += 1
    return count


def constraint_args_for_target(params, obj, input_path, tmpdir, tag):
    """Build the [--guides file, --density file] args for one remesh
    target, writing the temp files into tmpdir. Returns (args, notes):
    notes are skip reasons the caller reports as INFO. Guides enabled
    but nothing selected is a skip, not an error, so multi-object runs
    with partial selections still finish; likewise a named-but-missing
    vertex group skips, because the remesh mesh-swap drops vertex
    groups (weights live on the old topology) and a do-over would
    otherwise always cancel on its own recalled settings. Raises
    RuntimeError only for real setup mistakes: no group set, or the
    mask count not matching the exported OBJ (a topology-changing
    modifier with Apply Modifiers on)."""
    args = []
    notes = []
    if params.guides_enabled:
        chains = trace_edge_chains(obj.data)
        if not chains:
            notes.append(f"{obj.name}: guides on but no edge selection, "
                         f"remeshing unconstrained")
        else:
            guides_path = os.path.join(tmpdir, f"guides_{tag}.txt")
            write_guide_chains(obj, chains, guides_path)
            args += ["--guides", guides_path]
    if params.density_enabled:
        group_name = (params.density_vertex_group or "").strip()
        if not group_name:
            raise RuntimeError(
                "Density mask on but no vertex group set "
                "(pick one in the Density panel)")
        if obj.vertex_groups.get(group_name) is None:
            notes.append(f"{obj.name}: vertex group '{group_name}' not "
                         f"found, remeshing unconstrained")
            return args, notes
        multipliers = density_multipliers(
            obj, group_name, float(params.density_min),
            float(params.density_max))
        exported = count_obj_vertices(input_path)
        if exported != len(multipliers):
            raise RuntimeError(
                f"Density mask has {len(multipliers)} entries but the "
                f"exported mesh has {exported} vertices "
                f"(topology-changing modifier? turn off Apply Modifiers "
                f"or apply it first)")
        density_path = os.path.join(tmpdir, f"density_{tag}.txt")
        write_density_multipliers(obj, group_name, multipliers, density_path)
        args += ["--density", density_path]
    return args, notes


class RETOPOFORGE_OT_remesh(bpy.types.Operator):
    """Remesh the selected mesh objects with the retopoforge engine"""

    bl_idname = "retopoforge.remesh"
    bl_label = "Remesh with RetopoForge"
    bl_options = {"REGISTER", "UNDO"}

    def _export_job(self, context, obj, index):
        _ensure_object_mode()
        context.view_layer.update()
        for o in context.selected_objects:
            o.select_set(False)
        obj.select_set(True)
        context.view_layer.objects.active = obj
        # Export under the identity transform so local == world: the
        # round-trip cannot scale, rotate, or move the mesh by accident.
        saved_matrix = obj.matrix_world.copy()
        obj.matrix_world = Matrix()
        context.view_layer.update()
        input_path = os.path.join(self._tmpdir, f"in_{index}.obj")
        _export_selection(context, input_path, self._params.apply_modifiers)
        obj.matrix_world = saved_matrix
        return input_path

    def _finish_job(self, context, job, output_path, stdout_text):
        imported = _import_result(output_path)
        if not imported:
            raise RuntimeError("CLI output imported no mesh objects")
        new_mesh = imported[0].data
        new_mesh.name = job["obj"].name + "_remeshed"
        if self._params.keep_original:
            copy = job["obj"].copy()
            copy.data = new_mesh
            copy.name = job["obj"].name + "_retopo"
            context.collection.objects.link(copy)
            copy.matrix_world = job["matrix"]
            job["obj"].hide_viewport = True
        else:
            old_mesh = job["obj"].data
            job["obj"].data = new_mesh
            if old_mesh.users == 0:
                bpy.data.meshes.remove(old_mesh)
            job["obj"].matrix_world = job["matrix"]
        for temp in imported:
            bpy.data.objects.remove(temp, do_unlink=True)
        summary = parse_summary(stdout_text)
        if summary is not None:
            quads, non_quads, verts, seconds = summary
            line = (f"{job['obj'].name}: {quads} quads, {non_quads} non-quads, "
                    f"{verts} verts in {seconds:.1f}s")
        else:
            line = f"{job['obj'].name}: done (no summary parsed)"
        context.scene.retopoforge_last_report += line + "\n"
        # Both execute and modal paths funnel through here, so one save
        # covers every successful remesh; failures raise before this line.
        save_recall_entry(context, job["obj"].name, self._params)
        return line

    def _cleanup(self, context):
        wm = context.window_manager
        wm.progress_end()
        if getattr(self, "_timer", None) is not None:
            wm.event_timer_remove(self._timer)
            self._timer = None
        for area in context.screen.areas if context.screen else []:
            try:
                area.header_text_set(None)
            except RuntimeError:
                pass
        log_handle = getattr(self, "_log_handle", None)
        if log_handle is not None:
            try:
                log_handle.close()
            except OSError:
                pass
            self._log_handle = None
        tmpdir = getattr(self, "_tmpdir", "")
        if tmpdir and os.path.isdir(tmpdir):
            shutil.rmtree(tmpdir, ignore_errors=True)

    # -- synchronous path (headless use, tests) -------------------------

    def execute(self, context):
        self._params = context.scene.retopoforge_params
        prefs = context.preferences.addons[ADDON_ID].preferences
        binary = find_retopo_binary(prefs.retopo_binary)
        if not binary:
            self.report({"ERROR"}, "retopo binary not found (see add-on preferences)")
            return {"CANCELLED"}
        targets = _mesh_objects(context)
        if not targets:
            self.report({"ERROR"}, "Select at least one mesh object")
            return {"CANCELLED"}
        # Do-overs are one click: a previous remesh of this object restores
        # its exact params (same recall point as the modal invoke below, so
        # headless runs behave identically). self._params aliases the scene
        # group, so the loaded values flow straight into the CLI call.
        recall_name = _recall_target_name(context, targets)
        if load_recall_entry(context, recall_name):
            self.report({"INFO"}, f"Recalled last settings for '{recall_name}'")
        self._tmpdir = tempfile.mkdtemp(prefix="retopoforge_")
        context.scene.retopoforge_last_report = ""
        try:
            for index, obj in enumerate(targets):
                matrix = obj.matrix_world.copy()
                try:
                    input_path = self._export_job(context, obj, index)
                    output_path = os.path.join(self._tmpdir, f"out_{index}.obj")
                    extra, notes = constraint_args_for_target(
                        self._params, obj, input_path, self._tmpdir,
                        str(index))
                    for note in notes:
                        self.report({"INFO"}, note)
                    proc = subprocess.run(
                        self._params.cli_args(binary, input_path, output_path)
                        + extra,
                        capture_output=True, text=True,
                    )
                    if proc.returncode != 0:
                        tail = (proc.stderr or "").strip().splitlines()
                        detail = tail[-1] if tail else "unknown error"
                        raise RuntimeError(f"retopo failed: {detail}")
                    line = self._finish_job(
                        context,
                        {"obj": obj, "matrix": matrix},
                        output_path, proc.stdout)
                    self.report({"INFO"}, line)
                except RuntimeError as exc:
                    obj.matrix_world = matrix
                    self.report({"ERROR"}, str(exc))
                    return {"CANCELLED"}
        finally:
            tmpdir, self._tmpdir = self._tmpdir, ""
            if tmpdir and os.path.isdir(tmpdir):
                shutil.rmtree(tmpdir, ignore_errors=True)
        return {"FINISHED"}

    # -- modal path (interactive use, UI stays alive) --------------------

    def invoke(self, context, event):
        self._params = context.scene.retopoforge_params
        prefs = context.preferences.addons[ADDON_ID].preferences
        self._binary = find_retopo_binary(prefs.retopo_binary)
        if not self._binary:
            self.report({"ERROR"}, "retopo binary not found (see add-on preferences)")
            return {"CANCELLED"}
        targets = _mesh_objects(context)
        if not targets:
            self.report({"ERROR"}, "Select at least one mesh object")
            return {"CANCELLED"}
        # Recall before queueing: at invoke, an entry for this object
        # restores its last-used params so a do-over is one click.
        recall_name = _recall_target_name(context, targets)
        if load_recall_entry(context, recall_name):
            self.report({"INFO"}, f"Recalled last settings for '{recall_name}'")
        self._tmpdir = tempfile.mkdtemp(prefix="retopoforge_")
        self._queue = [{"obj": o, "matrix": o.matrix_world.copy()} for o in targets]
        self._total = len(self._queue)
        self._proc = None
        self._current = None
        self._output_path = ""
        self._log_path = ""
        self._log_handle = None
        context.scene.retopoforge_last_report = ""
        wm = context.window_manager
        wm.progress_begin(0, self._total)
        self._timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        if not self._start_next(context):
            self._cleanup(context)
            return {"CANCELLED"}
        return {"RUNNING_MODAL"}

    def _start_next(self, context):
        """Export the next queued object and spawn its CLI run. Returns
        True, or False after restoring the object's transform and
        reporting (the caller then cleans up and cancels)."""
        self._current = self._queue.pop(0)
        obj = self._current["obj"]
        index = self._total - len(self._queue) - 1
        try:
            input_path = self._export_job(context, obj, index)
            extra, notes = constraint_args_for_target(
                self._params, obj, input_path, self._tmpdir, str(index))
            for note in notes:
                self.report({"INFO"}, note)
        except RuntimeError as exc:
            obj.matrix_world = self._current["matrix"]
            self.report({"ERROR"}, str(exc))
            return False
        self._output_path = os.path.join(self._tmpdir, f"out_{index}.obj")
        # Never PIPEs here: the CLI prints megabytes of progress and the
        # modal poll loop does not drain, so pipes fill and wedge the child
        # (classic deadlock). A log file is unbounded; parse it at completion.
        self._log_path = os.path.join(self._tmpdir, f"run_{index}.log")
        self._log_handle = open(self._log_path, "w")
        self._proc = subprocess.Popen(
            self._params.cli_args(self._binary, input_path, self._output_path)
            + extra,
            stdout=self._log_handle, stderr=subprocess.STDOUT, text=True,
        )
        context.window_manager.progress_update(index)
        return True

    def modal(self, context, event):
        if event.type == "ESC":
            return self._cancel_modal(context)
        if event.type != "TIMER":
            return {"PASS_THROUGH"}
        if self._proc is None or self._proc.poll() is None:
            done = self._total - len(self._queue) - 1
            for area in context.screen.areas:
                if area.type == "VIEW_3D":
                    area.header_text_set(
                        f"RetopoForge: remeshing {done + 1}/{self._total} "
                        f"({self._current['obj'].name}) — ESC to cancel")
                    break
            return {"PASS_THROUGH"}
        # The child has exited (poll() above); close the log so buffers
        # flush, then read the whole transcript for the summary / error tail.
        self._log_handle.close()
        self._log_handle = None
        with open(self._log_path, encoding="utf-8", errors="replace") as f:
            stdout_text = f.read()
        if self._proc.returncode != 0:
            tail = stdout_text.strip().splitlines()
            detail = tail[-1] if tail else "unknown error"
            self._current["obj"].matrix_world = self._current["matrix"]
            self.report({"ERROR"}, f"retopo failed: {detail}")
            self._cleanup(context)
            return {"CANCELLED"}
        try:
            line = self._finish_job(context, self._current,
                                    self._output_path, stdout_text)
        except RuntimeError as exc:
            self._current["obj"].matrix_world = self._current["matrix"]
            self.report({"ERROR"}, str(exc))
            self._cleanup(context)
            return {"CANCELLED"}
        self.report({"INFO"}, line)
        if self._queue:
            context.window_manager.progress_update(
                self._total - len(self._queue))
            if not self._start_next(context):
                self._cleanup(context)
                return {"CANCELLED"}
            return {"PASS_THROUGH"}
        self._cleanup(context)
        return {"FINISHED"}

    def _cancel_modal(self, context):
        if self._proc is not None and self._proc.poll() is None:
            self._proc.kill()
            self._proc.wait()
        for job in [self._current] + (self._queue or []):
            if job is not None:
                job["obj"].matrix_world = job["matrix"]
        self.report({"WARNING"}, "RetopoForge remesh cancelled")
        self._cleanup(context)
        return {"CANCELLED"}


class RETOPOFORGE_OT_generate_lods(bpy.types.Operator):
    """Generate LOD rungs for the selection as <name>_lodN siblings.

    Design: rungs become SIBLING objects, not mesh swaps on the source
    (unlike Remesh) and not hidden backups. The source stays the full-res
    reference, each rung is independently visible/toggleable in the
    outliner, and game-engine LOD tooling expects sibling objects. The
    source mesh and transform are never touched, so keep_original does
    not apply here; on name clashes (re-runs) Blender's usual .001
    suffixing keeps every rung. One CLI invocation emits the whole chain
    (--lods), so this operator stays synchronous: no modal loop needed
    for a single subprocess call.
    """

    bl_idname = "retopoforge.generate_lods"
    bl_label = "Generate LODs"
    bl_options = {"REGISTER", "UNDO"}

    def _export_identity(self, context, obj, filepath, apply_modifiers):
        _ensure_object_mode()
        context.view_layer.update()
        for o in context.selected_objects:
            o.select_set(False)
        obj.select_set(True)
        context.view_layer.objects.active = obj
        # Same identity-transform export as Remesh: local == world, so the
        # round-trip cannot scale, rotate, or move the mesh by accident.
        saved_matrix = obj.matrix_world.copy()
        obj.matrix_world = Matrix()
        context.view_layer.update()
        _export_selection(context, filepath, apply_modifiers)
        obj.matrix_world = saved_matrix

    def _adopt_rung(self, obj, rung_path, rung_index):
        imported = _import_result(rung_path)
        if not imported:
            raise RuntimeError(f"CLI output {rung_path} imported no mesh objects")
        rung = imported[0]
        rung.name = f"{obj.name}_lod{rung_index}"
        rung.data.name = rung.name
        # True siblings: link the source's collections first, then drop any
        # others the importer linked (the active one), so the rung is never
        # momentarily collection-less.
        obj_colls = set(obj.users_collection)
        for coll in obj_colls:
            if rung.name not in coll.objects:
                coll.objects.link(rung)
        for coll in list(rung.users_collection):
            if coll not in obj_colls:
                coll.objects.unlink(rung)
        rung.matrix_world = obj.matrix_world
        rung.select_set(False)
        for temp in imported[1:]:
            bpy.data.objects.remove(temp, do_unlink=True)
        return rung

    def execute(self, context):
        params = context.scene.retopoforge_params
        prefs = context.preferences.addons[ADDON_ID].preferences
        binary = find_retopo_binary(prefs.retopo_binary)
        if not binary:
            self.report({"ERROR"}, "retopo binary not found (see add-on preferences)")
            return {"CANCELLED"}
        targets = _mesh_objects(context)
        if not targets:
            self.report({"ERROR"}, "Select at least one mesh object")
            return {"CANCELLED"}
        try:
            rung_targets = parse_lod_targets(params.lod_targets)
        except ValueError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        tmpdir = tempfile.mkdtemp(prefix="retopoforge_lods_")
        context.scene.retopoforge_last_report = ""
        try:
            for index, obj in enumerate(targets):
                matrix = obj.matrix_world.copy()
                try:
                    input_path = os.path.join(tmpdir, f"in_{index}.obj")
                    self._export_identity(
                        context, obj, input_path, params.apply_modifiers)
                    # --lods overrides --target-quads; the CLI writes
                    # <stem>_lodN.obj next to --output for each rung.
                    output_base = os.path.join(tmpdir, f"out_{index}.obj")
                    args = params.cli_args(binary, input_path, output_base)
                    args += ["--lods", ",".join(str(t) for t in rung_targets)]
                    proc = subprocess.run(args, capture_output=True, text=True)
                    if proc.returncode != 0:
                        tail = (proc.stderr or "").strip().splitlines()
                        detail = tail[-1] if tail else "unknown error"
                        raise RuntimeError(f"retopo failed: {detail}")
                    stem = os.path.splitext(os.path.basename(output_base))[0]
                    rung_paths = [os.path.join(tmpdir, f"{stem}_lod{r}.obj")
                                  for r in range(len(rung_targets))]
                    missing = [p for p in rung_paths if not os.path.isfile(p)]
                    if missing:
                        raise RuntimeError(
                            "retopo produced no output for "
                            + ", ".join(os.path.basename(p) for p in missing))
                    names = [self._adopt_rung(obj, p, r).name
                             for r, p in enumerate(rung_paths)]
                    obj.matrix_world = matrix
                    # Per-rung counts come from the CLI's "LOD N: ... quads=K
                    # non-quads=M" lines; a parse miss still reports success
                    # with names only (the meshes themselves are the proof).
                    counts = {int(r): (int(q), int(nq)) for r, q, nq
                              in _LOD_RUNG_RE.findall(proc.stdout or "")}
                    bits = []
                    for r, name in enumerate(names):
                        if r in counts:
                            quads, non_quads = counts[r]
                            bits.append(f"{name}={quads}q+{non_quads}nq")
                        else:
                            bits.append(name)
                    line = f"{obj.name}: " + ", ".join(bits)
                    context.scene.retopoforge_last_report += line + "\n"
                    save_recall_entry(context, obj.name, params)
                    self.report({"INFO"}, line)
                except RuntimeError as exc:
                    obj.matrix_world = matrix
                    self.report({"ERROR"}, str(exc))
                    return {"CANCELLED"}
            for o in context.selected_objects:
                o.select_set(False)
            for obj in targets:
                obj.select_set(True)
            context.view_layer.objects.active = targets[0]
        finally:
            if os.path.isdir(tmpdir):
                shutil.rmtree(tmpdir, ignore_errors=True)
        return {"FINISHED"}


def _ensure_bake_target(obj, image):
    """Give obj a material with an active Image Texture node holding image
    (the node Cycles bakes into). Reuses the first material slot, creating
    a node-based material only when the object has none."""
    mat = obj.data.materials[0] if len(obj.data.materials) else None
    if mat is None:
        mat = bpy.data.materials.new(name=f"{obj.name}_bake")
        mat.use_nodes = True
        obj.data.materials.append(mat)
    if not mat.use_nodes:
        mat.use_nodes = True
    nodes = mat.node_tree.nodes
    tex = None
    for node in nodes:
        if node.type == "TEX_IMAGE" and node.image is image:
            tex = node
            break
    if tex is None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = image
    for node in nodes:
        node.select = False
    tex.select = True
    nodes.active = tex
    return tex


def _ensure_high_material(obj):
    """Diffuse bakes read the high-poly albedo: an object with no material
    bakes undefined black, so give it a default Principled material. An
    object that already has materials is left strictly alone."""
    if len(obj.data.materials):
        return
    mat = bpy.data.materials.new(name=f"{obj.name}_high")
    mat.use_nodes = True
    obj.data.materials.append(mat)


def _smart_uv_low(context, low):
    _ensure_object_mode()
    context.view_layer.update()
    for o in context.selected_objects:
        o.select_set(False)
    low.select_set(True)
    context.view_layer.objects.active = low
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.smart_project(angle_limit=66.0, island_margin=0.02)
    bpy.ops.object.mode_set(mode="OBJECT")


class RETOPOFORGE_OT_bake_textures(bpy.types.Operator):
    """Bake diffuse (+ normal) from the HIGH-poly active object to the
    selected LOW-poly target (Smart-UV, Cycles CPU, selected-to-active)"""

    bl_idname = "retopoforge.bake_textures"
    bl_label = "Bake High to Low"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        params = context.scene.retopoforge_params
        high = context.view_layer.objects.active
        if high is None or high.type != "MESH":
            self.report({"ERROR"},
                        "Make the HIGH-poly source the active object")
            return {"CANCELLED"}
        lows = [o for o in context.selected_objects
                if o.type == "MESH" and o != high]
        if not lows:
            self.report({"ERROR"},
                        "Select the LOW-poly target (active object is the HIGH source)")
            return {"CANCELLED"}
        low = lows[0]
        if len(lows) > 1:
            self.report({"INFO"},
                        f"Multiple LOW candidates; baking to '{low.name}'")

        _smart_uv_low(context, low)
        _ensure_high_material(high)

        outdir = os.path.dirname(bpy.data.filepath) or "/tmp"
        size = int(params.bake_size)
        diffuse = bpy.data.images.new(f"{low.name}_diffuse", size, size,
                                      alpha=True)
        diffuse.file_format = "PNG"
        diffuse_path = os.path.join(outdir, f"{low.name}_diffuse.png")
        diffuse.filepath_raw = diffuse_path
        tex_node = _ensure_bake_target(low, diffuse)
        normal = None
        normal_path = ""
        if params.bake_normal:
            normal = bpy.data.images.new(f"{low.name}_normal", size, size,
                                         alpha=True)
            normal.file_format = "PNG"
            normal_path = os.path.join(outdir, f"{low.name}_normal.png")
            normal.filepath_raw = normal_path

        scene = context.scene
        saved_engine = scene.render.engine
        saved_samples = scene.cycles.samples if saved_engine == "CYCLES" else None
        scene.render.engine = "CYCLES"
        scene.cycles.device = "CPU"
        scene.cycles.samples = 1
        bake = scene.render.bake
        bake.use_selected_to_active = True
        bake.use_cage = False
        bake.cage_extrusion = float(params.bake_extrusion)
        bake.margin = int(params.bake_margin)
        bake.use_clear = True

        # Blender's selected-to-active bakes onto the ACTIVE object, so low
        # takes over as active for the bake itself (restored afterwards);
        # the user-facing contract stays active=HIGH at invoke time.
        for o in context.selected_objects:
            o.select_set(False)
        high.select_set(True)
        low.select_set(True)
        context.view_layer.objects.active = low
        context.view_layer.update()
        try:
            bpy.ops.object.bake(type="DIFFUSE", pass_filter={"COLOR"},
                                use_clear=True)
            diffuse.save()
            line = f"{low.name}: diffuse -> {diffuse_path}"
            if normal is not None:
                tex_node.image = normal
                bpy.ops.object.bake(type="NORMAL", use_clear=True)
                normal.save()
                line += f"\n{low.name}: normal -> {normal_path}"
        except RuntimeError as exc:
            self.report({"ERROR"}, f"Bake failed: {exc}")
            return {"CANCELLED"}
        finally:
            for o in context.selected_objects:
                o.select_set(False)
            high.select_set(True)
            low.select_set(True)
            context.view_layer.objects.active = high
            # The bake needs Cycles, but the scene is the user's: put the
            # render settings back the way they were.
            scene.render.engine = saved_engine
            if saved_samples is not None:
                scene.cycles.samples = saved_samples

        scene.retopoforge_last_report += line + "\n"
        self.report({"INFO"}, line.replace("\n", " | "))
        return {"FINISHED"}


class RETOPOFORGE_OT_export_guides(bpy.types.Operator):
    """Export the active object's edge selection as a --guides file"""

    bl_idname = "retopoforge.export_guides"
    bl_label = "Export Guide Strokes"
    bl_options = {"REGISTER"}

    filepath: StringProperty(
        name="Guides File",
        description="Where to write the --guides polyline file",
        default="",
        subtype="FILE_PATH",
    )

    def invoke(self, context, event):
        if not self.filepath:
            context.window_manager.fileselect_add(self)
            return {"RUNNING_MODAL"}
        return self.execute(context)

    def execute(self, context):
        obj = context.view_layer.objects.active
        if obj is None or obj.type != "MESH":
            self.report({"ERROR"}, "Make a mesh object the active object")
            return {"CANCELLED"}
        chains = trace_edge_chains(obj.data)
        if not chains:
            self.report({"ERROR"},
                        f"No edge selection on '{obj.name}' "
                        f"(select flow-stroke edges in Edit Mode first)")
            return {"CANCELLED"}
        if not self.filepath:
            self.report({"ERROR"}, "No output file given")
            return {"CANCELLED"}
        path = bpy.path.abspath(self.filepath)
        count = write_guide_chains(obj, chains, path)
        points = sum(len(c) for c in chains)
        self.report({"INFO"},
                    f"Exported {count} guide polylines "
                    f"({points} points) to {path}")
        return {"FINISHED"}


class RETOPOFORGE_OT_export_density(bpy.types.Operator):
    """Export the active object's density vertex group as a --density file"""

    bl_idname = "retopoforge.export_density"
    bl_label = "Export Density Mask"
    bl_options = {"REGISTER"}

    filepath: StringProperty(
        name="Density File",
        description="Where to write the --density multiplier file",
        default="",
        subtype="FILE_PATH",
    )

    def invoke(self, context, event):
        if not self.filepath:
            context.window_manager.fileselect_add(self)
            return {"RUNNING_MODAL"}
        return self.execute(context)

    def execute(self, context):
        obj = context.view_layer.objects.active
        if obj is None or obj.type != "MESH":
            self.report({"ERROR"}, "Make a mesh object the active object")
            return {"CANCELLED"}
        params = context.scene.retopoforge_params
        group_name = (params.density_vertex_group or "").strip()
        if not group_name:
            self.report({"ERROR"},
                        "No vertex group set (pick one in the Density panel)")
            return {"CANCELLED"}
        if not self.filepath:
            self.report({"ERROR"}, "No output file given")
            return {"CANCELLED"}
        try:
            multipliers = density_multipliers(
                obj, group_name, float(params.density_min),
                float(params.density_max))
        except RuntimeError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        path = bpy.path.abspath(self.filepath)
        count = write_density_multipliers(obj, group_name, multipliers, path)
        self.report({"INFO"},
                    f"Exported {count} density multipliers "
                    f"(group '{group_name}') to {path}")
        return {"FINISHED"}


class RETOPOFORGE_OT_reload(bpy.types.Operator):
    """Reload all scripts (picks up extension updates), then confirm"""

    bl_idname = "retopoforge.reload_scripts"
    bl_label = "Reload Scripts"

    def execute(self, context):
        bpy.ops.script.reload()
        self.report({"INFO"}, "Scripts reloaded")
        return {"FINISHED"}


class RETOPOFORGE_PT_panel(bpy.types.Panel):
    bl_label = "RetopoForge"
    bl_idname = "RETOPOFORGE_PT_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "RetopoForge"

    def draw(self, context):
        layout = self.layout
        prefs = context.preferences.addons[ADDON_ID].preferences
        binary = find_retopo_binary(prefs.retopo_binary)
        status = layout.box()
        if binary:
            status.label(text="retopo: found", icon="CHECKMARK")
        else:
            status.label(text="retopo: missing", icon="ERROR")
            status.label(text="Set the path in Preferences")
        layout.operator("retopoforge.remesh", text="Remesh Selected",
                        icon="MOD_REMESH")
        params = context.scene.retopoforge_params
        col = layout.column(align=True)
        col.prop(params, "target_quads")
        col.prop(params, "model_type")
        col.prop(params, "sharp_edge")
        col.prop(params, "smooth_normal")
        col.prop(params, "edge_scaling")
        col.prop(params, "adaptivity")
        col.prop(params, "anisotropy")
        col.prop(params, "apply_modifiers")
        col.prop(params, "keep_original")
        col.prop(params, "symmetry_enabled")
        sym_row = col.row()
        sym_row.enabled = params.symmetry_enabled
        sym_row.prop(params, "symmetry_plane")
        guides = layout.box()
        guides.label(text="Flow Guides: edge selection, per target")
        guides.prop(params, "guides_enabled")
        guides.operator("retopoforge.export_guides",
                        text="Export Guide Strokes", icon="GREASEPENCIL")
        density = layout.box()
        density.label(text="Density: vertex group weights")
        density.prop(params, "density_enabled")
        dcol = density.column(align=True)
        dcol.enabled = params.density_enabled
        active = context.view_layer.objects.active
        if active is not None and active.type == "MESH":
            dcol.prop_search(params, "density_vertex_group", active,
                             "vertex_groups", text="Group")
        else:
            dcol.prop(params, "density_vertex_group")
        drow = dcol.row(align=True)
        drow.prop(params, "density_min")
        drow.prop(params, "density_max")
        density.operator("retopoforge.export_density",
                         text="Export Density Mask", icon="GROUP_VERTEX")
        col.prop(params, "lod_targets")
        layout.operator("retopoforge.generate_lods", text="Generate LODs",
                        icon="MOD_DECIM")
        bake = layout.box()
        bake.label(text="Bake Assist: Active = HIGH, Selected = LOW")
        bcol = bake.column(align=True)
        bcol.prop(params, "bake_size")
        bcol.prop(params, "bake_extrusion")
        bcol.prop(params, "bake_margin")
        bcol.prop(params, "bake_normal")
        bake.operator("retopoforge.bake_textures", text="Bake High to Low",
                      icon="RENDER_RESULT")
        report = context.scene.retopoforge_last_report
        if report:
            box = layout.box()
            for line in report.strip().split("\n"):
                box.label(text=line)
        layout.label(text="Remeshing replaces topology;", icon="INFO")
        layout.label(text="UVs and vertex colors do not survive.")
        # Dev convenience: picks up extension updates without restarting
        # Blender, with an INFO report as visible confirmation.
        layout.operator("retopoforge.reload_scripts", text="Reload Scripts",
                        icon="FILE_REFRESH")


_CLASSES = (
    RetopoForgePreferences,
    RETOPOFORGE_PG_params,
    RETOPOFORGE_OT_remesh,
    RETOPOFORGE_OT_generate_lods,
    RETOPOFORGE_OT_bake_textures,
    RETOPOFORGE_OT_export_guides,
    RETOPOFORGE_OT_export_density,
    RETOPOFORGE_OT_reload,
    RETOPOFORGE_PT_panel,
)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.retopoforge_params = PointerProperty(
        type=RETOPOFORGE_PG_params)
    bpy.types.Scene.retopoforge_last_report = StringProperty(
        name="Last Report",
        description="Stats from the most recent retopoforge remesh",
        default="",
    )
    bpy.types.Scene.retopoforge_recall = StringProperty(
        name="Settings Recall",
        description="Per-object last-used remesh parameters as a JSON blob",
        default="",
    )


def unregister():
    del bpy.types.Scene.retopoforge_recall
    del bpy.types.Scene.retopoforge_last_report
    del bpy.types.Scene.retopoforge_params
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
