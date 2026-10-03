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
from mathutils import Matrix, Vector

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
        # Built from a retopoforge checkout (cargo).
        os.path.join(here, "..", "..", "rust", "target", "release", "retopo"),
        os.path.join(here, "..", "..", "rust", "target", "debug", "retopo"),
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
                     "guides_enabled", "features_enabled", "density_enabled")
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
    compute_uvs: BoolProperty(
        name="Compute UVs",
        description="Ask the engine for fresh UVs on the remesh (passed as --uvs on)",
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
    bake_roughness: BoolProperty(
        name="Bake Roughness",
        description="Bake roughness when the HIGH source uses it",
        default=True,
    )
    bake_metallic: BoolProperty(
        name="Bake Metallic",
        description="Bake metallic when the HIGH source uses it",
        default=True,
    )
    bake_ao: BoolProperty(
        name="Bake AO",
        description="Bake ambient occlusion from the HIGH source",
        default=True,
    )
    bake_emission: BoolProperty(
        name="Bake Emission",
        description="Bake emission when the HIGH source emits light",
        default=True,
    )
    bake_uv_mode: EnumProperty(
        name="LOW UVs",
        description="How the LOW bake target gets UVs",
        items=[
            ("SMART", "Smart UV", "Fast automatic projection (current)"),
            ("UNWRAP", "Unwrap + Pack",
             "Angle-based/conformal unwrap, uniform texel density, packed"),
        ],
        default="SMART",
    )
    bake_unwrap_method: EnumProperty(
        name="Unwrap Method",
        description="Unwrap algorithm for Unwrap + Pack mode",
        items=[
            ("ANGLE_BASED", "Angle Based", "Best for organic shapes"),
            ("CONFORMAL", "Conformal", "Best for hard-surface shapes"),
        ],
        default="ANGLE_BASED",
    )
    bake_pack_margin: FloatProperty(
        name="Pack Margin",
        description="Island margin for Unwrap + Pack mode (UV units)",
        default=0.005,
        min=0.0,
        max=0.1,
    )
    bake_average_scale: BoolProperty(
        name="Uniform Texel Density",
        description="Average island scale in Unwrap + Pack mode so every "
                    "island shares one texel density",
        default=True,
    )
    bake_texel_density: FloatProperty(
        name="Texel Density (px/unit)",
        description="Target texel density for Unwrap + Pack mode "
                    "(0: pack fit only, density is reported)",
        default=0.0,
        min=0.0,
    )
    project_uv_max_dist: FloatProperty(
        name="Projection Range",
        description="Max HIGH distance for UV projection, as a fraction "
                    "of the HIGH bounding-box diagonal (faces beyond "
                    "keep their UVs)",
        default=0.01,
        min=0.0,
        max=1.0,
    )
    transfer_max_dist: FloatProperty(
        name="Transfer Range",
        description="Max HIGH distance for color transfer, as a fraction "
                    "of the HIGH bounding-box diagonal (faces beyond "
                    "keep the fill color)",
        default=0.01,
        min=0.0,
        max=1.0,
    )
    bake_cage: PointerProperty(
        name="Bake Cage",
        description="Cage mesh for selected-to-active bakes (empty: ray extrusion)",
        type=bpy.types.Object,
    )
    guides_enabled: BoolProperty(
        name="Flow Guides",
        description="Constrain quad flow to the edge selection on each target (passed as --guides)",
        default=False,
    )
    features_enabled: BoolProperty(
        name="Sharp Features",
        description="Keep sharp-marked edges crisp on each target (passed as --features)",
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
            "--uvs", "on" if self.compute_uvs else "off",
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


def _operator_has_property(op, name):
    return name in op.get_rna_type().properties.keys()


def _export_selection(context, filepath, apply_modifiers):
    kwargs = dict(
        filepath=filepath,
        export_selected_objects=True,
        apply_modifiers=apply_modifiers,
        # Identity axis mapping (verified empirically: a (1,2,4) marker
        # round-trips byte-identical). Defaults rotate -90 deg about X;
        # NEGATIVE_Y/Z rotates 180 deg about Z. The importer applies no
        # axis mapping at all, so export must be identity.
        forward_axis="Y",
        up_axis="Z",
        export_uv=False,
        export_normals=False,
        export_materials=False,
        export_triangulated_mesh=False,
    )
    # Local coordinates, not world: the exporter runs while the object is
    # under the identity transform, and local coords stay correct even if
    # that ever changes. Identity + local = belt and suspenders. The
    # keyword only exists on Blender 5.0+; the 4.x LTS exporters reject
    # it, and there the identity transform alone already guarantees it.
    if _operator_has_property(bpy.ops.wm.obj_export, "apply_transform"):
        kwargs["apply_transform"] = False
    bpy.ops.wm.obj_export(**kwargs)


def _import_result(filepath):
    before = set(bpy.data.objects)
    # Axis params are accepted but applied as identity by the importer
    # (probed: file coords land in the mesh verbatim either way); kept
    # matching _export_selection for documentation value.
    bpy.ops.wm.obj_import(filepath=filepath, forward_axis="Y", up_axis="Z")
    return [o for o in bpy.data.objects
            if o not in before and o.type == "MESH"]


def trace_edge_chains(mesh):
    """Order the mesh's selected edges into polylines for --guides.

    Each chain is a list of vertex indices. Chains start at open ends
    (degree 1) first, then leftovers (loops, branches); a branch vertex
    ends every chain passing through it. Iteration is index-sorted, so
    the output is deterministic for a given selection."""
    selected = sorted(e.index for e in mesh.edges if e.select)
    return _trace_chains(mesh, selected)


def trace_sharp_chains(mesh):
    """Order the mesh's sharp-marked edges into polylines for --features.

    Same deterministic chain tracing as trace_edge_chains, but over the
    Edge > Mark Sharp flags instead of the edit-mode selection."""
    selected = sorted(e.index for e in mesh.edges if e.use_edge_sharp)
    return _trace_chains(mesh, selected)


def _trace_chains(mesh, selected):
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


def write_feature_chains(obj, chains, filepath):
    """Write vertex-index chains as a --features file (local coords, one
    'x y z' per line, blank line between polylines). Returns the number
    of polylines written."""
    mesh = obj.data
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(f"# RetopoForge sharp features from '{obj.name}' "
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
    """Build the [--guides file, --features file, --density file] args
    for one remesh target, writing the temp files into tmpdir. Returns
    (args, notes): notes are skip reasons the caller reports as INFO.
    Guides enabled but nothing selected is a skip, not an error, so
    multi-object runs with partial selections still finish; likewise
    sharp features enabled but nothing marked skips, and a
    named-but-missing vertex group skips, because the remesh mesh-swap
    drops vertex groups (weights live on the old topology) and a
    do-over would otherwise always cancel on its own recalled
    settings. Raises RuntimeError only for real setup mistakes: no
    group set, or the mask count not matching the exported OBJ (a
    topology-changing modifier with Apply Modifiers on)."""
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
    if params.features_enabled:
        chains = trace_sharp_chains(obj.data)
        if not chains:
            notes.append(f"{obj.name}: sharp features on but no marked "
                         f"edges, remeshing unconstrained")
        else:
            features_path = os.path.join(tmpdir, f"features_{tag}.txt")
            write_feature_chains(obj, chains, features_path)
            args += ["--features", features_path]
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

    # Chained callers (Remesh + Bake All) need HIGH intact whatever the
    # panel or the recalled settings say; never saved into the recall blob.
    force_keep_original: BoolProperty(
        name="Force Keep Original",
        default=False,
        options={"HIDDEN", "SKIP_SAVE"},
    )

    def _keep_original(self):
        return self._params.keep_original or self.force_keep_original

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
        if self._keep_original():
            result_obj = job["obj"].copy()
            result_obj.data = new_mesh
            result_obj.name = job["obj"].name + "_retopo"
            # Sibling of the source: same collections, not whichever
            # collection happens to be active.
            for coll in job["obj"].users_collection:
                coll.objects.link(result_obj)
            result_obj.matrix_world = job["matrix"]
            job["obj"].hide_viewport = True
        else:
            result_obj = job["obj"]
            old_mesh = result_obj.data
            result_obj.data = new_mesh
            if old_mesh.users == 0:
                bpy.data.meshes.remove(old_mesh)
            result_obj.matrix_world = job["matrix"]
        if self._params.apply_modifiers:
            # The exported mesh already had the stack applied; leaving it
            # live would apply it twice (a Mirror doubles, a Subsurf
            # re-smooths the fresh quads), exactly like Blender's Apply.
            result_obj.modifiers.clear()
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


# Marks a LOW material the bake created (safe to rewire on every bake);
# a material the user brought is only given image nodes, never rewired.
_BAKE_MATERIAL_TAG = "retopoforge_bake"
# Data maps store raw values: tagging them Non-Color keeps Blender from
# sRGB-encoding the bake and makes the saved PNGs load back correctly.
_NON_COLOR_MAPS = frozenset({"normal", "roughness", "metallic", "ao"})
# AO is the one Monte Carlo pass here; one sample is pure noise.
_AO_BAKE_SAMPLES = 64
# Scene settings the bake overrides, restored afterwards (dotted paths
# from the scene).
_BAKE_SCENE_SETTINGS = (
    "render.engine",
    "cycles.device",
    "cycles.samples",
    "render.bake.use_selected_to_active",
    "render.bake.use_cage",
    "render.bake.cage_object",
    "render.bake.cage_extrusion",
    "render.bake.margin",
    "render.bake.use_clear",
)


def _snapshot_settings(scene, paths):
    saved = []
    for path in paths:
        owner_path, _, attr = path.rpartition(".")
        owner = scene
        for part in owner_path.split("."):
            owner = getattr(owner, part)
        saved.append((owner, attr, getattr(owner, attr)))
    return saved


def _restore_settings(saved):
    for owner, attr, value in saved:
        try:
            setattr(owner, attr, value)
        except (AttributeError, TypeError, ValueError):
            pass


def _bake_image(name, size, filepath, non_color):
    """The image a map bakes into, reused across re-bakes (so repeated
    bakes overwrite one datablock and file instead of piling up .001
    copies)."""
    image = bpy.data.images.get(name)
    if image is None:
        image = bpy.data.images.new(name, size, size, alpha=True)
    else:
        # Back to a blank generated buffer at the requested size: the
        # last bake's PNG may since have been moved or deleted, and a
        # FILE image would then fail to load its buffer.
        image.source = "GENERATED"
        image.generated_type = "BLANK"
        image.generated_width = size
        image.generated_height = size
    image.file_format = "PNG"
    image.filepath_raw = filepath
    try:
        image.colorspace_settings.name = ("Non-Color" if non_color
                                          else "sRGB")
    except TypeError:
        pass
    return image


def _bake_material(obj):
    """obj's first material, the one the bake target nodes live in.
    An object with no (or an empty) first slot gets a fresh tagged
    node material there."""
    mats = obj.data.materials
    mat = mats[0] if len(mats) else None
    if mat is None:
        mat = bpy.data.materials.new(name=f"{obj.name}_bake")
        mat.use_nodes = True
        mat[_BAKE_MATERIAL_TAG] = True
        if len(mats):
            mats[0] = mat
        else:
            mats.append(mat)
    if not mat.use_nodes:
        mat.use_nodes = True
    return mat


def _bake_node(mat, name, image):
    """The Image Texture node holding one baked map (found by name on
    re-bakes)."""
    nodes = mat.node_tree.nodes
    node_name = f"RetopoForge {name}"
    node = nodes.get(node_name)
    if node is None or node.type != "TEX_IMAGE":
        node = nodes.new("ShaderNodeTexImage")
        node.name = node_name
        node.label = node_name
    node.image = image
    return node


def _activate_bake_node(mat, node):
    """Cycles bakes into the material's active Image Texture node."""
    for other in mat.node_tree.nodes:
        other.select = False
    node.select = True
    mat.node_tree.nodes.active = node


def _wire_baked_maps(mat, map_nodes):
    """Hook the baked maps into a bake-created material's Principled
    BSDF so the LOW renders textured right away. AO has no Principled
    input and stays an unlinked node."""
    if not mat.get(_BAKE_MATERIAL_TAG):
        return
    tree = mat.node_tree
    bsdf = next((n for n in tree.nodes if n.type == "BSDF_PRINCIPLED"),
                None)
    if bsdf is None:
        return

    def feed(socket_name, output):
        socket = bsdf.inputs.get(socket_name)
        if socket is None:
            return
        for link in list(socket.links):
            tree.links.remove(link)
        tree.links.new(output, socket)

    order = ("diffuse", "roughness", "metallic", "normal", "emission", "ao")
    for row, name in enumerate(n for n in order if n in map_nodes):
        map_nodes[name].location = (bsdf.location.x - 600,
                                    bsdf.location.y - 280 * row)
    for name, socket_name in (("diffuse", "Base Color"),
                              ("roughness", "Roughness"),
                              ("metallic", "Metallic"),
                              ("emission", "Emission Color")):
        if name in map_nodes:
            feed(socket_name, map_nodes[name].outputs["Color"])
    if "emission" in map_nodes:
        strength = bsdf.inputs.get("Emission Strength")
        if strength is not None and not strength.is_linked:
            strength.default_value = 1.0
    if "normal" in map_nodes:
        normal_map = tree.nodes.get("RetopoForge normal map")
        if normal_map is None or normal_map.type != "NORMAL_MAP":
            normal_map = tree.nodes.new("ShaderNodeNormalMap")
            normal_map.name = "RetopoForge normal map"
            normal_map.label = normal_map.name
        normal_map.location = (bsdf.location.x - 250,
                               map_nodes["normal"].location.y)
        for link in list(normal_map.inputs["Color"].links):
            tree.links.remove(link)
        tree.links.new(map_nodes["normal"].outputs["Color"],
                       normal_map.inputs["Color"])
        feed("Normal", normal_map.outputs["Normal"])


def _ensure_high_material(obj):
    """Diffuse bakes read the high-poly albedo: an object with no material
    bakes undefined black, so give it a default Principled material. An
    object that already has materials is left strictly alone."""
    if len(obj.data.materials):
        return
    mat = bpy.data.materials.new(name=f"{obj.name}_high")
    mat.use_nodes = True
    obj.data.materials.append(mat)


def _principled_nodes(obj):
    """Yield (material, Principled BSDF node) over obj's node materials."""
    for slot in obj.data.materials:
        mat = slot
        if mat is None or not mat.use_nodes:
            continue
        for node in mat.node_tree.nodes:
            if node.type == "BSDF_PRINCIPLED":
                yield mat, node
                break


def _socket_used(obj, socket, default):
    """True when any Principled `socket` on obj is texture-linked or set
    off its default (i.e. the HIGH source genuinely carries that map)."""
    for _, bsdf in _principled_nodes(obj):
        inp = bsdf.inputs.get(socket)
        if inp is None:
            continue
        if inp.is_linked:
            return True
        val = inp.default_value
        if isinstance(val, float):
            if abs(val - default) > 1e-9:
                return True
        else:
            try:
                if any(abs(c - d) > 1e-9 for c, d in zip(val, default)):
                    return True
            except TypeError:
                pass
    return False


def _emission_used(obj):
    """True when any Principled emits non-black light."""
    for _, bsdf in _principled_nodes(obj):
        color = bsdf.inputs.get("Emission Color")
        strength = bsdf.inputs.get("Emission Strength")
        if color is None or strength is None:
            continue
        sval = strength.default_value
        if strength.is_linked or sval > 1e-9:
            if color.is_linked:
                return True
            if any(c > 1e-9 for c in color.default_value[:3]):
                return True
    return False


def _metallic_bake_source(context, high):
    """Duplicate high with single-user materials rewired Metallic->Emission
    (Blender has no metallic bake type; an EMIT bake of the duplicate
    transfers the metallic map). Returns the dup; the caller must delete
    it (object + mesh + copied materials). Returns None when no
    Principled metallic is in play."""
    if not _socket_used(high, "Metallic", 0.0):
        return None
    dup = high.copy()
    dup.data = high.data.copy()
    dup.name = f"{high.name}_metallic_src"
    context.collection.objects.link(dup)
    for i, slot in enumerate(list(dup.data.materials)):
        if slot is None or not slot.use_nodes:
            continue
        mat = slot.copy()
        dup.data.materials[i] = mat
        bsdf = next((n for n in mat.node_tree.nodes
                     if n.type == "BSDF_PRINCIPLED"), None)
        out = next((n for n in mat.node_tree.nodes
                    if n.type == "OUTPUT_MATERIAL"), None)
        if bsdf is None or out is None:
            continue
        metallic = bsdf.inputs.get("Metallic")
        if metallic is None:
            continue
        emission = mat.node_tree.nodes.new("ShaderNodeEmission")
        emission.inputs["Strength"].default_value = 1.0
        if metallic.is_linked:
            src = metallic.links[0].from_socket
            mat.node_tree.links.new(src, emission.inputs["Color"])
        else:
            v = float(metallic.default_value)
            emission.inputs["Color"].default_value = (v, v, v, 1.0)
        for link in list(out.inputs["Surface"].links):
            mat.node_tree.links.remove(link)
        mat.node_tree.links.new(emission.outputs["Emission"],
                                out.inputs["Surface"])
    return dup


def _delete_bake_source(context, dup):
    if dup is None:
        return
    mesh = dup.data
    mats = [s for s in mesh.materials if s is not None]
    bpy.data.objects.remove(dup, do_unlink=True)
    bpy.data.meshes.remove(mesh, do_unlink=True)
    for mat in mats:
        if mat.users == 0:
            bpy.data.materials.remove(mat)


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


def _uv_world_areas(obj):
    """(total UV area, total world area) over obj's active UV layer."""
    mesh = obj.data
    uv_layer = mesh.uv_layers.active
    if uv_layer is None:
        return 0.0, 0.0
    world = obj.matrix_world.to_3x3()
    uv_area = 0.0
    world_area = 0.0
    for poly in mesh.polygons:
        loop_uvs = [uv_layer.uv[li].vector for li in poly.loop_indices]
        for a, b, c in zip(loop_uvs, loop_uvs[1:], loop_uvs[2:]):
            uv_area += abs((b.x - a.x) * (c.y - a.y)
                           - (c.x - a.x) * (b.y - a.y)) / 2.0
        verts = [world @ mesh.vertices[vi].co for vi in poly.vertices]
        origin = verts[0]
        for b, c in zip(verts[1:], verts[2:]):
            world_area += ((b - origin).cross(c - origin)).length / 2.0
    return uv_area, world_area


def _scale_uvs_about_center(obj, scale):
    mesh = obj.data
    uv_layer = mesh.uv_layers.active
    for item in uv_layer.uv:
        x, y = item.vector
        item.vector = (0.5 + (x - 0.5) * scale, 0.5 + (y - 0.5) * scale)


def _uvs_fit_tile(obj):
    return all(-1e-6 <= c <= 1.0 + 1e-6
               for item in obj.data.uv_layers.active.uv
               for c in item.vector)


def _unwrap_pack_low(context, low, params):
    """Unwrap + uniform-density + pack pipeline for the LOW bake target.
    Returns a one-line report note with the measured texel density
    (px/unit at the bake size). A nonzero density target is enforced by
    a uniform post-pack scale about the tile center when it still fits
    the 0-1 tile (bakes clip outside it); otherwise the pack fit stands
    and the note says what the tile fits."""
    _ensure_object_mode()
    context.view_layer.update()
    for o in context.selected_objects:
        o.select_set(False)
    low.select_set(True)
    context.view_layer.objects.active = low
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.unwrap(method=params.bake_unwrap_method, fill_holes=True,
                      correct_aspect=True,
                      margin=float(params.bake_pack_margin))
    if params.bake_average_scale:
        bpy.ops.uv.average_islands_scale()
    bpy.ops.uv.pack_islands(margin=float(params.bake_pack_margin))
    bpy.ops.object.mode_set(mode="OBJECT")
    size = int(params.bake_size)
    uv_area, world_area = _uv_world_areas(low)
    if uv_area <= 0.0 or world_area <= 0.0:
        return "uv: unwrap+pack (degenerate UVs, density unmeasured)"
    measured = size * (uv_area / world_area) ** 0.5
    target = float(params.bake_texel_density)
    if target > 0.0:
        _scale_uvs_about_center(low, target / measured)
        if _uvs_fit_tile(low):
            return (f"uv: unwrap+pack @ {target:.0f}px/unit "
                    f"(target met, pack fit {measured:.0f})")
        _scale_uvs_about_center(low, measured / target)
        return (f"uv: unwrap+pack @ {measured:.0f}px/unit "
                f"(target {target:.0f} exceeds the tile fit)")
    return f"uv: unwrap+pack @ {measured:.0f}px/unit"


class RETOPOFORGE_OT_bake_textures(bpy.types.Operator):
    """Bake PBR maps from the HIGH-poly active object to the selected
    LOW-poly target (UV prep per panel, Cycles CPU, selected-to-active)"""

    bl_idname = "retopoforge.bake_textures"
    bl_label = "Bake High to Low"
    bl_options = {"REGISTER", "UNDO"}

    # Remesh + Bake All bakes onto a LOW that did not exist a moment ago,
    # so no cage can match it yet: the chained call bakes without one.
    ignore_cage: BoolProperty(
        name="Ignore Cage",
        default=False,
        options={"HIDDEN", "SKIP_SAVE"},
    )

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

        # Validate the cage before touching anything (UV prep rewrites
        # LOW's UVs, so a late cancel would leave them changed).
        cage = None if self.ignore_cage else params.bake_cage
        if cage is not None:
            if cage.type != "MESH":
                self.report({"ERROR"},
                            f"Bake cage '{cage.name}' is not a mesh object")
                return {"CANCELLED"}
            # Blender casts cage rays onto the active (LOW) object and
            # requires matching face counts — reject early with the fix
            # (duplicate LOW, inflate slightly) instead of mid-bake.
            if len(cage.data.polygons) != len(low.data.polygons):
                self.report(
                    {"ERROR"},
                    f"Bake cage '{cage.name}' has "
                    f"{len(cage.data.polygons)} faces but '{low.name}' has "
                    f"{len(low.data.polygons)} (duplicate LOW and inflate it)")
                return {"CANCELLED"}

        uv_note = None
        if params.bake_uv_mode == "UNWRAP":
            uv_note = _unwrap_pack_low(context, low, params)
        else:
            _smart_uv_low(context, low)
        _ensure_high_material(high)

        # Job list: (map name, bake type, pass filter, bake-toggle on,
        # source carries the map). Diffuse + normal + AO always run when
        # toggled (albedo flat-colors still transfer; normal/AO derive
        # from geometry); roughness/metallic/emission skip with a note
        # when no HIGH material feeds that socket.
        metallic_src = None
        if params.bake_metallic:
            metallic_src = _metallic_bake_source(context, high)
        jobs = [
            ("diffuse", "DIFFUSE", {"COLOR"}, True, True),
            ("normal", "NORMAL", set(), params.bake_normal, True),
            ("roughness", "ROUGHNESS", set(), params.bake_roughness,
             _socket_used(high, "Roughness", 0.5)),
            ("metallic", "EMIT", set(), params.bake_metallic,
             metallic_src is not None),
            ("ao", "AO", set(), params.bake_ao, True),
            ("emission", "EMIT", set(), params.bake_emission,
             _emission_used(high)),
        ]

        outdir = os.path.dirname(bpy.data.filepath) or tempfile.gettempdir()
        size = int(params.bake_size)
        mat = _bake_material(low)
        map_nodes = {}
        for name, _, _, enabled, used in jobs:
            if not (enabled and used):
                continue
            image = _bake_image(f"{low.name}_{name}", size,
                                os.path.join(outdir, f"{low.name}_{name}.png"),
                                name in _NON_COLOR_MAPS)
            map_nodes[name] = _bake_node(mat, name, image)

        scene = context.scene
        saved = _snapshot_settings(scene, _BAKE_SCENE_SETTINGS)
        scene.render.engine = "CYCLES"
        scene.cycles.device = "CPU"
        bake = scene.render.bake
        bake.use_selected_to_active = True
        bake.use_cage = cage is not None
        bake.cage_object = cage
        bake.cage_extrusion = float(params.bake_extrusion)
        bake.margin = int(params.bake_margin)
        bake.use_clear = True

        # Blender's selected-to-active bakes onto the ACTIVE object, so low
        # takes over as active for the bake itself (restored afterwards);
        # the user-facing contract stays active=HIGH at invoke time. The
        # metallic job bakes from the rewired duplicate instead of high.
        lines = []
        try:
            for name, btype, filt, enabled, used in jobs:
                if not enabled:
                    continue
                if not used:
                    lines.append(f"{low.name}: {name} skipped (HIGH has no {name})")
                    continue
                src = metallic_src if name == "metallic" else high
                for o in context.selected_objects:
                    o.select_set(False)
                src.select_set(True)
                low.select_set(True)
                context.view_layer.objects.active = low
                context.view_layer.update()
                scene.cycles.samples = _AO_BAKE_SAMPLES if btype == "AO" else 1
                _activate_bake_node(mat, map_nodes[name])
                if filt:
                    bpy.ops.object.bake(type=btype, pass_filter=filt,
                                        use_clear=True)
                else:
                    bpy.ops.object.bake(type=btype, use_clear=True)
                image = map_nodes[name].image
                image.save()
                lines.append(f"{low.name}: {name} -> {image.filepath_raw}")
        except RuntimeError as exc:
            self.report({"ERROR"}, f"Bake failed: {exc}")
            return {"CANCELLED"}
        finally:
            _delete_bake_source(context, metallic_src)
            for o in context.selected_objects:
                o.select_set(False)
            high.select_set(True)
            low.select_set(True)
            context.view_layer.objects.active = high
            # The bake needs Cycles, but the scene is the user's: put the
            # render settings back the way they were.
            _restore_settings(saved)

        _wire_baked_maps(mat, map_nodes)
        _activate_bake_node(mat, map_nodes["diffuse"])
        if uv_note is not None:
            lines.insert(0, f"{low.name}: {uv_note}")
        line = "\n".join(lines)
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


class RETOPOFORGE_OT_export_features(bpy.types.Operator):
    """Export the active object's sharp-marked edges as a --features file"""

    bl_idname = "retopoforge.export_features"
    bl_label = "Export Sharp Features"
    bl_options = {"REGISTER"}

    filepath: StringProperty(
        name="Features File",
        description="Where to write the --features polyline file",
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
        chains = trace_sharp_chains(obj.data)
        if not chains:
            self.report({"ERROR"},
                        f"No sharp-marked edges on '{obj.name}' "
                        f"(mark crisp edges with Edge > Mark Sharp first)")
            return {"CANCELLED"}
        if not self.filepath:
            self.report({"ERROR"}, "No output file given")
            return {"CANCELLED"}
        path = bpy.path.abspath(self.filepath)
        count = write_feature_chains(obj, chains, path)
        points = sum(len(c) for c in chains)
        self.report({"INFO"},
                    f"Exported {count} sharp-feature polylines "
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


def _bary_weights_3d(p, a, b, c):
    """Barycentric weights of p in triangle abc (3D, area-based)."""
    ab = b - a
    ac = c - a
    ap = p - a
    d1 = ab.dot(ap)
    d2 = ac.dot(ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return (1.0, 0.0, 0.0)
    bp = p - b
    d3 = ab.dot(bp)
    d4 = ac.dot(bp)
    if d3 >= 0.0 and d4 <= d3:
        return (0.0, 1.0, 0.0)
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        return (1.0 - v, v, 0.0)
    cp = p - c
    d5 = ab.dot(cp)
    d6 = ac.dot(cp)
    if d6 >= 0.0 and d5 <= d6:
        return (0.0, 0.0, 1.0)
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        return (1.0 - w, 0.0, w)
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return (0.0, 1.0 - w, w)
    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    return (1.0 - v - w, v, w)


def _uv_islands(mesh, uv_layer):
    """Per-polygon UV island ids: polygons sharing an edge join one
    island unless the edge is a UV seam (its two corners carry different
    UVs on either side)."""
    parent = list(range(len(mesh.polygons)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    uvs = uv_layer.uv
    first_owner = {}
    for poly in mesh.polygons:
        verts = poly.vertices
        loops = poly.loop_indices
        n = len(verts)
        for k in range(n):
            va, vb = verts[k], verts[(k + 1) % n]
            ua = tuple(uvs[loops[k]].vector)
            ub = tuple(uvs[loops[(k + 1) % n]].vector)
            key, uv_key = ((va, vb), (ua, ub)) if va < vb else \
                ((vb, va), (ub, ua))
            owner = first_owner.get(key)
            if owner is None:
                first_owner[key] = (poly.index, uv_key)
            elif owner[1] == uv_key:
                parent[find(owner[0])] = find(poly.index)
    return [find(i) for i in range(len(parent))]


def _face_correspondence(high, low, max_dist_frac, island_of=None):
    """Nearest-point mapping LOW corners -> HIGH surface for attribute
    transfer.

    A LOW face is in range when its center lies within max_dist_frac
    of the HIGH bbox diagonal; its nearest HIGH face is the anchor.
    Every corner then maps to its own nearest HIGH point (sampling one
    HIGH face per LOW face would clamp all corners onto that small face
    and shrink the result). With island_of (per-HIGH-polygon UV island
    ids), a corner whose nearest point lies in another island than the
    anchor is re-projected within the anchor's island, so no LOW face
    straddles a UV seam. HIGH is read as its base mesh, so face indices
    always match its data (modifiers are not evaluated).

    Returns (items, projected, skipped): items holds (low_poly, corners)
    with each corner (low_loop, high_loops, high_verts, weights), three
    HIGH loop/vertex indices of the fan triangle holding the projected
    point and its barycentric weights."""
    from mathutils.bvhtree import BVHTree

    high_mesh = high.data
    high_world = high.matrix_world
    hverts = [high_world @ v.co for v in high_mesh.vertices]
    hpolys = high_mesh.polygons
    tree = BVHTree.FromPolygons(hverts, [tuple(p.vertices) for p in hpolys])
    diag = (high_world @ Vector(high.bound_box[6])
            - high_world @ Vector(high.bound_box[0])).length
    max_dist = float(max_dist_frac) * diag

    island_members = {}
    if island_of is not None:
        for index, island in enumerate(island_of):
            island_members.setdefault(island, []).append(index)
    island_trees = {}

    def nearest_in_island(island, point):
        entry = island_trees.get(island)
        if entry is None:
            members = island_members[island]
            entry = (BVHTree.FromPolygons(
                hverts, [tuple(hpolys[i].vertices) for i in members]),
                members)
            island_trees[island] = entry
        location, _, local, _ = entry[0].find_nearest(point)
        return location, entry[1][local]

    def corner_on(face_idx, point):
        hpoly = hpolys[face_idx]
        pverts = hpoly.vertices
        ploops = hpoly.loop_indices
        best = None
        best_d2 = float("inf")
        for k in range(1, len(pverts) - 1):
            tri = (0, k, k + 1)
            a, b, c = (hverts[pverts[t]] for t in tri)
            w = _bary_weights_3d(point, a, b, c)
            d2 = (point - (a * w[0] + b * w[1] + c * w[2])).length_squared
            if d2 < best_d2:
                best_d2 = d2
                best = (tuple(ploops[t] for t in tri),
                        tuple(pverts[t] for t in tri), w)
        return best

    low_world = low.matrix_world
    low_mesh = low.data
    items = []
    projected = 0
    skipped = 0
    for poly in low_mesh.polygons:
        _, _, anchor, dist = tree.find_nearest(low_world @ poly.center)
        if anchor is None or dist > max_dist:
            skipped += 1
            continue
        corners = []
        for li in poly.loop_indices:
            point = low_world @ low_mesh.vertices[
                low_mesh.loops[li].vertex_index].co
            location, _, face_idx, _ = tree.find_nearest(point)
            if (island_of is not None
                    and island_of[face_idx] != island_of[anchor]):
                location, face_idx = nearest_in_island(
                    island_of[anchor], point)
            corners.append((li,) + corner_on(face_idx, location))
        items.append((poly, corners))
        projected += 1
    return items, projected, skipped


class RETOPOFORGE_OT_project_uvs(bpy.types.Operator):
    """Copy HIGH UVs onto the LOW mesh by nearest-point projection
    (each LOW face stays inside one HIGH UV island, so seams survive;
    faces beyond range keep their UVs)"""

    bl_idname = "retopoforge.project_uvs"
    bl_label = "Project HIGH UVs"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        params = context.scene.retopoforge_params
        high = context.active_object
        lows = [o for o in context.selected_objects
                if o is not high and o.type == "MESH"]
        if high is None or high.type != "MESH" or not lows:
            self.report({"ERROR"},
                        "Select LOW meshes with the HIGH-poly source active")
            return {"CANCELLED"}
        low = lows[0]
        high_uv = high.data.uv_layers.active
        if high_uv is None:
            self.report({"ERROR"},
                        f"HIGH '{high.name}' has no UVs to project")
            return {"CANCELLED"}
        low_uv = low.data.uv_layers.active
        if low_uv is None:
            low_uv = low.data.uv_layers.new(name="Projected")
        items, projected, skipped = _face_correspondence(
            high, low, params.project_uv_max_dist,
            island_of=_uv_islands(high.data, high_uv))
        huv = high_uv.uv
        for _, corners in items:
            for li, (la, lb, lc), _, (w0, w1, w2) in corners:
                low_uv.uv[li].vector = (huv[la].vector * w0
                                        + huv[lb].vector * w1
                                        + huv[lc].vector * w2)
        line = (f"{low.name}: projected UVs on {projected}/"
                f"{projected + skipped} faces ({skipped} beyond range)")
        context.scene.retopoforge_last_report += line + "\n"
        self.report({"INFO"}, line)
        return {"FINISHED"}


class RETOPOFORGE_OT_transfer_colors(bpy.types.Operator):
    """Copy the HIGH active vertex-color layer onto LOW by
    nearest-point projection (for non-textured AI outputs; faces
    beyond range keep the fill color)"""

    bl_idname = "retopoforge.transfer_colors"
    bl_label = "Transfer HIGH Colors"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        params = context.scene.retopoforge_params
        high = context.active_object
        lows = [o for o in context.selected_objects
                if o is not high and o.type == "MESH"]
        if high is None or high.type != "MESH" or not lows:
            self.report({"ERROR"},
                        "Select LOW meshes with the HIGH-poly source active")
            return {"CANCELLED"}
        low = lows[0]
        high_col = high.data.color_attributes.active_color
        if high_col is None:
            self.report({"ERROR"},
                        f"HIGH '{high.name}' has no color attribute")
            return {"CANCELLED"}
        low_col = low.data.color_attributes.new(
            high_col.name, "FLOAT_COLOR", "CORNER")
        low.data.color_attributes.active_color = low_col
        items, projected, skipped = _face_correspondence(
            high, low, params.transfer_max_dist)
        by_point = high_col.domain == "POINT"
        hdata = high_col.data
        for _, corners in items:
            for li, hloops, hvs, (w0, w1, w2) in corners:
                ia, ib, ic = hvs if by_point else hloops
                a, b, c = hdata[ia].color, hdata[ib].color, hdata[ic].color
                low_col.data[li].color = tuple(
                    a[k] * w0 + b[k] * w1 + c[k] * w2 for k in range(4))
        line = (f"{low.name}: transferred colors on {projected}/"
                f"{projected + skipped} faces ({skipped} beyond range)")
        context.scene.retopoforge_last_report += line + "\n"
        self.report({"INFO"}, line)
        return {"FINISHED"}


class RETOPOFORGE_OT_remesh_and_bake(bpy.types.Operator):
    """Remesh the active HIGH object, then Smart-UV + bake all PBR maps
    to the result in one action"""

    bl_idname = "retopoforge.remesh_and_bake"
    bl_label = "Remesh + Bake All"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        high = context.active_object
        if high is None or high.type != "MESH":
            self.report({"ERROR"},
                        "Make the HIGH-poly mesh the active object")
            return {"CANCELLED"}
        # Chained bake needs the HIGH mesh intact: force keep-original for
        # the remesh leg via the operator, never via the scene params (the
        # remesh leg's settings recall would overwrite a param, and the
        # forced value would then be saved as the user's choice).
        before = set(bpy.data.objects)
        for o in context.selected_objects:
            o.select_set(False)
        high.select_set(True)
        context.view_layer.objects.active = high
        result = bpy.ops.retopoforge.remesh(force_keep_original=True)
        if "FINISHED" not in result:
            self.report({"ERROR"}, "Remesh leg failed, bake skipped")
            return {"CANCELLED"}
        fresh = [o for o in bpy.data.objects
                 if o not in before and o.type == "MESH"]
        if not fresh:
            self.report({"ERROR"}, "Remesh produced no new mesh object")
            return {"CANCELLED"}
        low = fresh[0]
        # The remesh leg hides HIGH; the chained bake raycasts it,
        # so unhide first (viewport-hidden objects bake black).
        high.hide_viewport = False
        for o in context.selected_objects:
            o.select_set(False)
        low.select_set(True)
        high.select_set(True)
        context.view_layer.objects.active = high
        result = bpy.ops.retopoforge.bake_textures(ignore_cage=True)
        if "FINISHED" not in result:
            self.report({"ERROR"}, "Bake leg failed")
            return {"CANCELLED"}
        self.report({"INFO"}, f"Remeshed + baked '{high.name}'")
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
        col.prop(params, "compute_uvs")
        col.prop(params, "apply_modifiers")
        col.prop(params, "keep_original")
        col.prop(params, "symmetry_enabled")
        sym_row = col.row()
        sym_row.enabled = params.symmetry_enabled
        sym_row.prop(params, "symmetry_plane")
        report = context.scene.retopoforge_last_report
        if report:
            box = layout.box()
            for line in report.strip().split("\n"):
                box.label(text=line)
        layout.label(text="Remeshing replaces topology;", icon="INFO")
        layout.label(text="Original UVs do not survive.")
        # Dev convenience: picks up extension updates without restarting
        # Blender, with an INFO report as visible confirmation.
        layout.operator("retopoforge.reload_scripts", text="Reload Scripts",
                        icon="FILE_REFRESH")


class RETOPOFORGE_PT_advanced(bpy.types.Panel):
    bl_label = "Advanced"
    bl_idname = "RETOPOFORGE_PT_advanced"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "RetopoForge"
    bl_parent_id = "RETOPOFORGE_PT_panel"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        params = context.scene.retopoforge_params
        col = self.layout.column(align=True)
        col.prop(params, "smooth_normal")
        col.prop(params, "edge_scaling")
        col.prop(params, "adaptivity")
        col.prop(params, "anisotropy")


class RETOPOFORGE_PT_guides(bpy.types.Panel):
    bl_label = "Flow Guides"
    bl_idname = "RETOPOFORGE_PT_guides"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "RetopoForge"
    bl_parent_id = "RETOPOFORGE_PT_panel"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        params = context.scene.retopoforge_params
        layout = self.layout
        layout.label(text="Edge selection, per target")
        layout.prop(params, "guides_enabled")
        layout.operator("retopoforge.export_guides",
                        text="Export Guide Strokes", icon="GREASEPENCIL")


class RETOPOFORGE_PT_features(bpy.types.Panel):
    bl_label = "Sharp Features"
    bl_idname = "RETOPOFORGE_PT_features"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "RetopoForge"
    bl_parent_id = "RETOPOFORGE_PT_panel"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        params = context.scene.retopoforge_params
        layout = self.layout
        layout.label(text="Marked edges, per target")
        layout.prop(params, "features_enabled")
        layout.operator("retopoforge.export_features",
                        text="Export Sharp Features", icon="EDGESEL")


class RETOPOFORGE_PT_density(bpy.types.Panel):
    bl_label = "Density Mask"
    bl_idname = "RETOPOFORGE_PT_density"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "RetopoForge"
    bl_parent_id = "RETOPOFORGE_PT_panel"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        params = context.scene.retopoforge_params
        layout = self.layout
        layout.label(text="Vertex group weights")
        layout.prop(params, "density_enabled")
        dcol = layout.column(align=True)
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
        layout.operator("retopoforge.export_density",
                        text="Export Density Mask", icon="GROUP_VERTEX")


class RETOPOFORGE_PT_lods(bpy.types.Panel):
    bl_label = "LODs"
    bl_idname = "RETOPOFORGE_PT_lods"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "RetopoForge"
    bl_parent_id = "RETOPOFORGE_PT_panel"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        params = context.scene.retopoforge_params
        layout = self.layout
        layout.prop(params, "lod_targets")
        layout.operator("retopoforge.generate_lods", text="Generate LODs",
                        icon="MOD_DECIM")


class RETOPOFORGE_PT_bake(bpy.types.Panel):
    bl_label = "Bake Assist"
    bl_idname = "RETOPOFORGE_PT_bake"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "RetopoForge"
    bl_parent_id = "RETOPOFORGE_PT_panel"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        params = context.scene.retopoforge_params
        layout = self.layout
        layout.label(text="Active = HIGH, Selected = LOW")
        bcol = layout.column(align=True)
        bcol.prop(params, "bake_size")
        bcol.prop(params, "bake_extrusion")
        bcol.prop(params, "bake_margin")
        bcol.prop(params, "bake_cage")
        bcol.prop(params, "bake_normal")
        bcol.prop(params, "bake_roughness")
        bcol.prop(params, "bake_metallic")
        bcol.prop(params, "bake_ao")
        bcol.prop(params, "bake_emission")
        bcol.separator()
        bcol.prop(params, "bake_uv_mode")
        bcol.prop(params, "bake_unwrap_method")
        bcol.prop(params, "bake_pack_margin")
        bcol.prop(params, "bake_average_scale")
        bcol.prop(params, "bake_texel_density")
        layout.operator("retopoforge.bake_textures", text="Bake High to Low",
                        icon="RENDER_RESULT")
        layout.operator("retopoforge.remesh_and_bake", text="Remesh + Bake All",
                        icon="PLAY")
        bcol.prop(params, "project_uv_max_dist")
        layout.operator("retopoforge.project_uvs", text="Project HIGH UVs",
                        icon="UV")
        bcol.prop(params, "transfer_max_dist")
        layout.operator("retopoforge.transfer_colors",
                        text="Transfer HIGH Colors", icon="GROUP_VCOL")


_CLASSES = (
    RetopoForgePreferences,
    RETOPOFORGE_PG_params,
    RETOPOFORGE_OT_remesh,
    RETOPOFORGE_OT_generate_lods,
    RETOPOFORGE_OT_bake_textures,
    RETOPOFORGE_OT_remesh_and_bake,
    RETOPOFORGE_OT_project_uvs,
    RETOPOFORGE_OT_transfer_colors,
    RETOPOFORGE_OT_export_guides,
    RETOPOFORGE_OT_export_features,
    RETOPOFORGE_OT_export_density,
    RETOPOFORGE_OT_reload,
    RETOPOFORGE_PT_panel,
    RETOPOFORGE_PT_advanced,
    RETOPOFORGE_PT_guides,
    RETOPOFORGE_PT_features,
    RETOPOFORGE_PT_density,
    RETOPOFORGE_PT_lods,
    RETOPOFORGE_PT_bake,
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
