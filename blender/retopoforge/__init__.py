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
        self._tmpdir = tempfile.mkdtemp(prefix="retopoforge_")
        context.scene.retopoforge_last_report = ""
        try:
            for index, obj in enumerate(targets):
                matrix = obj.matrix_world.copy()
                try:
                    input_path = self._export_job(context, obj, index)
                    output_path = os.path.join(self._tmpdir, f"out_{index}.obj")
                    proc = subprocess.run(
                        self._params.cli_args(binary, input_path, output_path),
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
        self._tmpdir = tempfile.mkdtemp(prefix="retopoforge_")
        self._queue = [{"obj": o, "matrix": o.matrix_world.copy()} for o in targets]
        self._total = len(self._queue)
        self._proc = None
        self._current = None
        self._output_path = ""
        context.scene.retopoforge_last_report = ""
        wm = context.window_manager
        wm.progress_begin(0, self._total)
        self._timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        self._start_next(context)
        return {"RUNNING_MODAL"}

    def _start_next(self, context):
        self._current = self._queue.pop(0)
        obj = self._current["obj"]
        index = self._total - len(self._queue) - 1
        input_path = self._export_job(context, obj, index)
        self._output_path = os.path.join(self._tmpdir, f"out_{index}.obj")
        self._proc = subprocess.Popen(
            self._params.cli_args(self._binary, input_path, self._output_path),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        context.window_manager.progress_update(index)

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
        stdout_text, stderr_text = self._proc.communicate()
        if self._proc.returncode != 0:
            tail = (stderr_text or "").strip().splitlines()
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
            self._start_next(context)
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
        report = context.scene.retopoforge_last_report
        if report:
            box = layout.box()
            for line in report.strip().split("\n"):
                box.label(text=line)
        layout.label(text="Remeshing replaces topology;", icon="INFO")
        layout.label(text="UVs and vertex colors do not survive.")
        # Dev convenience: picks up extension updates without restarting
        # Blender (same operator as F3 > Reload Scripts).
        layout.operator("script.reload", text="Reload Scripts",
                        icon="FILE_REFRESH")


_CLASSES = (
    RetopoForgePreferences,
    RETOPOFORGE_PG_params,
    RETOPOFORGE_OT_remesh,
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


def unregister():
    del bpy.types.Scene.retopoforge_last_report
    del bpy.types.Scene.retopoforge_params
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
