#!/usr/bin/env python3
"""Headless Blender worker: voxel-remesh one OBJ to roughly match a quad target.

Invoked ONLY via Blender in background mode (never the GUI):

    Blender --background --factory-startup --python bench/blender_voxel.py \
        -- <input.obj> <output.obj> <target-quads>

Steps: import OBJ, estimate a voxel size from the bounding-box diagonal,
calibrate with one trial remesh on a scratch copy, remesh the original with
the adjusted size, apply, export OBJ, print a STATS line.

Calibrated rather than one-shot because surface-area/diagonal^2 varies
widely per model; one trial remesh inside the same process keeps it to a
single Blender startup per case.
"""

import math
import sys

try:
    import bpy
    from mathutils import Vector
except ImportError:
    print(
        "blender_voxel.py must run inside Blender "
        "(--background --python bench/blender_voxel.py -- ...)",
        file=sys.stderr,
    )
    sys.exit(2)


def bbox_diagonal(obj):
    corners = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    lo = [min(c[i] for c in corners) for i in range(3)]
    hi = [max(c[i] for c in corners) for i in range(3)]
    return math.sqrt(sum((h - l) ** 2 for h, l in zip(hi, lo)))


def voxel_remesh(obj, voxel_size):
    """Add + apply a VOXEL REMESH modifier on obj. Returns polygon count."""
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    mod = obj.modifiers.new("VoxelRemesh", "REMESH")
    mod.mode = "VOXEL"
    mod.voxel_size = voxel_size
    mod.adaptivity = 0.0
    bpy.ops.object.modifier_apply(modifier=mod.name)
    obj.select_set(False)
    return len(obj.data.polygons)


def main(argv):
    input_path, output_path, target = argv[0], argv[1], int(argv[2])

    # Clean default scene.
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)

    bpy.ops.wm.obj_import(filepath=input_path)
    obj = bpy.context.selected_objects[0]
    bpy.context.view_layer.objects.active = obj

    diag = bbox_diagonal(obj)
    if diag <= 0:
        print("ERROR: degenerate bounding box", flush=True)
        return 1

    # Trial remesh on a scratch copy, then scale voxel size by
    # sqrt(actual/target) since faces ~= area / voxel_size^2.
    guess = diag * 0.5 / math.sqrt(target)
    guess = min(max(guess, diag / 1000.0), diag / 5.0)
    trial = obj.copy()
    trial.data = obj.data.copy()
    bpy.context.collection.objects.link(trial)
    trial_faces = voxel_remesh(trial, guess)
    bpy.data.objects.remove(trial, do_unlink=True)

    if trial_faces > 0:
        final_voxel = guess * math.sqrt(trial_faces / target)
    else:
        final_voxel = guess
    final_voxel = min(max(final_voxel, diag / 2000.0), diag / 2.0)

    final_faces = voxel_remesh(obj, final_voxel)

    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.wm.obj_export(
        filepath=output_path,
        check_existing=False,
        export_selected_objects=True,
        export_materials=False,
    )
    mesh = obj.data
    verts = len(mesh.vertices)
    quads = sum(1 for p in mesh.polygons if len(p.vertices) == 4)
    print(
        f"STATS quads={quads} non_quads={final_faces - quads} verts={verts} "
        f"voxel={final_voxel:.6g} trial_faces={trial_faces}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if len(args) != 3:
        print("usage: blender_voxel.py -- <input.obj> <output.obj> <target>",
              file=sys.stderr)
        sys.exit(2)
    sys.exit(main(args))
