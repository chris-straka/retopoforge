# SPDX-License-Identifier: GPL-3.0-or-later
"""genforge adapter: repair-topology (rebuild a character mesh to budget).

genforge's character chain (genforge docs/phase-7.md) calls this when its
check stage finds the mesh over the class budget (P_VERTS, P_TRIS) or
the textures off-budget (P_TEX_SIZE, P_TEX_NORMAL). Contract:

    tools/genforge_adapter.sh repair-topology IN.glb OUT.glb RESULT.json \\
        [--class humanoid|quadruped|custom]

(the .sh wrapper runs this inside headless Blender with the extension
from this checkout and the installed `retopo` CLI).

Per skinned mesh that is over budget or lacks a normal map: the
extension's Remesh + Bake All builds a quad mesh (`retopo`, target from
the budget) and bakes the original's color and normal onto fresh UVs at
the budget texture size; the skin weights are transferred from the
original (nearest face, max 4 influences, normalized) and the new mesh
keeps the original's armature, so bone names, hierarchy and animations
are untouched. Meshes already within budget pass through. Textures over
the budget on untouched meshes are scaled down.

Budgets follow rfcheck's classes: humanoid -> hero (10,000 verts,
15,000 tris, 1024 px), quadruped/custom -> monster (12,000 / 20,000 /
1024). Counts are glTF vertices (split at UV seams), read back from the
exported file.

RESULT.json (same schema as the weightforge/motionforge/rigforge
adapters): {"ok": bool, "outputs": [relative paths], "tool":
"retopoforge", "stage": "repair-topology", "repair-topology": {...},
"reason"?}. Exit 0 = rebuilt within budget; 1 = rebuilt but still over
budget, or a remesh/bake step failed (result written); 2 = error (bad
args, unreadable input, no retopo binary): no result file.
"""

import json
import os
import struct
import sys

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
BUDGETS = {
    "hero": {"verts": 10000, "tris": 15000, "tex": 1024},
    "monster": {"verts": 12000, "tris": 20000, "tex": 1024},
}
CLASS_BUDGET = {"humanoid": "hero", "quadruped": "monster", "custom": "monster"}


class AdapterError(Exception):
    """Exit-2 errors: no result.json is written."""


def parse(argv):
    if len(argv) < 4:
        raise AdapterError(
            "usage: genforge_adapter repair-topology IN.glb OUT.glb RESULT.json "
            "[--class humanoid|quadruped|custom]"
        )
    stage, src, out, result = argv[:4]
    if stage != "repair-topology":
        raise AdapterError(f"unknown adapter stage {stage!r} (retopoforge has repair-topology)")
    klass = "humanoid"
    rest = argv[4:]
    i = 0
    while i < len(rest):
        if rest[i] == "--class" and i + 1 < len(rest):
            klass = rest[i + 1]
            i += 2
        else:
            raise AdapterError(f"unknown flag {rest[i]!r}")
    if klass not in CLASS_BUDGET:
        raise AdapterError(f"--class must be humanoid|quadruped|custom, got {klass!r}")
    if not os.path.isfile(src):
        raise AdapterError(f"input not found: {src}")
    return src, out, result, klass


def glb_counts(path):
    """{mesh name: (verts, tris)} from a GLB's accessors (rfcheck's way)."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != b"glTF":
        raise AdapterError(f"{path} is not a GLB")
    (length,) = struct.unpack_from("<I", data, 12)
    doc = json.loads(data[20 : 20 + length])
    accessors = doc.get("accessors", [])
    counts = {}
    for mesh in doc.get("meshes", []):
        verts = tris = 0
        for prim in mesh.get("primitives", []):
            pos = prim.get("attributes", {}).get("POSITION")
            if pos is not None:
                verts += accessors[pos]["count"]
            if prim.get("mode", 4) == 4:
                idx = prim.get("indices")
                n = accessors[idx]["count"] if idx is not None else (accessors[pos]["count"] if pos is not None else 0)
                tris += n // 3
        counts[mesh.get("name", "?")] = (verts, tris)
    return counts


def max_image_size(obj):
    biggest = 0
    for slot in obj.material_slots:
        mat = slot.material
        if mat is None or not mat.use_nodes:
            continue
        for node in mat.node_tree.nodes:
            if node.type == "TEX_IMAGE" and node.image is not None:
                biggest = max(biggest, *node.image.size)
    return biggest


def has_normal_map(obj):
    for slot in obj.material_slots:
        mat = slot.material
        if mat is None or not mat.use_nodes:
            continue
        if any(n.type == "NORMAL_MAP" for n in mat.node_tree.nodes):
            return True
    return False


def select_only(objs, active):
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for o in bpy.context.selected_objects:
        o.select_set(False)
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = active


def baked_material(low):
    """Fresh glTF-friendly material: baked color + tangent normal map."""
    diffuse = bpy.data.images.get(f"{low.name}_diffuse")
    normal = bpy.data.images.get(f"{low.name}_normal")
    if diffuse is None:
        raise RuntimeError(f"bake produced no color map for {low.name}")
    mat = bpy.data.materials.new(f"{low.name}_baked")
    mat.use_nodes = True
    tree = mat.node_tree
    bsdf = next(n for n in tree.nodes if n.type == "BSDF_PRINCIPLED")
    tex = tree.nodes.new("ShaderNodeTexImage")
    tex.image = diffuse
    tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    if normal is not None:
        normal.colorspace_settings.name = "Non-Color"
        ntex = tree.nodes.new("ShaderNodeTexImage")
        ntex.image = normal
        nmap = tree.nodes.new("ShaderNodeNormalMap")
        tree.links.new(ntex.outputs["Color"], nmap.inputs["Color"])
        tree.links.new(nmap.outputs["Normal"], bsdf.inputs["Normal"])
    for img in (diffuse, normal):
        if img is not None:
            img.pack()
    low.data.materials.clear()
    low.data.materials.append(mat)
    return mat


def rebuild(high, budget, report):
    """Remesh + bake one skinned mesh; returns the new object."""
    params = bpy.context.scene.retopoforge_params
    # Quads ~ verts; leave room for UV-seam splits and the tri budget.
    target = int(min(budget["verts"] * 0.7, budget["tris"] * 0.45))
    params.target_quads = target
    params.apply_modifiers = False  # rest-pose mesh data, not the posed deform
    params.keep_original = False
    params.bake_size = budget["tex"]
    params.bake_normal = True
    params.bake_roughness = False
    params.bake_metallic = False
    params.bake_ao = False
    params.bake_emission = False
    # Smart UV project: a plain unwrap of a seamless closed body
    # collapses into one degenerate island.
    params.bake_uv_mode = "SMART"
    # Remesh + Bake All, split in two so the new mesh can be cleaned
    # before the bake (same calls as retopoforge.remesh_and_bake).
    before = set(bpy.data.objects)
    select_only([high], high)
    result = bpy.ops.retopoforge.remesh(force_keep_original=True)
    if "FINISHED" not in result:
        raise RuntimeError(f"remesh failed on {high.name}")
    fresh = [o for o in bpy.data.objects if o not in before and o.type == "MESH"]
    if not fresh:
        raise RuntimeError(f"remesh produced no mesh for {high.name}")
    low = fresh[0]
    # retopo's OBJ comes back flat-shaded and not always consistently
    # wound: smooth it (glTF would split every corner otherwise) and
    # point every face outward (inward faces make the bake rays miss).
    for poly in low.data.polygons:
        poly.use_smooth = True
    select_only([low], low)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    # A handful of zero-area faces survive extraction; drop them.
    bpy.ops.mesh.dissolve_degenerate(threshold=1e-5)
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode="OBJECT")
    high.hide_viewport = False
    select_only([high, low], high)
    result = bpy.ops.retopoforge.bake_textures(ignore_cage=True)
    if "FINISHED" not in result:
        raise RuntimeError(f"bake failed on {high.name}")
    # Weights: nearest-face interpolation from the original, then the
    # mobile rig rules (4 influences, normalized).
    low.vertex_groups.clear()
    for group in high.vertex_groups:
        low.vertex_groups.new(name=group.name)
    select_only([high, low], high)
    bpy.ops.object.data_transfer(
        use_reverse_transfer=False,
        data_type="VGROUP_WEIGHTS",
        vert_mapping="POLYINTERP_NEAREST",
        layers_select_src="ALL",
        layers_select_dst="NAME",
        mix_mode="REPLACE",
    )
    select_only([low], low)
    bpy.ops.object.vertex_group_limit_total(group_select_mode="ALL", limit=4)
    bpy.ops.object.vertex_group_normalize_all(group_select_mode="ALL", lock_active=False)
    baked_material(low)
    name = high.name
    report.append(
        {
            "mesh": name,
            "target_quads": target,
            "quads": sum(1 for p in low.data.polygons if len(p.vertices) == 4),
            "faces": len(low.data.polygons),
            "groups": len(low.vertex_groups),
        }
    )
    mesh = high.data
    bpy.data.objects.remove(high, do_unlink=True)
    if mesh.users == 0:
        bpy.data.meshes.remove(mesh)
    low.name = name
    low.data.name = name
    return low


def write_result(path, payload):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")


def relative(path, base):
    rel = os.path.relpath(os.path.realpath(path), os.path.realpath(base))
    return os.path.realpath(path) if rel.startswith("..") else rel


def run_adapter(argv):
    """Work in this Blender session; returns (exit code, payload or None)."""
    try:
        src, out, result, klass = parse(argv)
        before_counts = glb_counts(src)
    except AdapterError as exc:
        print(f"genforge_adapter: {exc}", file=sys.stderr)
        return 2, None
    budget_name = CLASS_BUDGET[klass]
    budget = BUDGETS[budget_name]
    if REPO + "/blender" not in sys.path:
        sys.path.insert(0, os.path.join(REPO, "blender"))
    bpy.ops.wm.read_factory_settings(use_empty=True)
    try:
        bpy.ops.preferences.addon_enable(module="retopoforge")
    except Exception as exc:  # noqa: BLE001
        print(f"genforge_adapter: cannot enable retopoforge: {exc}", file=sys.stderr)
        return 2, None
    import retopoforge

    if not retopoforge.find_retopo_binary(""):
        print("genforge_adapter: retopo CLI not found (PATH or rust/target)", file=sys.stderr)
        return 2, None
    try:
        # No bind-pose guessing: with it, a skeleton under a translated
        # root node comes back lifted by that translation on export.
        bpy.ops.import_scene.gltf(filepath=src, guess_original_bind_pose=False)
    except RuntimeError as exc:
        print(f"genforge_adapter: cannot import {src}: {exc}", file=sys.stderr)
        return 2, None
    arms = [o for o in bpy.data.objects if o.type == "ARMATURE"]
    for arm in arms:
        arm.data.pose_position = "REST"
    meshes = [o for o in bpy.data.objects if o.type == "MESH" and o.visible_get()]
    rebuilt = []
    scaled = []
    try:
        for obj in meshes:
            verts, tris = before_counts.get(obj.data.name, before_counts.get(obj.name, (0, 0)))
            over = verts > budget["verts"] or tris > budget["tris"]
            needs_normal = obj.find_armature() is not None and not has_normal_map(obj)
            if obj.find_armature() is not None and (over or needs_normal):
                rebuild(obj, budget, rebuilt)
            elif max_image_size(obj) > budget["tex"]:
                for slot in obj.material_slots:
                    if slot.material and slot.material.use_nodes:
                        for node in slot.material.node_tree.nodes:
                            if node.type == "TEX_IMAGE" and node.image and max(node.image.size) > budget["tex"]:
                                w, h = node.image.size
                                k = budget["tex"] / max(w, h)
                                node.image.scale(max(1, int(w * k)), max(1, int(h * k)))
                                scaled.append(node.image.name)
    except RuntimeError as exc:
        payload = {
            "ok": False,
            "outputs": [],
            "tool": "retopoforge",
            "stage": "repair-topology",
            "reason": str(exc),
            "repair-topology": {"budget": budget_name, "rebuilt": rebuilt},
        }
        write_result(result, payload)
        return 1, payload
    for arm in arms:
        arm.data.pose_position = "POSE"
    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.export_scene.gltf(
        filepath=out,
        export_format="GLB",
        use_visible=True,
        export_animations=True,
        export_skins=True,
        export_image_format="AUTO",
    )
    after = glb_counts(out)
    over = {
        name: {"verts": v, "tris": t}
        for name, (v, t) in after.items()
        if v > budget["verts"] or t > budget["tris"]
    }
    ok = not over
    details = {
        "budget": budget_name,
        "limits": budget,
        "before": {k: {"verts": v, "tris": t} for k, (v, t) in before_counts.items()},
        "after": {k: {"verts": v, "tris": t} for k, (v, t) in after.items()},
        "rebuilt": rebuilt,
        "scaled_images": scaled,
        "retopo": retopoforge.find_retopo_binary(""),
    }
    payload = {
        "ok": ok,
        "outputs": [relative(out, os.path.dirname(os.path.abspath(result)))],
        "tool": "retopoforge",
        "stage": "repair-topology",
        "repair-topology": details,
    }
    if not ok:
        payload["reason"] = f"still over the {budget_name} budget: {sorted(over)}"
    write_result(result, payload)
    return (0 if ok else 1), payload


if __name__ == "__main__":
    args = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    code, _payload = run_adapter(args)
    sys.stdout.flush()
    sys.exit(code)
