# Headless test: the genforge repair-topology adapter (tools/genforge_adapter.py).
#
#   /Applications/Blender.app/Contents/MacOS/Blender --background \
#       --factory-startup --python blender/tests/test_genforge_adapter.py
#
# A dense skinned, textured, animated sphere-on-a-stick (over the hero
# budget, 2048 px texture, no normal map) goes in. The adapter must
# rebuild it within budget, keep the skeleton and the clip, bake color +
# normal at 1024 px, keep every vertex skinned, and write genforge's
# result.json (exit 0). Error paths return 2 with no result.
# Needs the `retopo` CLI (PATH or rust/target); exits 0 with SKIP without it.

import importlib.util
import json
import os
import shutil
import struct
import sys
import tempfile

import bpy

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))


def check(condition, message):
    print(("PASS" if condition else "FAIL") + ": " + message)
    if not condition:
        raise SystemExit(1)


def glb_json(path):
    with open(path, "rb") as f:
        data = f.read()
    (length,) = struct.unpack_from("<I", data, 12)
    return json.loads(data[20 : 20 + length]), data


def build_fixture(path):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=160, ring_count=80, radius=0.5, location=(0, 0, 1.0))
    body = bpy.context.active_object
    body.name = "Body"
    body.data.name = "Body"
    bpy.ops.object.shade_smooth()
    arm_data = bpy.data.armatures.new("Rig")
    arm = bpy.data.objects.new("Rig", arm_data)
    bpy.context.scene.collection.objects.link(arm)
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    lower = arm_data.edit_bones.new("DEF-lower")
    lower.head, lower.tail = (0, 0, 0.5), (0, 0, 1.0)
    upper = arm_data.edit_bones.new("DEF-upper")
    upper.head, upper.tail = (0, 0, 1.0), (0, 0, 1.5)
    upper.parent = lower
    bpy.ops.object.mode_set(mode="OBJECT")
    g_lo = body.vertex_groups.new(name="DEF-lower")
    g_up = body.vertex_groups.new(name="DEF-upper")
    for v in body.data.vertices:
        z = (body.matrix_world @ v.co).z
        t = min(1.0, max(0.0, (z - 0.7) / 0.6))
        g_lo.add([v.index], 1.0 - t, "REPLACE")
        g_up.add([v.index], t, "REPLACE")
    body.parent = arm
    mod = body.modifiers.new("Armature", "ARMATURE")
    mod.object = arm
    img = bpy.data.images.new("checker", 2048, 2048, alpha=False)
    img.generated_type = "COLOR_GRID"
    mat = bpy.data.materials.new("Skin")
    mat.use_nodes = True
    tex = mat.node_tree.nodes.new("ShaderNodeTexImage")
    tex.image = img
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    mat.node_tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    body.data.materials.append(mat)
    # One clip: bend the upper bone.
    arm.animation_data_create()
    action = bpy.data.actions.new("bend")
    arm.animation_data.action = action
    pb = arm.pose.bones["DEF-upper"]
    pb.rotation_mode = "XYZ"
    for frame, angle in ((1, 0.0), (20, 0.8)):
        pb.rotation_euler = (angle, 0, 0)
        pb.keyframe_insert("rotation_euler", frame=frame)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.export_scene.gltf(filepath=path, export_format="GLB", export_animations=True)


def main():
    sys.path.insert(0, os.path.join(REPO, "blender"))
    import retopoforge

    if not retopoforge.find_retopo_binary(""):
        print("SKIP: retopo CLI not found")
        return
    spec = importlib.util.spec_from_file_location(
        "genforge_adapter", os.path.join(REPO, "tools", "genforge_adapter.py")
    )
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)

    work = tempfile.mkdtemp(prefix="retopoforge_genforge_")
    try:
        src = os.path.join(work, "dense.glb")
        build_fixture(src)
        before = adapter.glb_counts(src)
        check(before["Body"][0] > 10000, f"fixture over the hero vertex budget {before}")
        step = os.path.join(work, "09-fix")
        os.makedirs(step)
        out = os.path.join(step, "output.glb")
        result = os.path.join(step, "result.json")
        code, payload = adapter.run_adapter(["repair-topology", src, out, result, "--class", "humanoid"])
        check(code == 0, f"adapter exit 0 (got {code}: {payload and payload.get('reason')})")
        with open(result, encoding="utf-8") as f:
            written = json.load(f)
        check(written["ok"] is True and written["tool"] == "retopoforge", "result ok, tool name")
        check(written["outputs"] == ["output.glb"], f"outputs relative ({written['outputs']})")
        verts, tris = adapter.glb_counts(out)["Body"]
        check(verts <= 10000 and tris <= 15000, f"within hero budget ({verts} verts, {tris} tris)")
        doc, data = glb_json(out)
        joints = sorted(doc["nodes"][j]["name"] for j in doc["skins"][0]["joints"])
        check(joints == ["DEF-lower", "DEF-upper"], f"skeleton kept ({joints})")
        check(len(doc.get("animations", [])) == 1, "clip kept")
        mats = doc.get("materials", [])
        check(mats and "normalTexture" in mats[0], "normal map baked")
        check("baseColorTexture" in mats[0].get("pbrMetallicRoughness", {}), "color map baked")
        # Every vertex still skinned (weights sum to 1).
        prim = doc["meshes"][0]["primitives"][0]
        acc = doc["accessors"][prim["attributes"]["WEIGHTS_0"]]
        view = doc["bufferViews"][acc["bufferView"]]
        (json_len,) = struct.unpack_from("<I", data, 12)
        bin_start = 20 + json_len + 8
        off = bin_start + view.get("byteOffset", 0) + acc.get("byteOffset", 0)
        check(acc["componentType"] == 5126, "float weights")
        sums = [sum(struct.unpack_from("<4f", data, off + 16 * i)) for i in range(acc["count"])]
        check(min(sums) > 0.999 and max(sums) < 1.001, f"weights normalized ({min(sums):.4f}..{max(sums):.4f})")
        # Error paths: no result written.
        for argv in (
            ["repair-topology", os.path.join(work, "missing.glb"), out, os.path.join(work, "e1.json")],
            ["explode", src, out, os.path.join(work, "e2.json")],
            ["repair-topology", src, out, os.path.join(work, "e3.json"), "--class", "dragon"],
        ):
            code, payload = adapter.run_adapter(argv)
            check(code == 2 and payload is None, f"{argv[0]} {argv[-1]}: exit 2, no result")
        check(not any(os.path.exists(os.path.join(work, f"e{i}.json")) for i in (1, 2, 3)), "no stray results")
        print("RETOPOFORGE_GENFORGE_ADAPTER_OK")
    finally:
        shutil.rmtree(work, ignore_errors=True)


main()
