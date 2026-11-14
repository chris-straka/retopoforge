#!/usr/bin/env python3
"""Copy a clip library's animations onto a rigged GLB with the same bone
names (stdlib only).

    bench/creature/clips_onto.py RIGGED.glb CLIPS.glb OUT.glb [--only a,b,c]

The game's clips for the standard humanoid skeleton live in one
skeleton-only GLB; every character on that skeleton plays them as they
are. This copies each clip's rotation channels onto RIGGED's bones by
node name (translation and scale are left out: the body stays in place,
which is all a skin check needs), so `weights check` can sample the
candidate on the game's own motion. Channels for bones RIGGED lacks are
dropped; the count kept and dropped is printed.
"""
import json
import struct
import sys


def read_glb(path):
    b = open(path, "rb").read()
    assert b[:4] == b"glTF", path
    n = struct.unpack("<I", b[12:16])[0]
    js = json.loads(b[20:20 + n])
    rest = b[20 + n:]
    bin_ = b""
    if len(rest) >= 8:
        m = struct.unpack("<I", rest[:4])[0]
        bin_ = rest[8:8 + m]
    return js, bytearray(bin_)


def write_glb(path, js, bin_):
    j = json.dumps(js, separators=(",", ":")).encode()
    j += b" " * (-len(j) % 4)
    bin_ = bytes(bin_) + b"\0" * (-len(bin_) % 4)
    total = 12 + 8 + len(j) + 8 + len(bin_)
    with open(path, "wb") as f:
        f.write(b"glTF" + struct.pack("<II", 2, total))
        f.write(struct.pack("<I", len(j)) + b"JSON" + j)
        f.write(struct.pack("<I", len(bin_)) + b"BIN\0" + bin_)


SIZES = {5120: 1, 5121: 1, 5122: 2, 5123: 2, 5125: 4, 5126: 4}
COUNTS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def main():
    a = sys.argv[1:]
    only = None
    if "--only" in a:
        i = a.index("--only")
        only = set(a[i + 1].split(","))
        del a[i:i + 2]
    rigged, clips, out = a
    js, bin_ = read_glb(rigged)
    cj, cbin = read_glb(clips)
    names = {n.get("name"): i for i, n in enumerate(js["nodes"])}
    js.setdefault("accessors", [])
    js.setdefault("bufferViews", [])
    js.setdefault("buffers", [{"byteLength": 0}])
    copied = {}

    def copy_accessor(k):
        if k in copied:
            return copied[k]
        acc = cj["accessors"][k]
        bv = cj["bufferViews"][acc["bufferView"]]
        width = SIZES[acc["componentType"]] * COUNTS[acc["type"]]
        start = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
        stride = bv.get("byteStride", width)
        data = b"".join(bytes(cbin[start + i * stride:start + i * stride + width]) for i in range(acc["count"]))
        bin_.extend(b"\0" * (-len(bin_) % 4))
        js["bufferViews"].append({"buffer": 0, "byteOffset": len(bin_), "byteLength": len(data)})
        bin_.extend(data)
        new = {k2: v for k2, v in acc.items() if k2 not in ("bufferView", "byteOffset", "sparse")}
        new["bufferView"] = len(js["bufferViews"]) - 1
        js["accessors"].append(new)
        copied[k] = len(js["accessors"]) - 1
        return copied[k]

    kept = dropped = 0
    anims = []
    for an in cj.get("animations", []):
        if only and an.get("name") not in only:
            continue
        samplers, channels = [], []
        for ch in an["channels"]:
            t = ch["target"]
            name = cj["nodes"][t["node"]].get("name")
            if t.get("path") != "rotation" or name not in names:
                dropped += 1
                continue
            s = an["samplers"][ch["sampler"]]
            samplers.append({"input": copy_accessor(s["input"]), "output": copy_accessor(s["output"]),
                             "interpolation": s.get("interpolation", "LINEAR")})
            channels.append({"sampler": len(samplers) - 1, "target": {"node": names[name], "path": "rotation"}})
            kept += 1
        if channels:
            anims.append({"name": an.get("name", "clip"), "samplers": samplers, "channels": channels})
    js["animations"] = anims
    js["buffers"][0]["byteLength"] = len(bin_) + (-len(bin_) % 4)
    js["buffers"][0].pop("uri", None)
    write_glb(out, js, bin_)
    print(f"CLIPS {len(anims)} clips, {kept} channels kept, {dropped} dropped")


if __name__ == "__main__":
    main()
