#!/usr/bin/env python3
"""The round's pictures for the owner (needs Pillow and ffmpeg).

    bench/creature/pictures.py WORK OUT_DIR --ours TOOL [--vs decimate]
        [--subjects a,b] [--frames 36]

Per subject, in OUT_DIR/<subject>/:
  before.jpg   the source (the cleaned generator download), clay
  after.jpg    our mesh (TOOL's cut), clay with its wireframe, same camera
  vs.jpg       the --vs tool's cut, same camera
  joint.jpg    one bent joint: ours left, --vs right
  clip.mp4     both rigged meshes moving (ours left, --vs right), a
               two-second loop, H.264, a few hundred KB
Renders go through $BLENDER (bench/creature/render.py, clip.py).
"""
import argparse
import glob
import os
import shutil
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))


def blender(script, *args):
    r = subprocess.run([os.environ["BLENDER"], "-b", "--factory-startup", "-t", "2", "-P",
                        os.path.join(HERE, script), "--", *args], capture_output=True, text=True, timeout=3600)
    if r.returncode != 0:
        print(r.stdout[-800:], r.stderr[-800:])


def cand(work, subj, tool):
    hits = [h for h in sorted(glob.glob(os.path.join(work, subj, "cand", f"{tool}@*.obj")))
            if not h.endswith((".tmp.obj", ".first.obj")) and ".try" not in h]
    return os.path.basename(hits[0])[:-4] if hits else None


def label(img, text):
    d = ImageDraw.Draw(img)
    font = ImageFont.load_default(size=max(14, img.width // 32))
    d.rectangle([0, 0, img.width, font.size + 10], fill=(255, 255, 255))
    d.text((8, 4), text, fill=(0, 0, 0), font=font)
    return img


def side_by_side(a, b, la, lb):
    A = label(Image.open(a).convert("RGB"), la)
    B = label(Image.open(b).convert("RGB"), lb)
    out = Image.new("RGB", (A.width + B.width, max(A.height, B.height)), (255, 255, 255))
    out.paste(A, (0, 0))
    out.paste(B, (A.width, 0))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("work")
    ap.add_argument("out")
    ap.add_argument("--ours", required=True)
    ap.add_argument("--vs", default="decimate")
    ap.add_argument("--subjects", default="")
    ap.add_argument("--frames", type=int, default=36)
    ap.add_argument("--size", type=int, default=900)
    a = ap.parse_args()
    work = os.path.abspath(a.work)
    subs = a.subjects.split(",") if a.subjects else sorted(
        os.path.basename(os.path.dirname(p)) for p in glob.glob(os.path.join(work, "*", "ref", "aligned.glb"))
        if not os.path.basename(os.path.dirname(os.path.dirname(p))).startswith(("a_", "p_")))
    for s in subs:
        d = os.path.join(work, s)
        o = os.path.join(a.out, s)
        os.makedirs(o, exist_ok=True)
        dense = os.path.join(d, "dense.obj")
        tmp = tempfile.mkdtemp()
        blender("render.py", "-", dense, os.path.join(tmp, "src"), "--frame", dense, "--no-wire",
                "--size", str(a.size))
        names = {}
        for tag, tool in (("ours", a.ours), ("vs", a.vs)):
            c = cand(work, s, tool)
            if not c:
                continue
            names[tag] = c
            rig = os.path.join(d, "score", c + ".rigged.glb")
            obj = os.path.join(d, "cand", c + ".obj")
            blender("render.py", rig, obj, os.path.join(tmp, tag), "--frame", dense, "--size", str(a.size))
            blender("clip.py", rig, obj, os.path.join(tmp, tag + "_clip"), "--frame", dense,
                    "--frames", str(a.frames), "--size", "480")
        if os.path.exists(os.path.join(tmp, "src.whole.png")):
            label(Image.open(os.path.join(tmp, "src.whole.png")).convert("RGB"), "generator download").save(
                os.path.join(o, "before.jpg"), quality=88)
        for tag, fn in (("ours", "after.jpg"), ("vs", "vs.jpg")):
            p = os.path.join(tmp, tag + ".whole.png")
            if os.path.exists(p):
                label(Image.open(p).convert("RGB"), names[tag]).save(os.path.join(o, fn), quality=88)
        ja, jb = os.path.join(tmp, "ours.joint.png"), os.path.join(tmp, "vs.joint.png")
        if os.path.exists(ja) and os.path.exists(jb):
            side_by_side(ja, jb, names["ours"], names["vs"]).save(os.path.join(o, "joint.jpg"), quality=88)
        fa, fb = os.path.join(tmp, "ours_clip"), os.path.join(tmp, "vs_clip")
        if os.path.isdir(fa) and os.path.isdir(fb):
            frames = os.path.join(tmp, "both")
            os.makedirs(frames, exist_ok=True)
            for f in sorted(os.listdir(fa)):
                if os.path.exists(os.path.join(fb, f)):
                    side_by_side(os.path.join(fa, f), os.path.join(fb, f), names["ours"], names["vs"]).save(
                        os.path.join(frames, f))
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "18", "-i",
                            os.path.join(frames, "%04d.png"), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                            "-crf", "26", "-movflags", "+faststart", os.path.join(o, "clip.mp4")])
        shutil.rmtree(tmp, ignore_errors=True)
        print(s, sorted(os.listdir(o)), flush=True)


if __name__ == "__main__":
    main()
