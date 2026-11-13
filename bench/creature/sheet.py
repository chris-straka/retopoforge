#!/usr/bin/env python3
"""Picture sheets for the creature bench: one row a subject, one column a
tool, labelled. Renders whatever picture is missing first
(bench/creature/render.py, through $BLENDER).

    bench/creature/sheet.py WORK --tools a,b --out sheet.jpg
        [--subjects x,y] [--view whole|joint] [--size 600] [--jobs 3]

Needs Pillow (python3 -c "import PIL").
"""
import argparse
import concurrent.futures as cf
import glob
import json
import os
import subprocess

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))


def picture(work, subj, cand, view, size):
    sd = os.path.join(work, subj, "score")
    prefix = os.path.join(sd, cand + ".pic")
    png = f"{prefix}.{view}.png"
    if not os.path.exists(png):
        rigged = os.path.join(sd, cand + ".rigged.glb")
        obj = os.path.join(work, subj, "cand", cand + ".obj")
        if not os.path.exists(obj):
            return None
        subprocess.run([os.environ["BLENDER"], "-b", "--factory-startup", "-t", "2", "-P",
                        os.path.join(HERE, "render.py"), "--", rigged if os.path.exists(rigged) else "-",
                        obj, prefix, "--size", str(size)], capture_output=True, timeout=1200)
    return png if os.path.exists(png) else None


def caption(work, subj, cand):
    p = os.path.join(work, subj, "score", cand + ".json")
    if not os.path.exists(p):
        return cand
    r = json.load(open(p))
    m, b = r.get("metrics") or {}, r.get("bend") or {}
    bits = [cand, f"{m.get('tris')} tris"]
    if b.get("score") is not None:
        bits.append(f"bend {b['score']:.0f}")
    if m.get("n_joints"):
        bits.append(f"loops {m.get('joint_loops_mean')} poles {m.get('joint_poles_mean')}")
        bits.append(f"flips {m.get('rest_flips')}/{m.get('posed_flips')}")
    bits.append(f"d95 {m.get('dist_p95')}%")
    return "  ".join(bits[:3]) + "\n" + "  ".join(bits[3:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("work")
    ap.add_argument("--tools", required=True)
    ap.add_argument("--subjects", default="")
    ap.add_argument("--view", default="whole")
    ap.add_argument("--size", type=int, default=600)
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    work = os.path.abspath(a.work)
    subs = a.subjects.split(",") if a.subjects else sorted(
        os.path.basename(os.path.dirname(d)) for d in glob.glob(os.path.join(work, "*", "cand")))
    tools = a.tools.split(",")
    cells = []
    for s in subs:
        for t in tools:
            hits = sorted(glob.glob(os.path.join(work, s, "cand", f"{t}@*.obj")))
            hits = [h for h in hits if not h.endswith((".tmp.obj", ".first.obj"))]
            cells.append((s, os.path.basename(hits[0])[:-4] if hits else None))
    with cf.ThreadPoolExecutor(a.jobs) as ex:
        pngs = list(ex.map(lambda c: picture(work, c[0], c[1], a.view, a.size) if c[1] else None, cells))
    sz, cap = a.size, 62
    font = ImageFont.load_default(size=15)
    sheet = Image.new("RGB", (sz * len(tools), (sz + cap) * len(subs)), (255, 255, 255))
    draw = ImageDraw.Draw(sheet)
    for k, ((s, cand), png) in enumerate(zip(cells, pngs)):
        x, y = (k % len(tools)) * sz, (k // len(tools)) * (sz + cap)
        if png:
            sheet.paste(Image.open(png).convert("RGB").resize((sz, sz)), (x, y + cap))
        text = f"{s}\n{caption(work, s, cand) if cand else 'no cut'}"
        draw.multiline_text((x + 6, y + 4), text, fill=(0, 0, 0), font=font, spacing=2)
    sheet.save(a.out, quality=88)
    print(a.out)


if __name__ == "__main__":
    main()
