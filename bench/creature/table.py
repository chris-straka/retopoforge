#!/usr/bin/env python3
"""Creature bench tables (stdlib only).

    bench/creature/table.py WORK [--tools a,b,c] [--json out.json]
    bench/creature/table.py WORK --compare A B

On pro and animal subjects (a professional mesh is the truth, tool
`artist`), `pro-loops%` is the share of the pro's ringed joints the
candidate also rings, and `flow` the edge-flow angle against the pro
mesh (metrics.py --pro).

The first form prints, per kind of subject (tripo, animal, prop), the
median of each score per tool, then one row per subject and tool.
`--compare A B` lines tool B up against tool A subject by subject: per
score, the median change and on how many subjects B is better / worse,
and the verdict by the keep rule (docs/log.md):

  keep B when, over the subjects with a rig, B is better on more subjects
  than worse for at least one of bend, loops, joint poles, posed flips;
  and is not worse on more subjects than better for any of them; and its
  median dist_p95 is no more than 10% above A's.
"""
import argparse
import glob
import json
import os
import statistics

# (key, label, higher_is_better)
SCORES = [
    ("bend", "bend", True),
    ("joint_loops_mean", "loops", True),
    ("joints_ringed_pct", "ringed%", True),
    ("joint_poles_mean", "poles", False),
    ("loop_recall", "pro-loops%", True),
    ("flow_deg", "flow", False),
    ("pro_pose_d95", "bent-vs-pro%", False),
    ("pro_d95", "rest-vs-pro%", False),
    ("posed_flips", "pflips", False),
    ("rest_flips", "rflips", False),
    ("dist_p95", "d95%", False),
    ("dist_max", "dmax%", False),
    ("ndev_mean", "ndev", False),
    ("irr_pct", "irr%", False),
    ("open_edges", "open", False),
    ("tris", "tris", None),
]
RIGGED = {"bend", "joint_loops_mean", "joints_ringed_pct", "joint_poles_mean", "posed_flips", "loop_recall",
          "pro_pose_d95"}


def load(work):
    rows = {}
    for p in glob.glob(os.path.join(work, "*", "score", "*.json")):
        if p.endswith(".metrics.json"):
            continue
        r = json.load(open(p))
        m = dict(r.get("metrics") or {})
        b = r.get("bend") or {}
        if b.get("score") is not None:
            m["bend"] = b["score"]
            m["bend_fails"] = b.get("fails")
        tool, _, tris = r["cand"].partition("@")
        rows[(r["subject"], tool)] = {"kind": r["kind"], "budget": int(tris or 0), **m}
    # loops where the pro has them: share of the artist's ringed joints
    # that the candidate also rings
    for (subj, tool), r in rows.items():
        art = rows.get((subj, "artist"))
        if not art or tool == "artist":
            continue
        want = [j for j, x in (art.get("joints") or {}).items() if x.get("loops", 0) >= 1]
        have = r.get("joints") or {}
        if want:
            r["loop_recall"] = round(100.0 * sum(1 for j in want if have.get(j, {}).get("loops", 0) >= 1)
                                     / len(want), 1)
    return rows


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.3g}" if abs(v) < 100 else f"{v:.0f}"
    return str(v)


def summary(rows, tools):
    out = []
    kinds = sorted({r["kind"] for r in rows.values()})
    for kind in kinds:
        out.append(f"\n## {kind}: medians over subjects\n")
        out.append("| tool | n | " + " | ".join(lbl for _, lbl, _ in SCORES) + " |")
        out.append("|---" * (len(SCORES) + 2) + "|")
        for t in tools:
            sub = [r for (s, tt), r in rows.items() if tt == t and r["kind"] == kind]
            if not sub:
                continue
            cells = []
            for k, _, _ in SCORES:
                vals = [r[k] for r in sub if r.get(k) is not None]
                cells.append(fmt(statistics.median(vals)) if vals else "-")
            out.append(f"| {t} | {len(sub)} | " + " | ".join(cells) + " |")
    out.append("\n## every subject\n")
    out.append("| subject | tool | " + " | ".join(lbl for _, lbl, _ in SCORES) + " |")
    out.append("|---" * (len(SCORES) + 2) + "|")
    for s in sorted({s for s, _ in rows}):
        for t in tools:
            r = rows.get((s, t))
            if r:
                out.append(f"| {s} | {t} | " + " | ".join(fmt(r.get(k)) for k, _, _ in SCORES) + " |")
    return "\n".join(out)


def compare(rows, a, b):
    subs = sorted({s for (s, t) in rows if t == a} & {s for (s, t) in rows if t == b})
    out = [f"{b} against {a} on {len(subs)} subjects\n",
           "| score | median A | median B | B better | B worse |", "|---|---|---|---|---|"]
    verdict = {"gain": False, "loss": False}
    d95 = {}
    for k, lbl, hib in SCORES:
        if hib is None:
            continue
        pairs = [(rows[(s, a)].get(k), rows[(s, b)].get(k)) for s in subs]
        pairs = [(x, y) for x, y in pairs if x is not None and y is not None]
        if not pairs:
            continue
        better = sum(1 for x, y in pairs if (y > x if hib else y < x))
        worse = sum(1 for x, y in pairs if (y < x if hib else y > x))
        ma, mb = statistics.median(x for x, _ in pairs), statistics.median(y for _, y in pairs)
        out.append(f"| {lbl} | {fmt(ma)} | {fmt(mb)} | {better} | {worse} |")
        if k in RIGGED:
            verdict["gain"] |= better > worse
            verdict["loss"] |= worse > better
        if k == "dist_p95":
            d95 = {"a": ma, "b": mb}
    ok_shape = not d95 or d95["b"] <= 1.10 * d95["a"]
    keep = verdict["gain"] and not verdict["loss"] and ok_shape
    out.append(f"\nkeep rule: gain={verdict['gain']} loss={verdict['loss']} "
               f"shape_ok={ok_shape} -> {'KEEP' if keep else 'DROP'}")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("work")
    ap.add_argument("--tools", default="")
    ap.add_argument("--compare", nargs=2)
    ap.add_argument("--json")
    a = ap.parse_args()
    rows = load(a.work)
    if a.json:
        json.dump({f"{s}|{t}": r for (s, t), r in rows.items()}, open(a.json, "w"), indent=1, sort_keys=True)
    if a.compare:
        print(compare(rows, *a.compare))
        return
    tools = a.tools.split(",") if a.tools else sorted({t for _, t in rows})
    print(summary(rows, tools))


if __name__ == "__main__":
    main()
