use super::{cxx_max, cxx_min};
use crate::vector2::Vector2;

// Global UV atlas: shelf-packs the per-island UV ranges (each already
// normalized to 0..1 by QuadExtractor::computeRemeshedVertexUvs, so
// islands would otherwise stack on each other) into one shared 0..1
// atlas. `spans` holds each merged island's (start, count) slice of
// `uvs`, in merge order.
//
// (C++ contract comment, kept:) Packing inside 0..1 was chosen over
// per-island UDIM offsets because the shipped --uvs contract requires
// every UV inside 0..1 (test_cli_uvs asserts it, and GLB TEXCOORD_0
// consumers expect normalized coordinates); UDIM tiles (u >= 1) would
// break both. Deterministic shelf packing: islands sort by box height
// (ties by width, then merge order), fill rows left to right inside a
// strip ceil(sqrt(N)) boxes wide, then one uniform scale fits the shelves
// into 0..1 with a fixed 2-texel-at-1k gutter between boxes. A single
// island returns untouched, so one-island output keeps today's exact UVs.
// Geometry is never touched (only the UV array is rewritten), and the
// --uvs off path never calls this.
const ATLAS_GUTTER: f64 = 2.0 / 1024.0;

pub(crate) fn pack_island_uvs_into_atlas(uvs: &mut [Vector2], spans: &[(usize, usize)]) {
    if uvs.is_empty() {
        return;
    }
    let mut islands = Vec::new();
    for (i, span) in spans.iter().enumerate() {
        if span.1 > 0 {
            islands.push(i);
        }
    }
    if islands.len() <= 1 {
        return;
    }

    #[derive(Clone, Copy, Default)]
    struct AtlasBox {
        min_u: f64,
        min_v: f64,
        width: f64,
        height: f64,
    }
    let mut boxes = vec![AtlasBox::default(); spans.len()];
    let mut max_width = 0.0;
    for &island in &islands {
        let begin = spans[island].0;
        let end = (begin + spans[island].1).min(uvs.len());
        if begin >= end {
            continue;
        }
        let mut min_u = uvs[begin].x();
        let mut max_u = min_u;
        let mut min_v = uvs[begin].y();
        let mut max_v = min_v;
        for p in &uvs[begin + 1..end] {
            min_u = cxx_min(min_u, p.x());
            max_u = cxx_max(max_u, p.x());
            min_v = cxx_min(min_v, p.y());
            max_v = cxx_max(max_v, p.y());
        }
        boxes[island] = AtlasBox {
            min_u,
            min_v,
            width: max_u - min_u,
            height: max_v - min_v,
        };
        max_width = cxx_max(max_width, boxes[island].width);
    }

    // Stable sort with the literal C++ comparator (descending height,
    // then descending width, then ascending merge order). NaN box
    // extents make the comparator inconsistent on both sides (C++
    // `stable_sort` and Rust `sort_by` then both pick an unspecified but
    // deterministic order); sane UVs never produce them.
    islands.sort_by(|&a, &b| {
        if boxes[a].height != boxes[b].height {
            if boxes[b].height < boxes[a].height {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        } else if boxes[a].width != boxes[b].width {
            if boxes[b].width < boxes[a].width {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        } else {
            a.cmp(&b)
        }
    });

    // A strip ceil(sqrt(N)) boxes wide keeps the shelves roughly square
    // for the common equal-box case (every island spans the full unit
    // square before packing).
    let columns = (islands.len() as f64).sqrt().ceil();
    let strip_width = if max_width > 0.0 {
        columns * max_width
    } else {
        1.0
    };
    struct Shelf {
        members: Vec<usize>,
        width: f64,
        height: f64,
    }
    let mut shelves = vec![Shelf {
        members: Vec::new(),
        width: 0.0,
        height: 0.0,
    }];
    for &island in &islands {
        // `shelves` always holds at least one shelf (it starts with one
        // and only grows), so the back index is always valid.
        let back = shelves.len() - 1;
        if !shelves[back].members.is_empty()
            && shelves[back].width + boxes[island].width > strip_width
        {
            shelves.push(Shelf {
                members: Vec::new(),
                width: 0.0,
                height: 0.0,
            });
        }
        let back = shelves.len() - 1;
        shelves[back].members.push(island);
        shelves[back].width += boxes[island].width;
        let grown = cxx_max(shelves[back].height, boxes[island].height);
        shelves[back].height = grown;
    }

    // The gutter is fixed in atlas units; only shrink it when the box
    // count alone would overflow the unit square (hundreds of islands).
    let mut widest_shelf = 0usize;
    let mut total_height = 0.0;
    for shelf in &shelves {
        widest_shelf = widest_shelf.max(shelf.members.len());
        total_height += shelf.height;
    }
    let mut gutter = ATLAS_GUTTER;
    let gaps = (widest_shelf.saturating_sub(1)).max(shelves.len().saturating_sub(1));
    if gaps > 0 && gutter * gaps as f64 > 0.5 {
        gutter = 0.5 / gaps as f64;
    }

    // FMA audit (site E): Clang fuses `1.0 - gutter * count` to
    // llvm.fmuladd(-gutter, count, 1.0) (negation first, then one fused
    // op); the division stays separate. Transcribed explicitly at both
    // occurrences below.
    let mut scale = 1.0;
    for shelf in &shelves {
        if shelf.width <= 0.0 {
            continue;
        }
        scale = cxx_min(
            scale,
            (-gutter).mul_add((shelf.members.len() - 1) as f64, 1.0) / shelf.width,
        );
    }
    if total_height > 0.0 {
        scale = cxx_min(
            scale,
            (-gutter).mul_add((shelves.len() - 1) as f64, 1.0) / total_height,
        );
    }
    scale = cxx_min(1.0, cxx_max(1e-9, scale));

    let mut y = 0.0;
    for shelf in &shelves {
        let mut x = 0.0;
        for &island in &shelf.members {
            let bx = boxes[island];
            let begin = spans[island].0;
            let end = (begin + spans[island].1).min(uvs.len());
            for slot in &mut uvs[begin..end] {
                // FMA audit (site F): Clang fuses to
                // llvm.fmuladd(u - min, scale, origin). Transcribed
                // explicitly.
                let u = (slot.x() - bx.min_u).mul_add(scale, x);
                let v = (slot.y() - bx.min_v).mul_add(scale, y);
                // `std::min(1.0, std::max(0.0, u))`, transcribed
                // literally (NOT `clamp`: clamp keeps a NaN self where the
                // C++ yields 0.0, and clippy's manual_clamp must not
                // "fix" this).
                *slot = Vector2::new(cxx_min(1.0, cxx_max(0.0, u)), cxx_min(1.0, cxx_max(0.0, v)));
            }
            // FMA audit (site G): Clang fuses the inner `(w * scale) +
            // gutter` to llvm.fmuladd(w, scale, gutter), then adds the
            // running origin separately. Transcribed explicitly.
            x += bx.width.mul_add(scale, gutter);
        }
        y += shelf.height.mul_add(scale, gutter);
    }
}

// Per-island durations are accumulated in microseconds: a mesh split into
// hundreds of islands spends well under a millisecond on most of them, and
// truncating each one to whole milliseconds loses the bulk of the total.
