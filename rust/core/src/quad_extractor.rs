//! Port of `core/quadextractor.*` (`retopo.core.quad_extractor`).
//!
//! Line-by-line mirror: same methods in the same order, same thresholds,
//! same constants. Public API is snake_cased 1:1 (`extractConnections` ->
//! [`QuadExtractor::extract_connections`], but private in both).
//!
//! Deliberate restructures (all behavior-preserving, each marked at the
//! site with `// Restructure:`):
//!
//! - `std::unordered_map/set` iteration order is libc++ `__hash_table`
//!   order, which no stock Rust map reproduces — and the greedy passes
//!   observe it (collapse survivors, face discovery, neighbor sums), so
//!   sorted iteration diverges structurally. Every unordered container here
//!   is a `CxxSet`/`CxxMap` (exact order emulation, proved by the
//!   `cxx_hash_oracle` unit test against the fixture's CXXHASH section).
//!   Containers mirroring `std::map`/`std::set` stay
//!   [`BTreeMap`]/[`BTreeSet`] (same sorted order as the C++).
//! - C++ erases from maps/sets while iterating (`simplifyGraph`,
//!   `smoothAroundVertices`) and default-inserts on `operator[]` reads;
//!   Rust collects keys first, then mutates, and reads through `get` with
//!   an explicit default. Each site notes why the insert/erase traffic is
//!   unobservable.
//! - C++ lambdas capturing `&mut self` become private associated functions
//!   and free helpers with explicit parameters (borrowck); the call graphs
//!   are unchanged.
//! - `tbb::parallel_for` becomes [`crate::par::parallel_each`]: every
//!   parallel site in the C++ reads one buffer and writes another (or
//!   writes disjoint slots), so the parallel result is identical by
//!   construction (noted at the site).
//! - `std::sort` on hole edge scores becomes `sort_unstable_by`: both are
//!   deterministic but order score ties differently (libc++ introsort vs
//!   pdqsort). The oracle measures the fallout; see the report.
//! - `std::max_element` (first maximum) is a manual strict-`<` loop:
//!   Rust's `max_by` returns the LAST maximum.
//! - `AUTO_REMESHER_DEV` obj-dump blocks are compiled out (the flag is not
//!   in the build definitions) and are not mirrored.
//!
//! FMA transcription (mandatory audit, redone 2026-10-01 on brew Clang 23
//! ARM64 `-O3` after the oracle rejected the first audit — see below):
//! BARE-SCALAR `p +/- q*r` shapes fuse (first product fused: `p*b + q*d`
//! -> `fma(p, b, q*d)`, `v + d*s` -> `fma(d, s, v)`), but Vector2/3
//! OPERATOR expressions never do: `a*(1-r)+b*r`, `v+d*s`, `v-d*s`,
//! `a+ab*v+ac*w` and `a-ab*v-ac*w` through the (inlined) `operator*` /
//! `operator+` / `operator-` temporaries all compile to plain fmul/fadd/
//! fsub chains (probe-verified per shape; the temporaries break the
//! fusion patterns). The first (wrong) audit assumed the vector shapes
//! fused like the scalar ones and transcribed them with explicit
//! `mul_add`; the differential oracle caught it (200/218 cases off by
//! 1ulp on connection endpoints). Rule used below: vector shapes go
//! through the crate operators untouched (Rust never fuses implicitly,
//! so the operator expression is already bitwise); bare-scalar mul-add
//! shapes use explicit [`f64::mul_add`] via `fma_first`/`fma_first_sub`
//! or inline. The vector-internal fusions (dot, length) are already
//! mirrored by [`crate::vector2`]/[`crate::vector3`].
//!
//! Former inline mirrors, deduped at join: `PositionKey` was a private
//! mirror of `retopo.core.position_key` and
//! `TpVector3`/`AxisAlignedBox`/`AxisAlignedBoxTree` a private mirror of
//! the thirdparty isotropicremesher bounding-box tree, both vendored while
//! the sibling lanes ran concurrently. The port now uses the joined
//! `crate::position_key` and `crate::iso_remesh_kernel` directly
//! (call sites adapted to the joined owned-box API).

use crate::double_utils::is_zero;
use crate::iso_remesh_kernel::{AxisAlignedBoundingBox, AxisAlignedBoundingBoxTree};
use crate::mesh_separator::MeshSeparator;
use crate::par::parallel_each;
use crate::position_key::PositionKey;
use crate::progress::ProgressHandler;
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::f64::consts::PI;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Caller-level vector-expression helpers: each mirrors one C++ expression
// shape through the crate operators (see the module FMA note: the vector
// shapes are UNFUSED on both sides, so the operator expression is already
// bitwise). Scalar mul-add shapes use `fma_first`/`fma_first_sub`/inline
// `mul_add` (those DO fuse in the C++).
// ---------------------------------------------------------------------------

/// `a * (1 - ratio) + b * ratio` via the crate operators (unfused, like
/// the C++; `1 - ratio` with the C++ int literal equals `1.0 - ratio`).
#[inline]
fn lerp_vec3(a: Vector3, b: Vector3, ratio: f64) -> Vector3 {
    a * (1.0 - ratio) + b * ratio
}

/// `Vector2` form of [`lerp_vec3`].
#[inline]
fn lerp_vec2(a: Vector2, b: Vector2, ratio: f64) -> Vector2 {
    a * (1.0 - ratio) + b * ratio
}

/// `v + d * s` via the crate operators (unfused, like the C++).
#[inline]
fn add_scaled_vec3(v: Vector3, d: Vector3, s: f64) -> Vector3 {
    v + d * s
}

/// `v - d * s` via the crate operators (unfused, like the C++).
#[inline]
fn sub_scaled_vec3(v: Vector3, d: Vector3, s: f64) -> Vector3 {
    v - d * s
}

/// `a + ab * v + ac * w` (left-nested, as written) via the crate
/// operators (unfused, like the C++).
#[inline]
fn add_two_scaled_vec3(a: Vector3, ab: Vector3, v: f64, ac: Vector3, w: f64) -> Vector3 {
    a + ab * v + ac * w
}

/// `p * b + q * d` with the FIRST product fused: `fma(p, b, q * d)`.
#[inline]
fn fma_first(p: f64, b: f64, q: f64, d: f64) -> f64 {
    p.mul_add(b, q * d)
}

/// `p * b - q * d` as the IR fuses it: `fma(p, b, q * -d)`.
#[inline]
fn fma_first_sub(p: f64, b: f64, q: f64, d: f64) -> f64 {
    p.mul_add(b, q * -d)
}

// ---------------------------------------------------------------------------
// Inline emulation of libc++ `unordered_set<size_t>` / `unordered_map<size_t, V>`
// iteration order. Every container below that mirrors a C++ unordered container
// is a [`CxxSet`]/[`CxxMap`]; containers mirroring `std::map`/`std::set` stay
// [`BTreeMap`]/[`BTreeSet`].
//
// Why: the extractor's greedy passes (collapse survivor selection, face
// discovery order in `extract_mesh`, neighbor summation order in
// `smooth_and_project`, first-come-wins tiling) observe unordered iteration
// order, so sorted iteration diverges structurally from the C++ (83/227
// fixture cases diverge before `extract_mesh` even runs). The emulation
// reproduces libc++ `__hash_table` order bit-for-bit; see `cxx_hash_oracle`
// below, proved against the CXXHASH + NEXTPRIME fixture sections.
//
// Rules (from brew LLVM 23 `__hash_table`, each cited; all keys here are
// `size_t`, hashed by identity — `__scalar_hash<size_t, 1>`):
//
// - A table is a node list (iteration order) plus a bucket count. New keys
//   go immediately before their bucket chain's first node, or at the list
//   front when the chain is empty (`__emplace_unique`: "insert_after
//   __bucket_list_[__chash], or __first_node if bucket is null").
// - Bucket index is `__constrain_hash`: mask when the count is a power of
//   two, modulo otherwise.
// - A rehash regroups in place: nodes met out of chain order are spliced to
//   the front of their chain (`__do_rehash`, unique-keys path). It triggers
//   on insert when `size + 1 > buckets * max_load` (load factor 1, never
//   changed) with the new count `next_prime(max(2 * buckets + 1, size + 1))`
//   (`2` for a fresh table). Erase never rehashes.
// - Erase unlinks (`remove`); survivors keep order. `clear` keeps the bucket
//   count. Copies preserve list order and the count (`__copy_construct`),
//   EXCEPT copies of empty tables, which come out fresh (`bc == 0`): the
//   copy ctor early-returns before allocating buckets when size is 0.
// - `std::__next_prime` is the true smallest-prime-`>=`-n function (probed
//   densely to 300000 plus tail values; the dylib throws past the last
//   representable prime, which needs a 2^63-element table and is
//   unreachable — the mirror saturates there, documented at the site).
//
// Bounds: float/int exactness below 2^24 elements (`size + 1 > bc * 1.0f`
// in the C++). Production tables hold 100k+ entries (cross-point graphs),
// so every op is O(1) (O(n) regroup on growth); the `HashMap` lookups are
// never iterated, keeping their order unobservable. `u64`/`usize` are
// interchangeable below (LP64, like `PositionKey`).
//
// Deliberately missing `insert`/`entry` on [`CxxMap`]: `BTreeMap::insert`
// overwrites while C++ `insert` keeps the old value, so every map write
// site names its C++ counterpart explicitly (`insert_new` keeps,
// `set`/`get_or_default` overwrite-or-insert; `set` is currently only
// reached by the container oracle tests).
// ---------------------------------------------------------------------------

/// `std::__constrain_hash`, literal (the `bc == 0` arm yields `h`, exactly
/// like the C++; unreachable here — every caller rehashes from 0 first).
#[inline]
fn cxx_constrain_hash(hash: usize, buckets: usize) -> usize {
    if buckets & buckets.wrapping_sub(1) == 0 {
        hash & buckets.wrapping_sub(1)
    } else if hash < buckets {
        hash
    } else {
        hash % buckets
    }
}

/// `std::__is_hash_power2`, literal.
#[inline]
fn cxx_is_hash_pow2(buckets: usize) -> bool {
    buckets > 2 && buckets & (buckets - 1) == 0
}

/// Smallest prime `>= n` (`std::__next_prime`, probed; `next_prime(0) == 0`
/// is a dylib quirk the mirror keeps). Primality is deterministic
/// Miller-Rabin (7-base set, exact for all `u64`).
fn cxx_next_prime(n: usize) -> usize {
    if n == 0 {
        return 0;
    }
    if n <= 2 {
        return 2;
    }
    // `n + 1` cannot overflow: `usize::MAX` is odd, so an even `n` here is
    // at most `MAX - 1`.
    let mut candidate = if n.is_multiple_of(2) { n + 1 } else { n };
    loop {
        if cxx_is_prime(candidate as u64) {
            return candidate;
        }
        // Past the last representable prime the C++ throws `overflow_error`;
        // that needs a table with ~2^63 elements, so saturate instead.
        candidate = candidate.saturating_add(2);
        if candidate == usize::MAX {
            return usize::MAX;
        }
    }
}

/// Deterministic Miller-Rabin for `u64` (bases 2, 325, 9375, 28178, 450775,
/// 9780504, 1795265022 — exact over the full range).
fn cxx_is_prime(n: u64) -> bool {
    // Trial division first: exact, and fast for the small counts tables use.
    const SMALL: [u64; 12] = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37];
    for p in SMALL {
        if n == p {
            return true;
        }
        if n.is_multiple_of(p) {
            return false;
        }
    }
    let mut d = n - 1;
    let mut r = 0u32;
    while d.is_multiple_of(2) {
        d /= 2;
        r += 1;
    }
    const BASES: [u64; 7] = [2, 325, 9375, 28178, 450775, 9780504, 1795265022];
    'bases: for a in BASES {
        let a = a % n;
        if a == 0 {
            continue;
        }
        let mut x = cxx_mod_pow(a, d, n);
        if x == 1 || x == n - 1 {
            continue;
        }
        for _ in 1..r {
            x = ((x as u128 * x as u128) % n as u128) as u64;
            if x == n - 1 {
                continue 'bases;
            }
        }
        return false;
    }
    true
}

fn cxx_mod_pow(mut base: u64, mut exp: u64, modulus: u64) -> u64 {
    let mut result = 1u64;
    base %= modulus;
    while exp > 0 {
        if exp % 2 == 1 {
            result = ((result as u128 * base as u128) % modulus as u128) as u64;
        }
        base = ((base as u128 * base as u128) % modulus as u128) as u64;
        exp /= 2;
    }
    result
}

// NOTE: the previous `cxx_regroup` / `cxx_maybe_rehash` splice loop now lives
// inside `CxxTable` as the O(n) closed-form `regroup`, the verbatim trigger
// `grow_rehash_if_needed`, and the O(1) chain-head lookup in `insert_fresh`
// (same libc++ order, O(1) updates).

/// Empty-list marker for [`CxxSlot`] links (no table here reaches 2^32
/// entries, let alone `usize::MAX`).
const NO_SLOT: usize = usize::MAX;

/// Fixed-seed integer hasher for [`CxxTable`]'s key lookup (never
/// iterated, so the hash order is unobservable): one rotate-add-multiply
/// per `usize` instead of SipHash's rounds. Keys are mesh indices, never
/// adversarial, so a non-cryptographic mixer is safe.
#[derive(Clone, Copy, Debug, Default)]
struct CxxHashBuilder;

#[derive(Clone, Debug)]
struct CxxHasher {
    hash: u64,
}

impl std::hash::Hasher for CxxHasher {
    fn write(&mut self, bytes: &[u8]) {
        // Only `write_usize` runs (keys are `usize`); fold bytes so the
        // impl stays total.
        for chunk in bytes.chunks(8) {
            let mut word = [0u8; 8];
            word[..chunk.len()].copy_from_slice(chunk);
            self.write_u64(u64::from_ne_bytes(word));
        }
    }

    fn write_usize(&mut self, value: usize) {
        self.write_u64(value as u64);
    }

    fn write_u64(&mut self, value: u64) {
        const ROTATE: u32 = 5;
        const SEED: u64 = 0x51_7c_c1_b7_27_22_0a_95;
        self.hash = (self.hash.rotate_left(ROTATE).wrapping_add(value)).wrapping_mul(SEED);
    }

    fn finish(&self) -> u64 {
        self.hash
    }
}

impl std::hash::BuildHasher for CxxHashBuilder {
    type Hasher = CxxHasher;

    fn build_hasher(&self) -> CxxHasher {
        CxxHasher { hash: 0 }
    }
}

/// One node of a [`CxxTable`]'s iteration list. Slots are append-only and
/// recycled through the table's free list; a live slot's index never
/// changes, so inserts and erases never shift any lookup.
#[derive(Clone, Debug)]
struct CxxSlot<V> {
    key: usize,
    /// `Some` on live slots; `None` on free-list slots (the value is
    /// dropped at erase time so dead buffers are freed).
    value: Option<V>,
    prev: usize,
    next: usize,
}

/// Exact-order libc++ `__hash_table` emulation with O(1) updates.
///
/// Iteration order, bucket-count evolution, and every method's return value
/// are identical to the previous `Vec` + `BTreeMap` emulation (proved by
/// the CXXHASH fixture oracle and the randomized old-vs-new differential
/// test); only the asymptotics changed, because production tables hold
/// 100k+ entries (cross-point graphs), not the hundreds the old code was
/// shaped for:
/// - node list: index-linked slots instead of a `Vec` that memmoves on
///   every insert/erase;
/// - membership: `HashMap` key->slot with a fixed-seed integer hasher
///   instead of a `BTreeMap` whose values all shift on every insert/erase
///   (never iterated, so its order is unobservable);
/// - chain heads: a `Vec` indexed by chain (chains are dense in
///   `[0, buckets)`) instead of an O(n) scan per insert;
/// - rehash regroup: the O(n) closed form instead of O(n^2) splicing (see
///   [`CxxTable::regroup`]).
#[derive(Debug)]
struct CxxTable<V> {
    slots: Vec<CxxSlot<V>>,
    free: Vec<usize>,
    head: usize,
    index: HashMap<usize, usize, CxxHashBuilder>,
    /// First slot per chain (`len == buckets`; `None` for empty chains).
    chain_head: Vec<Option<usize>>,
    buckets: usize,
}

impl<V: Clone> Clone for CxxTable<V> {
    /// Copy ctor (the old `CxxSet::clone` rule): a copy of an EMPTY table
    /// is fresh (`buckets == 0`); non-empty tables keep list order and the
    /// count.
    fn clone(&self) -> Self {
        if self.index.is_empty() {
            Self::new()
        } else {
            Self {
                slots: self.slots.clone(),
                free: self.free.clone(),
                head: self.head,
                index: self.index.clone(),
                chain_head: self.chain_head.clone(),
                buckets: self.buckets,
            }
        }
    }
}

impl<V> Default for CxxTable<V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<V> CxxTable<V> {
    fn new() -> Self {
        Self {
            slots: Vec::new(),
            free: Vec::new(),
            head: NO_SLOT,
            index: HashMap::with_hasher(CxxHashBuilder),
            chain_head: Vec::new(),
            buckets: 0,
        }
    }

    fn len(&self) -> usize {
        self.index.len()
    }

    fn is_empty(&self) -> bool {
        self.index.is_empty()
    }

    /// Bucket count (test-only inspection).
    #[cfg(test)]
    fn buckets(&self) -> usize {
        self.buckets
    }

    fn contains_key(&self, key: &usize) -> bool {
        self.index.contains_key(key)
    }

    /// The value of a live slot (every slot named by `index` is live).
    fn live_value(&self, slot: usize) -> &V {
        self.slots[slot]
            .value
            .as_ref()
            .expect("indexed slot holds a value")
    }

    /// Mutable form of [`CxxTable::live_value`].
    fn live_value_mut(&mut self, slot: usize) -> &mut V {
        self.slots[slot]
            .value
            .as_mut()
            .expect("indexed slot holds a value")
    }

    fn get(&self, key: &usize) -> Option<&V> {
        self.index.get(key).map(|slot| self.live_value(*slot))
    }

    fn get_mut(&mut self, key: &usize) -> Option<&mut V> {
        let slot = *self.index.get(key)?;
        Some(self.live_value_mut(slot))
    }

    /// Links a detached slot at the list front.
    fn link_front(&mut self, slot: usize) {
        let old_head = self.head;
        self.slots[slot].prev = NO_SLOT;
        self.slots[slot].next = old_head;
        if old_head != NO_SLOT {
            self.slots[old_head].prev = slot;
        }
        self.head = slot;
    }

    /// Links a detached slot immediately before `at`.
    fn link_before(&mut self, slot: usize, at: usize) {
        let prev = self.slots[at].prev;
        self.slots[slot].prev = prev;
        self.slots[slot].next = at;
        self.slots[at].prev = slot;
        if prev != NO_SLOT {
            self.slots[prev].next = slot;
        } else {
            self.head = slot;
        }
    }

    /// Detaches a linked slot (neighbors/head fixed; the slot's own links
    /// go stale — `alloc_slot` overwrites them on reuse).
    fn unlink(&mut self, slot: usize) {
        let prev = self.slots[slot].prev;
        let next = self.slots[slot].next;
        if prev != NO_SLOT {
            self.slots[prev].next = next;
        } else {
            self.head = next;
        }
        if next != NO_SLOT {
            self.slots[next].prev = prev;
        }
    }

    /// Takes a slot for a fresh key (recycled or appended); the slot comes
    /// back detached with its key/value stored.
    fn alloc_slot(&mut self, key: usize, value: V) -> usize {
        let slot = self.free.pop().unwrap_or_else(|| {
            self.slots.push(CxxSlot {
                key: 0,
                value: None,
                prev: NO_SLOT,
                next: NO_SLOT,
            });
            self.slots.len() - 1
        });
        self.slots[slot] = CxxSlot {
            key,
            value: Some(value),
            prev: NO_SLOT,
            next: NO_SLOT,
        };
        slot
    }

    /// Grow step: the `size + 1 > bc * max_load` trigger plus
    /// `__rehash_unique(max(2 * bc + !is_pow2(bc), size + 1))`, verbatim
    /// from the previous `cxx_maybe_rehash` (the C++ float spell agrees
    /// below 2^24 elements).
    fn grow_rehash_if_needed(&mut self) {
        if self.index.len() + 1 > self.buckets {
            let arg = (2 * self.buckets + usize::from(!cxx_is_hash_pow2(self.buckets)))
                .max(self.index.len() + 1);
            let grown = if arg == 1 {
                2
            } else if arg & (arg - 1) != 0 {
                cxx_next_prime(arg)
            } else {
                arg
            };
            debug_assert!(grown > self.buckets);
            self.buckets = grown;
            // Size the chain-head table before regrouping (even for an
            // empty table, whose regroup is a no-op): every chain the new
            // inserts touch must index validly.
            self.chain_head.clear();
            self.chain_head.resize(grown, None);
            self.regroup();
        }
    }

    /// `__do_rehash` unique-keys path in O(n): regroups the list in place
    /// under the new bucket count.
    ///
    /// Closed form of the previous splice loop (proved equivalent by the
    /// randomized old-vs-new differential test): one walk in iteration
    /// order tracks `prev` (the chain of the last appended arrival) and
    /// each chain's run. An arrival appends to its run when its chain
    /// equals `prev` or the chain is new (updating `prev`); a returning
    /// chain's arrival prepends instead (the splice arm, which leaves
    /// `prev` untouched). Runs keep creation order; each run holds its
    /// prepended arrivals reversed, then its appended arrivals in order.
    /// Slots are relaid out in the regrouped order (iteration stays cache
    /// friendly) and both lookups rebuilt.
    fn regroup(&mut self) {
        if self.index.is_empty() {
            return;
        }
        // Pass 1: classify arrivals, count runs. Chains are dense in
        // `[0, buckets)`, so a Vec table replaces hashing; at regroup
        // time `buckets <= len` (the trigger fired), bounding the table.
        struct Run {
            rank: usize,
            prepended: usize,
            appended: usize,
        }
        let mut runs: Vec<Option<Run>> = Vec::new();
        runs.resize_with(self.buckets, || None);
        let mut chain_order: Vec<usize> = Vec::new();
        let mut prev: Option<usize> = None;
        let mut cursor = self.head;
        while cursor != NO_SLOT {
            let chain = cxx_constrain_hash(self.slots[cursor].key, self.buckets);
            cursor = self.slots[cursor].next;
            let fresh = runs[chain].is_none();
            let run = runs[chain].get_or_insert_with(|| {
                let rank = chain_order.len();
                chain_order.push(chain);
                Run {
                    rank,
                    prepended: 0,
                    appended: 0,
                }
            });
            if fresh || Some(chain) == prev {
                run.appended += 1;
                prev = Some(chain);
            } else {
                run.prepended += 1;
            }
        }
        // Pass 2: lay out segments (runs in creation order) and place
        // every slot with per-run cursors — the prepend cursor runs down
        // from its sub-segment end (arrivals land reversed), the append
        // cursor runs up from its sub-segment start.
        let mut base_of_rank = vec![0usize; chain_order.len()];
        let mut base = 0;
        for (rank, chain) in chain_order.iter().enumerate() {
            base_of_rank[rank] = base;
            let run = runs[*chain].as_ref().expect("counted chain has a run");
            base += run.prepended + run.appended;
        }
        debug_assert_eq!(base, self.index.len());
        let mut cursors: Vec<(usize, usize)> = chain_order
            .iter()
            .enumerate()
            .map(|(rank, chain)| {
                let run = runs[*chain].as_ref().expect("counted chain has a run");
                (
                    base_of_rank[rank] + run.prepended,
                    base_of_rank[rank] + run.prepended,
                )
            })
            .collect();
        let mut final_order = vec![NO_SLOT; self.index.len()];
        let mut seen = vec![false; chain_order.len()];
        let mut prev: Option<usize> = None;
        let mut cursor = self.head;
        while cursor != NO_SLOT {
            let slot = cursor;
            let chain = cxx_constrain_hash(self.slots[slot].key, self.buckets);
            cursor = self.slots[slot].next;
            let rank = runs[chain].as_ref().expect("placed chain was counted").rank;
            // Same rule and walk as pass 1, so the classification agrees.
            if !seen[rank] || Some(chain) == prev {
                seen[rank] = true;
                prev = Some(chain);
                let at = cursors[rank].1;
                cursors[rank].1 += 1;
                final_order[at] = slot;
            } else {
                cursors[rank].0 -= 1;
                let at = cursors[rank].0;
                final_order[at] = slot;
            }
        }
        debug_assert!(final_order.iter().all(|slot| *slot != NO_SLOT));
        // Pass 3: relay out slots in regrouped order, rebuild lookups.
        let mut new_slots: Vec<CxxSlot<V>> = Vec::with_capacity(final_order.len());
        for (new_idx, old_slot) in final_order.iter().enumerate() {
            let node = &mut self.slots[*old_slot];
            new_slots.push(CxxSlot {
                key: node.key,
                value: Some(node.value.take().expect("regroup walks live slots")),
                prev: if new_idx == 0 { NO_SLOT } else { new_idx - 1 },
                next: if new_idx + 1 == final_order.len() {
                    NO_SLOT
                } else {
                    new_idx + 1
                },
            });
        }
        self.slots = new_slots;
        self.free.clear();
        self.head = 0;
        self.index.clear();
        for (new_idx, node) in self.slots.iter().enumerate() {
            self.index.insert(node.key, new_idx);
        }
        for (rank, chain) in chain_order.iter().enumerate() {
            self.chain_head[*chain] = Some(base_of_rank[rank]);
        }
    }

    /// Fresh-key insert with the C++ order effects (rehash, then
    /// front-of-chain placement). The key must be absent. Returns the slot
    /// the key landed on.
    fn insert_fresh(&mut self, key: usize, value: V) -> usize {
        debug_assert!(!self.index.contains_key(&key));
        self.grow_rehash_if_needed();
        let chain = cxx_constrain_hash(key, self.buckets);
        let slot = self.alloc_slot(key, value);
        if let Some(head_slot) = self.chain_head[chain] {
            self.link_before(slot, head_slot);
        } else {
            self.link_front(slot);
        }
        self.chain_head[chain] = Some(slot);
        self.index.insert(key, slot);
        slot
    }

    /// C++ `insert`/`emplace`: keeps the old value (returns `false`) when
    /// the key is present.
    fn insert_new(&mut self, key: usize, value: V) -> bool {
        if self.index.contains_key(&key) {
            return false;
        }
        self.insert_fresh(key, value);
        true
    }

    /// C++ `operator[] = value`: overwrites when present (no order change),
    /// inserts (with the same order effects) when absent.
    #[cfg_attr(not(test), allow(dead_code))] // container oracle tests
    fn set(&mut self, key: usize, value: V) {
        if let Some(&slot) = self.index.get(&key) {
            self.slots[slot].value = Some(value);
            return;
        }
        self.insert_fresh(key, value);
    }

    /// C++ `operator[]` for read-mutate: default-inserts on absence.
    fn get_or_default(&mut self, key: usize) -> &mut V
    where
        V: Default,
    {
        if !self.index.contains_key(&key) {
            self.insert_fresh(key, V::default());
        }
        let slot = self.index[&key];
        self.live_value_mut(slot)
    }

    fn remove(&mut self, key: &usize) -> Option<V> {
        let slot = self.index.remove(key)?;
        let chain = cxx_constrain_hash(*key, self.buckets);
        let next = self.slots[slot].next;
        self.unlink(slot);
        // Runs stay contiguous across unlink, so when the erased node led
        // its chain the successor (iff it shares the chain) is the new
        // head; otherwise the chain just lost its only node.
        if self.chain_head[chain] == Some(slot) {
            let next_shares =
                next != NO_SLOT && cxx_constrain_hash(self.slots[next].key, self.buckets) == chain;
            self.chain_head[chain] = if next_shares { Some(next) } else { None };
        }
        let value = self.slots[slot].value.take();
        self.free.push(slot);
        value
    }

    /// `clear`: empties the table but keeps the bucket count.
    #[cfg_attr(not(test), allow(dead_code))] // container oracle tests
    fn clear(&mut self) {
        self.slots.clear();
        self.free.clear();
        self.head = NO_SLOT;
        self.index.clear();
        self.chain_head.fill(None);
    }

    /// First key in list order (`nextMap.begin()`), or `None` when empty.
    fn first_key(&self) -> Option<&usize> {
        if self.head == NO_SLOT {
            None
        } else {
            Some(&self.slots[self.head].key)
        }
    }
}

/// Borrowed keys in list order.
struct CxxSetIter<'a> {
    slots: &'a [CxxSlot<()>],
    cursor: usize,
    remaining: usize,
}

impl<'a> Iterator for CxxSetIter<'a> {
    type Item = &'a usize;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 || self.cursor == NO_SLOT {
            return None;
        }
        let node = &self.slots[self.cursor];
        self.cursor = node.next;
        self.remaining -= 1;
        Some(&node.key)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl<'a> ExactSizeIterator for CxxSetIter<'a> {}

/// Borrowed `(key, value)` pairs in list order.
struct CxxMapIter<'a, V> {
    slots: &'a [CxxSlot<V>],
    cursor: usize,
    remaining: usize,
}

impl<'a, V> Iterator for CxxMapIter<'a, V> {
    type Item = (&'a usize, &'a V);

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 || self.cursor == NO_SLOT {
            return None;
        }
        let node = &self.slots[self.cursor];
        self.cursor = node.next;
        self.remaining -= 1;
        Some((
            &node.key,
            node.value.as_ref().expect("listed slot holds a value"),
        ))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl<'a, V> ExactSizeIterator for CxxMapIter<'a, V> {}

/// Emulated `std::unordered_set<size_t>` (see [`CxxTable`]): iteration
/// yields list order, like the C++ iterators.
#[derive(Clone, Debug, Default)]
struct CxxSet {
    table: CxxTable<()>,
}

impl CxxSet {
    fn new() -> Self {
        Self {
            table: CxxTable::new(),
        }
    }

    fn len(&self) -> usize {
        self.table.len()
    }

    fn is_empty(&self) -> bool {
        self.table.is_empty()
    }

    fn contains(&self, key: &usize) -> bool {
        self.table.contains_key(key)
    }

    /// C++ `insert`: no-op (returns `false`) when the key is present.
    fn insert(&mut self, key: usize) -> bool {
        self.table.insert_new(key, ())
    }

    fn remove(&mut self, key: &usize) -> bool {
        self.table.remove(key).is_some()
    }

    /// `clear`: empties the table but keeps the bucket count.
    #[cfg_attr(not(test), allow(dead_code))] // container oracle tests
    fn clear(&mut self) {
        self.table.clear();
    }

    fn iter(&self) -> CxxSetIter<'_> {
        CxxSetIter {
            slots: &self.table.slots,
            cursor: self.table.head,
            remaining: self.table.len(),
        }
    }

    /// Iteration order as a vector (test-only inspection).
    #[cfg(test)]
    fn order_vec(&self) -> Vec<usize> {
        self.iter().copied().collect()
    }

    /// Bucket count (test-only inspection).
    #[cfg(test)]
    fn buckets(&self) -> usize {
        self.table.buckets()
    }
}

impl<'a> IntoIterator for &'a CxxSet {
    type Item = &'a usize;
    type IntoIter = CxxSetIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl FromIterator<usize> for CxxSet {
    /// Range/iterator insert: sequential per-element inserts in order.
    fn from_iter<I: IntoIterator<Item = usize>>(iter: I) -> Self {
        let mut set = Self::new();
        set.extend(iter);
        set
    }
}

impl<const N: usize> From<[usize; N]> for CxxSet {
    fn from(values: [usize; N]) -> Self {
        values.into_iter().collect()
    }
}

impl Extend<usize> for CxxSet {
    fn extend<I: IntoIterator<Item = usize>>(&mut self, iter: I) {
        for key in iter {
            self.insert(key);
        }
    }
}

/// Emulated `std::unordered_map<size_t, V>` (see [`CxxTable`]).
/// There is deliberately no `insert` or `entry` (see the section note);
/// every write names its C++ counterpart.
#[derive(Clone, Debug, Default)]
struct CxxMap<V> {
    table: CxxTable<V>,
}

impl<V> CxxMap<V> {
    fn new() -> Self {
        Self {
            table: CxxTable::new(),
        }
    }

    #[cfg_attr(not(test), allow(dead_code))] // container oracle tests
    fn len(&self) -> usize {
        self.table.len()
    }

    fn is_empty(&self) -> bool {
        self.table.is_empty()
    }

    fn contains_key(&self, key: &usize) -> bool {
        self.table.contains_key(key)
    }

    fn get(&self, key: &usize) -> Option<&V> {
        self.table.get(key)
    }

    fn get_mut(&mut self, key: &usize) -> Option<&mut V> {
        self.table.get_mut(key)
    }

    /// C++ `insert`/`emplace`: keeps the old value (returns `false`) when
    /// the key is present.
    fn insert_new(&mut self, key: usize, value: V) -> bool {
        self.table.insert_new(key, value)
    }

    /// C++ `operator[] = value`: overwrites when present, inserts (with the
    /// same order effects) when absent.
    #[cfg_attr(not(test), allow(dead_code))] // container oracle tests
    fn set(&mut self, key: usize, value: V) {
        self.table.set(key, value)
    }

    /// C++ `operator[]` for read-mutate: default-inserts on absence.
    fn get_or_default(&mut self, key: usize) -> &mut V
    where
        V: Default,
    {
        self.table.get_or_default(key)
    }

    fn remove(&mut self, key: &usize) -> Option<V> {
        self.table.remove(key)
    }

    /// `clear`: empties the table but keeps the bucket count.
    #[cfg_attr(not(test), allow(dead_code))] // container oracle tests
    fn clear(&mut self) {
        self.table.clear()
    }

    fn iter(&self) -> CxxMapIter<'_, V> {
        CxxMapIter {
            slots: &self.table.slots,
            cursor: self.table.head,
            remaining: self.table.len(),
        }
    }

    /// First key in list order (`nextMap.begin()`), or `None` when empty.
    fn first_key(&self) -> Option<&usize> {
        self.table.first_key()
    }

    /// Iteration order as a vector (test-only inspection).
    #[cfg(test)]
    fn order_vec(&self) -> Vec<(usize, V)>
    where
        V: Clone,
    {
        self.iter()
            .map(|(key, value)| (*key, value.clone()))
            .collect()
    }

    /// Bucket count (test-only inspection).
    #[cfg(test)]
    fn buckets(&self) -> usize {
        self.table.buckets()
    }
}

impl<'a, V> IntoIterator for &'a CxxMap<V> {
    type Item = (&'a usize, &'a V);
    type IntoIter = CxxMapIter<'a, V>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<V> FromIterator<(usize, V)> for CxxMap<V> {
    /// Range/iterator insert: sequential per-element inserts in order.
    fn from_iter<I: IntoIterator<Item = (usize, V)>>(iter: I) -> Self {
        let mut map = Self::new();
        map.extend(iter);
        map
    }
}

impl<V> Extend<(usize, V)> for CxxMap<V> {
    fn extend<I: IntoIterator<Item = (usize, V)>>(&mut self, iter: I) {
        for (key, value) in iter {
            self.insert_new(key, value);
        }
    }
}

// ---------------------------------------------------------------------------
// Sorted per-round indexes for the fixpoint passes.
// ---------------------------------------------------------------------------
//
// The collapse/merge/cleanup passes rebuild adjacency maps over all faces
// every round. `BTreeMap` builds malloc per entry (150k+ allocs/round at
// 50k faces); these sorted-vector indexes collect (key, value) pairs in
// face order, sort once, and group — identical keys, values, and order to
// the `BTreeMap` builds (stable sort over face-ordered pairs keeps
// face-index vecs ascending; neighbor pairs dedup to the same sets),
// with a handful of allocations per round. Lookups are binary searches
// with identical hit/miss/value behavior. Equivalence is proved by
// `fixpoint_indexes_match_btree` plus the pipeline bitwise checks.

/// Sorted `(edge, face)` groups: identical keys, face-index vecs
/// (ascending face order), and group order to the per-round
/// `BTreeMap<(usize, usize), Vec<usize>>` build.
struct EdgeFaceIndex {
    edges: Vec<(usize, usize)>,
    starts: Vec<usize>,
    faces: Vec<usize>,
}

impl EdgeFaceIndex {
    fn build(polygons: &[Vec<usize>]) -> Self {
        let mut pairs: Vec<((usize, usize), usize)> = Vec::new();
        for (face_index, face) in polygons.iter().enumerate() {
            for i in 0..face.len() {
                pairs.push((
                    QuadExtractor::edge_of(face[i], face[(i + 1) % face.len()]),
                    face_index,
                ));
            }
        }
        // Stable: pairs start in face order, so each edge's group keeps
        // ascending face order exactly like the serial `push` build.
        pairs.sort_by(|a, b| a.0.cmp(&b.0));
        let mut edges = Vec::new();
        let mut starts = Vec::new();
        let mut faces = Vec::new();
        let mut i = 0;
        while i < pairs.len() {
            let edge = pairs[i].0;
            edges.push(edge);
            starts.push(faces.len());
            while i < pairs.len() && pairs[i].0 == edge {
                faces.push(pairs[i].1);
                i += 1;
            }
        }
        starts.push(faces.len());
        Self {
            edges,
            starts,
            faces,
        }
    }

    /// Faces incident to `edge` in ascending face order (`None` when absent).
    fn get(&self, edge: &(usize, usize)) -> Option<&[usize]> {
        let group = self.edges.binary_search(edge).ok()?;
        Some(&self.faces[self.starts[group]..self.starts[group + 1]])
    }

    /// `(edge, faces)` groups in ascending edge order.
    fn groups(&self) -> impl Iterator<Item = ((usize, usize), &[usize])> + '_ {
        self.edges.iter().enumerate().map(|(group, edge)| {
            (
                *edge,
                &self.faces[self.starts[group]..self.starts[group + 1]],
            )
        })
    }
}

/// Sorted `(vertex, neighbor)` groups, deduped: identical keys and
/// ascending neighbor order to the per-round
/// `BTreeMap<usize, BTreeSet<usize>>` build.
struct NeighborIndex {
    verts: Vec<usize>,
    starts: Vec<usize>,
    neighbors: Vec<usize>,
}

impl NeighborIndex {
    fn build(polygons: &[Vec<usize>]) -> Self {
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for face in polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                pairs.push((face[i], face[j]));
                pairs.push((face[j], face[i]));
            }
        }
        pairs.sort();
        pairs.dedup();
        let mut verts = Vec::new();
        let mut starts = Vec::new();
        let mut neighbors = Vec::new();
        let mut i = 0;
        while i < pairs.len() {
            let vert = pairs[i].0;
            verts.push(vert);
            starts.push(neighbors.len());
            while i < pairs.len() && pairs[i].0 == vert {
                neighbors.push(pairs[i].1);
                i += 1;
            }
        }
        starts.push(neighbors.len());
        Self {
            verts,
            starts,
            neighbors,
        }
    }

    /// Neighbors of `vertex` in ascending order (`None` when absent).
    fn get(&self, vertex: &usize) -> Option<&[usize]> {
        let group = self.verts.binary_search(vertex).ok()?;
        Some(&self.neighbors[self.starts[group]..self.starts[group + 1]])
    }

    /// Neighbors of `vertex`, or an empty slice when absent (every face
    /// vertex is present by construction; the empty case is unreachable
    /// but total, unlike `BTreeMap` indexing).
    fn get_or_empty(&self, vertex: &usize) -> &[usize] {
        self.get(vertex).unwrap_or(&[])
    }
}

/// Sorted `(vertex, face count)` runs: identical keys and counts to the
/// per-round `BTreeMap<usize, usize>` vertex-face-count build.
struct FaceCountIndex {
    verts: Vec<usize>,
    counts: Vec<usize>,
}

impl FaceCountIndex {
    fn build(polygons: &[Vec<usize>]) -> Self {
        let mut verts: Vec<usize> = Vec::new();
        for face in polygons {
            verts.extend(face.iter().copied());
        }
        verts.sort_unstable();
        let mut keys = Vec::new();
        let mut counts = Vec::new();
        let mut i = 0;
        while i < verts.len() {
            let vert = verts[i];
            let mut count = 0;
            while i < verts.len() && verts[i] == vert {
                count += 1;
                i += 1;
            }
            keys.push(vert);
            counts.push(count);
        }
        Self {
            verts: keys,
            counts,
        }
    }

    fn get(&self, vertex: &usize) -> Option<&usize> {
        let slot = self.verts.binary_search(vertex).ok()?;
        Some(&self.counts[slot])
    }
}

// ---------------------------------------------------------------------------
// QuadExtractor.
// ---------------------------------------------------------------------------

/// The isoline a connection was cut from: which uv coordinate is held
/// constant, which integer value it is held at, and which triangle produced
/// the segment.
#[derive(Clone, Copy, Debug)]
struct ConnectionInfo {
    triangle_index: usize,
    coord_index: i32,
    integer: i32,
}

/// Quad extractor (mirrors `AutoRemesher::QuadExtractor`).
///
/// Holds borrowed inputs (the C++ holds pointers) and owned outputs.
/// Nullable inputs (null in C++) are `Option`s that stay `None` unless the
/// corresponding setter ran.
pub struct QuadExtractor<'a> {
    vertices: &'a [Vector3],
    triangles: &'a [Vec<usize>],
    triangle_uvs: &'a [Vec<Vector2>],
    remeshed_vertices: Vec<Vector3>,
    remeshed_polygons: Vec<Vec<usize>>,
    remeshed_vertex_uvs: Vec<Vector2>,
    compute_vertex_uvs: bool,
    extracted_connections: Vec<(Vector3, Vector3)>,
    extracted_connection_moved: Vec<u8>,
    original_triangle_uvs: Option<&'a [Vec<Vector2>]>,
    singular_vertices: Option<&'a [usize]>,
    progress_handler: Option<ProgressHandler>,
    connection_infos: BTreeMap<(usize, usize), ConnectionInfo>,
    added_connections: BTreeSet<(usize, usize)>,
    half_edges: BTreeSet<(usize, usize)>,
}

/// A walked cleanup-ladder route: the rungs, the dissolved faces, and the
/// sink face (`usize::MAX` for a border sink).
type CleanupRoute = (Vec<(usize, usize)>, BTreeSet<usize>, usize);

impl<'a> QuadExtractor<'a> {
    #[must_use]
    pub fn new(
        vertices: &'a [Vector3],
        triangles: &'a [Vec<usize>],
        triangle_uvs: &'a [Vec<Vector2>],
    ) -> Self {
        Self {
            vertices,
            triangles,
            triangle_uvs,
            remeshed_vertices: Vec::new(),
            remeshed_polygons: Vec::new(),
            remeshed_vertex_uvs: Vec::new(),
            compute_vertex_uvs: false,
            extracted_connections: Vec::new(),
            extracted_connection_moved: Vec::new(),
            original_triangle_uvs: None,
            singular_vertices: None,
            progress_handler: None,
            connection_infos: BTreeMap::new(),
            added_connections: BTreeSet::new(),
            half_edges: BTreeSet::new(),
        }
    }

    #[must_use]
    pub fn remeshed_vertices(&self) -> &[Vector3] {
        &self.remeshed_vertices
    }

    #[must_use]
    pub fn remeshed_quads(&self) -> &[Vec<usize>] {
        &self.remeshed_polygons
    }

    /// Per-output-vertex UVs interpolated from the input parameterization,
    /// normalized to 0..1 over this island's UV bounding box. Only computed
    /// when [`Self::set_compute_vertex_uvs`] ran before [`Self::extract`];
    /// otherwise empty. Computing them never alters geometry: a pure
    /// post-pass over the final positions.
    #[must_use]
    pub fn remeshed_vertex_uvs(&self) -> &[Vector2] {
        &self.remeshed_vertex_uvs
    }

    pub fn set_compute_vertex_uvs(&mut self, compute: bool) {
        self.compute_vertex_uvs = compute;
    }

    /// The raw connections produced by `extract_connections`, before graph
    /// cleanup.
    #[must_use]
    pub fn extracted_connections(&self) -> &[(Vector3, Vector3)] {
        &self.extracted_connections
    }

    pub fn set_original_triangle_uvs(&mut self, original_triangle_uvs: &'a [Vec<Vector2>]) {
        self.original_triangle_uvs = Some(original_triangle_uvs);
    }

    pub fn set_singular_vertices(&mut self, singular_vertices: &'a [usize]) {
        self.singular_vertices = Some(singular_vertices);
    }

    pub fn set_progress_handler(&mut self, progress_handler: ProgressHandler) {
        self.progress_handler = Some(progress_handler);
    }

    /// Per connection of [`Self::extracted_connections`]: 0 untouched, 1 on
    /// a triangle whose uv was repaired, 2 added by `hold_singular_lines`.
    #[must_use]
    pub fn extracted_connection_moved(&self) -> &[u8] {
        &self.extracted_connection_moved
    }

    fn report(&self, fraction: f32, name: &str) {
        if let Some(handler) = &self.progress_handler {
            handler(fraction, name);
        }
    }

    fn diagnose(&self, build: impl FnOnce() -> String) {
        // Every std::cerr diagnostic in extract() and its helpers is gated
        // on m_progressHandler: with no progress subscriber (a quiet run)
        // the extractor stays silent. Failures propagate via return values.
        // The closure keeps it zero-cost when unset, like the C++ `if`.
        if self.progress_handler.is_some() {
            eprint!("{}", build());
        }
    }

    pub fn extract(&mut self) -> bool {
        // The fractions are the measured share of extraction each step
        // costs. The topology cleanup passes at the end are over half of
        // it, so they report individually instead of as one long silent
        // block.
        self.report(0.0, "Extracting connections");
        self.diagnose(|| "Extract connections...\n".to_string());
        let mut cross_points = Vec::new();
        let mut cross_point_source_triangles = Vec::new();
        let mut connections = BTreeSet::new();
        self.extract_connections(
            &mut cross_points,
            &mut cross_point_source_triangles,
            &mut connections,
        );
        self.report(0.07, "Holding singular lines");
        self.hold_singular_lines(
            &mut cross_points,
            &mut cross_point_source_triangles,
            &mut connections,
        );
        self.extracted_connections.clear();
        self.extracted_connection_moved.clear();
        self.extracted_connections.reserve(connections.len());
        let mut triangle_moved = Vec::new();
        if let Some(original) = self.original_triangle_uvs
            && original.len() == self.triangle_uvs.len()
        {
            triangle_moved = vec![0u8; self.triangle_uvs.len()];
            for (i, moved) in triangle_moved.iter_mut().enumerate() {
                let before = &original[i];
                let after = &self.triangle_uvs[i];
                for k in 0..3 {
                    if k >= before.len() || k >= after.len() {
                        break;
                    }
                    if before[k].x() != after[k].x() || before[k].y() != after[k].y() {
                        *moved = 1;
                        break;
                    }
                }
            }
        }
        self.extracted_connection_moved.reserve(connections.len());
        for (first, second) in &connections {
            let first = *first;
            let second = *second;
            self.extracted_connections
                .push((cross_points[first], cross_points[second]));
            let edge = (first.min(second), first.max(second));
            if self.added_connections.contains(&edge) {
                self.extracted_connection_moved.push(2);
            } else if !triangle_moved.is_empty() {
                let first_triangle = cross_point_source_triangles[first];
                let second_triangle = cross_point_source_triangles[second];
                self.extracted_connection_moved.push(u8::from(
                    triangle_moved[first_triangle] != 0 || triangle_moved[second_triangle] != 0,
                ));
            } else {
                self.extracted_connection_moved.push(0);
            }
        }
        self.diagnose(|| "Extract connections done\n".to_string());

        self.report(0.21, "Extracting edges");
        self.diagnose(|| "Extract edges...\n".to_string());
        let mut edge_connect_map: CxxMap<CxxSet> = CxxMap::new();
        Self::extract_edges(&connections, &mut edge_connect_map);
        if Self::collapse_short_edges(&mut cross_points, &mut edge_connect_map) {
            Self::simplify_graph(&mut edge_connect_map);
        }
        Self::collapse_triangles(&mut cross_points, &mut edge_connect_map);
        if Self::remove_single_endpoints(&mut cross_points, &mut edge_connect_map) {
            Self::simplify_graph(&mut edge_connect_map);
        }
        self.diagnose(|| "Extract edges done\n".to_string());

        self.report(0.25, "Extracting mesh");
        self.diagnose(|| "Extract mesh...\n".to_string());
        self.extract_mesh(cross_points, cross_point_source_triangles, edge_connect_map);
        self.diagnose(|| "Extract mesh done\n".to_string());

        self.report(0.29, "Fixing holes");
        self.fix_holes();

        self.report(0.30, "Removing non-manifold faces");
        let mut changed = false;
        if self.remove_isolated_faces() {
            changed = true;
        }
        while self.remove_non_manifold_faces() {
            changed = true;
            self.remove_isolated_faces();
        }

        if changed {
            self.rebuild_half_edges();
            self.fix_holes();
        }

        {
            let mut used_vertices = BTreeSet::new();
            for face in &self.remeshed_polygons {
                for v in face {
                    used_vertices.insert(*v);
                }
            }
            if used_vertices.len() < self.remeshed_vertices.len() {
                let mut compacted_vertices = Vec::with_capacity(used_vertices.len());
                let mut old_to_new = BTreeMap::new();
                for old_index in &used_vertices {
                    old_to_new.insert(*old_index, compacted_vertices.len());
                    compacted_vertices.push(self.remeshed_vertices[*old_index]);
                }
                for face in &mut self.remeshed_polygons {
                    for v in face {
                        *v = old_to_new[v];
                    }
                }
                self.remeshed_vertices = compacted_vertices;
            }
        }

        self.report(0.31, "Smoothing and projecting");
        self.diagnose(|| "Smooth and project...\n".to_string());
        self.smooth_and_project(5, None);
        self.diagnose(|| "Smooth and project done\n".to_string());

        self.report(0.44, "Splitting seven edge faces");
        self.split_seven_edge_faces();
        self.report(0.45, "Splitting six edge faces");
        self.split_six_edge_faces();
        // A pentagon is the best place for a triangle to end up, it comes
        // out of the collapse as a quad, so the triangles run first and the
        // merge takes care of whatever pentagons are left over
        self.report(0.46, "Cleaning up triangles");
        self.cleanup_triangles();
        self.report(0.53, "Merging shared five edge faces");
        // Restructure: the C++ builds a remapping closure borrowing the
        // outer handler while calling a `&mut self` method; the mirror
        // parks the outer handler in an `Arc` (still `Send + Sync`, still
        // the same handler object), calls through a shared clone, then
        // restores it. Same fractions reach the same handler in the same
        // order.
        let outer_progress = self.progress_handler.take();
        if let Some(outer) = outer_progress {
            let shared: Arc<ProgressHandler> = Arc::new(outer);
            let inner = Arc::clone(&shared);
            // FMA: `0.53 + (0.85 - 0.53) * fraction` is `fmaf` in f32.
            let remapped: ProgressHandler = Box::new(move |fraction, name| {
                inner((0.85f32 - 0.53f32).mul_add(fraction, 0.53f32), name);
            });
            self.merge_shared_five_edge_faces(Some(&remapped));
            drop(remapped);
            // `remapped` held the only other clone, so the `Arc` is uniquely
            // owned again; restore the exact same handler object.
            if let Ok(outer) = Arc::try_unwrap(shared) {
                self.progress_handler = Some(outer);
            }
        } else {
            self.merge_shared_five_edge_faces(None);
        }
        // Runs last, it only reconnects quad pairs, so it wants the
        // triangles and pentagons to have become quads already
        self.report(0.85, "Switching high valence edges");
        self.switch_high_valence_edges();
        self.report(0.89, "Converting triangle and five edge fans");
        self.convert_triangle_and_five_edge_fans();
        self.report(0.93, "Collapsing three valence diagonals");
        self.collapse_three_valence_diagonals();
        self.report(0.95, "Merging double shared edge quads");
        self.merge_double_shared_edge_quads();
        self.report(0.96, "Merging three and five valence triangles");
        self.merge_three_and_five_valence_triangles();
        self.report(0.97, "Collapsing three valence corners");
        self.collapse_three_valence_corners();
        self.report(0.98, "Splitting high valence triangle fans");
        self.split_high_valence_triangle_fans();
        self.report(0.99, "Collapsing three valence edge pairs");
        self.collapse_three_valence_edge_pairs();
        // No progress event (the engine oracle pins the sequence): a fast
        // tail sweep, silent unless defects were actually removed.
        self.cleanup_residual_routes();
        self.report(1.0, "");

        // Pure post-pass over the final positions: reads m_remeshedVertices,
        // never writes it, so geometry is identical with the flag on or off.
        if self.compute_vertex_uvs {
            self.compute_remeshed_vertex_uvs();
        }

        true
    }

    fn extract_edges(
        connections: &BTreeSet<(usize, usize)>,
        edge_connect_map: &mut CxxMap<CxxSet>,
    ) {
        for (first, second) in connections {
            edge_connect_map.get_or_default(*first).insert(*second);
            edge_connect_map.get_or_default(*second).insert(*first);
        }
        Self::simplify_graph(edge_connect_map);
    }

    fn simplify_graph(graph: &mut CxxMap<CxxSet>) {
        loop {
            let mut delay_pairs: CxxMap<(usize, usize)> = CxxMap::new();
            // Restructure: the C++ erases vertices while iterating the map.
            // Erasure only unlinks (libc++ `remove` keeps survivors in
            // order), and rewiring happens after the scan, so scanning a
            // snapshot and erasing as the scan goes visits the same
            // vertices with the same neighbor pairs in the same emulated
            // order as the C++ visit-and-erase.
            let mut snapshot = Vec::new();
            for (vertex, neighbors) in graph.iter() {
                if neighbors.len() != 2 {
                    continue;
                }
                let mut it = neighbors.iter();
                // Invariant: two neighbors (checked above); the fallback is
                // unreachable.
                if let (Some(first), Some(second)) = (it.next(), it.next()) {
                    snapshot.push((*vertex, (*first, *second)));
                }
            }
            for (vertex, (first_neighbor, second_neighbor)) in snapshot {
                if delay_pairs.contains_key(&first_neighbor)
                    || delay_pairs.contains_key(&second_neighbor)
                {
                    continue;
                }
                // C++ `insert`: each vertex is visited (hence scheduled) at
                // most once per round, so keep-old and overwrite agree.
                delay_pairs.insert_new(vertex, (first_neighbor, second_neighbor));
                graph.remove(&vertex);
            }
            if delay_pairs.is_empty() {
                break;
            }
            for (vertex, (first, second)) in &delay_pairs {
                if let Some(neighbors) = graph.get_mut(first) {
                    neighbors.remove(vertex);
                    neighbors.insert(*second);
                } else {
                    // C++ `operator[]` then erase (no-op on the fresh set)
                    // then insert: the key is absent, so build it directly.
                    graph.insert_new(*first, CxxSet::from([*second]));
                }
                if let Some(neighbors) = graph.get_mut(second) {
                    neighbors.remove(vertex);
                    neighbors.insert(*first);
                } else {
                    graph.insert_new(*second, CxxSet::from([*first]));
                }
            }
        }
    }

    fn remove_single_endpoints(
        _cross_points: &mut [Vector3],
        edge_connect_map: &mut CxxMap<CxxSet>,
    ) -> bool {
        let mut removed = false;
        let mut endpoints = Vec::new();
        for (vertex, neighbors) in edge_connect_map.iter() {
            if neighbors.len() != 1 {
                continue;
            }
            endpoints.push(*vertex);
        }
        for endpoint in endpoints {
            let mut loop_index = endpoint;
            while let Some(neighbors) = edge_connect_map.get(&loop_index) {
                if neighbors.len() != 1 {
                    break;
                }
                // Invariant: exactly one neighbor (checked above).
                let Some(neighbor) = neighbors.iter().next().copied() else {
                    break;
                };
                edge_connect_map.remove(&loop_index);
                removed = true;
                if let Some(neighbor_set) = edge_connect_map.get_mut(&neighbor) {
                    neighbor_set.remove(&loop_index);
                } else {
                    break;
                }
                loop_index = neighbor;
            }
        }
        removed
    }

    fn collapse_triangles(
        cross_points: &mut [Vector3],
        edge_connect_map: &mut CxxMap<CxxSet>,
    ) -> bool {
        let mut triangles = BTreeSet::new();
        for (level0, neighbors) in edge_connect_map.iter() {
            let level0 = *level0;
            for level1 in neighbors {
                let Some(find_level2) = edge_connect_map.get(level1) else {
                    continue;
                };
                for level2 in find_level2 {
                    if level0 == *level2 {
                        continue;
                    }
                    let Some(find_level3) = edge_connect_map.get(level2) else {
                        continue;
                    };
                    if !find_level3.contains(&level0) {
                        continue;
                    }
                    let mut sorted = [level0, *level1, *level2];
                    sorted.sort_unstable();
                    triangles.insert((sorted[0], sorted[1], sorted[2]));
                }
            }
        }

        if triangles.is_empty() {
            return false;
        }

        // Collapse one edge per triangle, the shortest one. Merging every
        // connected triangle into a single point instead would drag all of
        // their neighbors onto that point and leave a star of slivers
        // behind it
        let mut collapsed = false;
        for triangle in &triangles {
            let corners = [triangle.0, triangle.1, triangle.2];
            let mut shortest_edge = (0, 0);
            let mut shortest_length = f64::MAX;
            let mut still_a_triangle = true;
            for i in 0..3 {
                let j = (i + 1) % 3;
                match edge_connect_map.get(&corners[i]) {
                    Some(neighbors) if neighbors.contains(&corners[j]) => {}
                    _ => {
                        still_a_triangle = false;
                        break;
                    }
                }
                let length = (cross_points[corners[i]] - cross_points[corners[j]]).length();
                if length < shortest_length {
                    shortest_length = length;
                    shortest_edge = (corners[i], corners[j]);
                }
            }
            // An earlier collapse may have already taken this one apart
            if !still_a_triangle {
                continue;
            }
            Self::collapse_edge(cross_points, edge_connect_map, shortest_edge);
            collapsed = true;
        }

        collapsed
    }

    fn collapse_short_edges(
        cross_points: &mut [Vector3],
        edge_connect_map: &mut CxxMap<CxxSet>,
    ) -> bool {
        let mut total_length = 0.0;
        let mut edge_count = 0;
        let mut edge_lengths = BTreeMap::new();
        for (point, neighbors) in edge_connect_map.iter() {
            for neighbor in neighbors {
                if edge_lengths.contains_key(&(*neighbor, *point)) {
                    continue;
                }
                let edge_length = (cross_points[*point] - cross_points[*neighbor]).length();
                total_length += edge_length;
                edge_lengths.insert((*point, *neighbor), edge_length);
                edge_count += 1;
            }
        }
        if 0 == edge_count {
            return false;
        }
        let average_edge_length = total_length / edge_count as f64;
        let collapsed_length = average_edge_length * 0.01;
        let mut collapsed = false;
        for (edge, length) in &edge_lengths {
            if *length > collapsed_length {
                continue;
            }
            Self::collapse_edge(cross_points, edge_connect_map, *edge);
            collapsed = true;
        }
        collapsed
    }

    fn collapse_edge(
        cross_points: &mut [Vector3],
        edge_connect_map: &mut CxxMap<CxxSet>,
        edge: (usize, usize),
    ) {
        let has_forward = edge_connect_map
            .get(&edge.1)
            .is_some_and(|neighbors| neighbors.contains(&edge.0));
        if !has_forward {
            return;
        }
        let has_backward = edge_connect_map
            .get(&edge.0)
            .is_some_and(|neighbors| neighbors.contains(&edge.1));
        if !has_backward {
            return;
        }
        // Copy of a non-empty set (it holds `edge.1`, checked above), so
        // the copy keeps its order and count on both sides.
        let Some(first_neighbors) = edge_connect_map.get(&edge.0).cloned() else {
            return;
        };
        cross_points[edge.1] = (cross_points[edge.0] + cross_points[edge.1]) * 0.5;
        for neighbor in &first_neighbors {
            if *neighbor == edge.1 {
                continue;
            }
            edge_connect_map.get_or_default(edge.1).insert(*neighbor);
            edge_connect_map.get_or_default(*neighbor).insert(edge.1);
            edge_connect_map.get_or_default(*neighbor).remove(&edge.0);
        }
        edge_connect_map.remove(&edge.0);
        edge_connect_map.get_or_default(edge.1).remove(&edge.0);
        let second_empty = edge_connect_map
            .get(&edge.1)
            .is_some_and(|set| set.is_empty());
        if second_empty {
            edge_connect_map.remove(&edge.1);
        }
    }

    fn ring_face_normal(points: &[Vector3], corners: &[usize]) -> Vector3 {
        let mut center = Vector3::default();
        for corner in corners {
            center += points[*corner];
        }
        center /= corners.len() as f64;
        let mut normals = Vector3::default();
        for i in 0..corners.len() {
            normals += Vector3::normal(
                &points[corners[i % corners.len()]],
                &points[corners[(i + 1) % corners.len()]],
                &center,
            );
        }
        normals.normalized()
    }

    fn ring_side(points: &[Vector3], triangle_normals: &CxxMap<Vector3>, corners: &[usize]) -> i32 {
        let ring_normal = Self::ring_face_normal(points, corners);
        let mut original_normal = Vector3::default();
        for it in corners {
            // C++ `operator[]` read: every corner is present (the map is
            // built over all points), and the table is never iterated, so
            // the default-on-miss value below is exactly equivalent.
            original_normal += triangle_normals.get(it).copied().unwrap_or_default();
        }
        let dot = Vector3::dot_product(&ring_normal, &original_normal.normalized());
        const DOT_THRESHOLD: f64 = 0.259; // > 75 or < 105 degrees
        if dot > DOT_THRESHOLD {
            1
        } else if dot < -DOT_THRESHOLD {
            -1
        } else {
            0
        }
    }

    /// One face-emission attempt (mirrors the five copy-pasted
    /// corner/side/halfedge blocks of `extractMesh`, one per face size):
    /// skip used corners, orient by ring side, skip used halfedges, then
    /// record the face, its corners (both windings) and its halfedges.
    fn try_add_face(
        points: &[Vector3],
        triangle_normals: &CxxMap<Vector3>,
        corners: &mut BTreeSet<(usize, usize, usize)>,
        half_edges: &mut BTreeSet<(usize, usize)>,
        quads: &mut Vec<Vec<usize>>,
        face: &[usize],
    ) {
        if Self::face_corner_exists(corners, face) {
            return;
        }
        let side = Self::ring_side(points, triangle_normals, face);
        if side > 0 {
            if Self::face_half_edge_exists(half_edges, face) {
                return;
            }
            quads.push(face.to_vec());
            Self::add_face_corners(corners, face);
            Self::add_face_half_edges(half_edges, face);
        } else if side < 0 {
            let reversed: Vec<usize> = face.iter().rev().copied().collect();
            if Self::face_half_edge_exists(half_edges, &reversed) {
                return;
            }
            quads.push(reversed.clone());
            Self::add_face_corners(corners, &reversed);
            Self::add_face_half_edges(half_edges, &reversed);
        }
    }

    fn corner_used(
        corners: &BTreeSet<(usize, usize, usize)>,
        previous: usize,
        current: usize,
        next: usize,
    ) -> bool {
        if corners.contains(&(previous, current, next)) {
            return true;
        }
        if corners.contains(&(next, current, previous)) {
            return true;
        }
        false
    }

    fn face_corner_exists(corners: &BTreeSet<(usize, usize, usize)>, vertices: &[usize]) -> bool {
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            let k = (i + 2) % vertices.len();
            if Self::corner_used(corners, vertices[i], vertices[j], vertices[k]) {
                return true;
            }
        }
        false
    }

    fn add_face_corners(corners: &mut BTreeSet<(usize, usize, usize)>, vertices: &[usize]) {
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            let k = (i + 2) % vertices.len();
            corners.insert((vertices[i], vertices[j], vertices[k]));
            corners.insert((vertices[k], vertices[j], vertices[i]));
        }
    }

    fn face_half_edge_exists(half_edges: &BTreeSet<(usize, usize)>, vertices: &[usize]) -> bool {
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            if half_edges.contains(&(vertices[i], vertices[j])) {
                return true;
            }
        }
        false
    }

    fn add_face_half_edges(half_edges: &mut BTreeSet<(usize, usize)>, vertices: &[usize]) {
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            half_edges.insert((vertices[i], vertices[j]));
        }
    }

    fn extract_mesh(
        &mut self,
        points: Vec<Vector3>,
        point_source_triangles: Vec<usize>,
        edge_connect_map: CxxMap<CxxSet>,
    ) {
        // Never iterated (insert + `operator[]` reads only), but kept
        // emulated like every C++ unordered container.
        let mut triangle_normals: CxxMap<Vector3> = CxxMap::new();
        for (point_index, source) in point_source_triangles.iter().enumerate() {
            let triangle_vertices = &self.triangles[*source];
            let triangle_normal = Vector3::normal(
                &self.vertices[triangle_vertices[0]],
                &self.vertices[triangle_vertices[1]],
                &self.vertices[triangle_vertices[2]],
            );
            // C++ `insert` over fresh `point_index` keys.
            triangle_normals.insert_new(point_index, triangle_normal);
        }

        let mut corners = BTreeSet::new();
        let triangle_round = 4;
        for round in 0..5 {
            for (level0, _) in edge_connect_map.iter() {
                let level0 = *level0;
                let Some(find_level1) = edge_connect_map.get(&level0) else {
                    continue;
                };
                let triangle_vertices = &self.triangles[point_source_triangles[level0]];
                // Dead in the C++ too (computed, never read); kept so the
                // indexing side effects stay identical.
                let _triangle_normal = Vector3::normal(
                    &self.vertices[triangle_vertices[0]],
                    &self.vertices[triangle_vertices[1]],
                    &self.vertices[triangle_vertices[2]],
                );
                // Restructure: neighbor levels are copied out level by level
                // (the C++ nests iterators seven deep); the values and the
                // visit order are unchanged.
                let level1s: Vec<usize> = find_level1.iter().copied().collect();
                for level1 in level1s {
                    let Some(find_level2) = edge_connect_map.get(&level1) else {
                        continue;
                    };
                    if self.half_edges.contains(&(level0, level1))
                        && self.half_edges.contains(&(level1, level0))
                    {
                        continue;
                    }
                    let level2s: Vec<usize> = find_level2.iter().copied().collect();
                    for level2 in level2s {
                        if level0 == level2 {
                            continue;
                        }
                        let Some(find_level3) = edge_connect_map.get(&level2) else {
                            continue;
                        };
                        if self.half_edges.contains(&(level1, level2))
                            && self.half_edges.contains(&(level2, level1))
                        {
                            continue;
                        }
                        let level3s: Vec<usize> = find_level3.iter().copied().collect();
                        for level3 in level3s {
                            if level0 == level3 {
                                if triangle_round == round {
                                    Self::try_add_face(
                                        &points,
                                        &triangle_normals,
                                        &mut corners,
                                        &mut self.half_edges,
                                        &mut self.remeshed_polygons,
                                        &[level0, level1, level2],
                                    );
                                    break;
                                }
                            } else if triangle_round == round {
                                break;
                            }
                            if level1 == level3 || level0 == level3 {
                                continue;
                            }
                            let Some(find_level4) = edge_connect_map.get(&level3) else {
                                continue;
                            };
                            if self.half_edges.contains(&(level2, level3))
                                && self.half_edges.contains(&(level3, level2))
                            {
                                continue;
                            }
                            let level4s: Vec<usize> = find_level4.iter().copied().collect();
                            for level4 in level4s {
                                if level0 != level4 {
                                    if level2 == level4 || level1 == level4 {
                                        continue;
                                    }
                                    if round < 1 {
                                        continue;
                                    }
                                    let Some(find_level5) = edge_connect_map.get(&level4) else {
                                        continue;
                                    };
                                    if self.half_edges.contains(&(level3, level4))
                                        && self.half_edges.contains(&(level4, level3))
                                    {
                                        continue;
                                    }
                                    let level5s: Vec<usize> = find_level5.iter().copied().collect();
                                    for level5 in level5s {
                                        if level0 != level5 {
                                            if level3 == level5
                                                || level2 == level5
                                                || level1 == level5
                                            {
                                                continue;
                                            }
                                            if round < 2 {
                                                continue;
                                            }
                                            let Some(find_level6) = edge_connect_map.get(&level5)
                                            else {
                                                continue;
                                            };
                                            if self.half_edges.contains(&(level4, level5))
                                                && self.half_edges.contains(&(level5, level4))
                                            {
                                                continue;
                                            }
                                            let level6s: Vec<usize> =
                                                find_level6.iter().copied().collect();
                                            for level6 in level6s {
                                                if level0 != level6 {
                                                    if level4 == level6
                                                        || level3 == level6
                                                        || level2 == level6
                                                        || level1 == level6
                                                    {
                                                        continue;
                                                    }
                                                    if round < 3 {
                                                        continue;
                                                    }
                                                    let Some(find_level7) =
                                                        edge_connect_map.get(&level6)
                                                    else {
                                                        continue;
                                                    };
                                                    if self.half_edges.contains(&(level5, level6))
                                                        && self
                                                            .half_edges
                                                            .contains(&(level6, level5))
                                                    {
                                                        continue;
                                                    }
                                                    let level7s: Vec<usize> =
                                                        find_level7.iter().copied().collect();
                                                    for level7 in level7s {
                                                        if level0 != level7 {
                                                            continue;
                                                        }
                                                        if 3 != round {
                                                            break;
                                                        }
                                                        Self::try_add_face(
                                                            &points,
                                                            &triangle_normals,
                                                            &mut corners,
                                                            &mut self.half_edges,
                                                            &mut self.remeshed_polygons,
                                                            &[
                                                                level0, level1, level2, level3,
                                                                level4, level5, level6,
                                                            ],
                                                        );
                                                        break;
                                                    }
                                                    continue;
                                                }
                                                if 2 != round {
                                                    break;
                                                }
                                                Self::try_add_face(
                                                    &points,
                                                    &triangle_normals,
                                                    &mut corners,
                                                    &mut self.half_edges,
                                                    &mut self.remeshed_polygons,
                                                    &[
                                                        level0, level1, level2, level3, level4,
                                                        level5,
                                                    ],
                                                );
                                                break;
                                            }
                                            continue;
                                        }
                                        if 1 != round {
                                            break;
                                        }
                                        Self::try_add_face(
                                            &points,
                                            &triangle_normals,
                                            &mut corners,
                                            &mut self.half_edges,
                                            &mut self.remeshed_polygons,
                                            &[level0, level1, level2, level3, level4],
                                        );
                                        break;
                                    }
                                    continue;
                                }
                                if 0 != round {
                                    break;
                                }
                                Self::try_add_face(
                                    &points,
                                    &triangle_normals,
                                    &mut corners,
                                    &mut self.half_edges,
                                    &mut self.remeshed_polygons,
                                    &[level0, level1, level2, level3],
                                );
                                break;
                            }
                        }
                    }
                }
            }
        }

        self.remeshed_vertices = points;
    }

    fn add_cross_point(
        cross_point_map: &mut BTreeMap<PositionKey, usize>,
        cross_points: &mut Vec<Vector3>,
        source_triangles: &mut Vec<usize>,
        position: Vector3,
        triangle_index: usize,
    ) -> usize {
        if let Some(existing) = cross_point_map.get(&PositionKey::from_vector(&position)) {
            return *existing;
        }
        let index = cross_points.len();
        cross_point_map.insert(PositionKey::from_vector(&position), index);
        cross_points.push(position);
        source_triangles.push(triangle_index);
        index
    }

    fn add_connection(
        &mut self,
        connections: &mut BTreeSet<(usize, usize)>,
        from_point_index: usize,
        to_point_index: usize,
        triangle_index: usize,
        coord_index: i32,
        integer: i32,
    ) {
        if from_point_index == to_point_index {
            return;
        }
        connections.insert((from_point_index, to_point_index));
        // `std::map::insert` keeps the FIRST entry on duplicate keys, like
        // `BTreeMap` entry-or-default below (no overwrite).
        self.connection_infos
            .entry((
                from_point_index.min(to_point_index),
                from_point_index.max(to_point_index),
            ))
            .or_insert(ConnectionInfo {
                triangle_index,
                coord_index,
                integer,
            });
    }

    fn extract_connections(
        &mut self,
        cross_points: &mut Vec<Vector3>,
        source_triangles: &mut Vec<usize>,
        connections: &mut BTreeSet<(usize, usize)>,
    ) {
        #[derive(Clone, Copy)]
        struct CrossPoint {
            position3: Vector3,
            position2: Vector2,
            integer: i32,
        }

        let mut cross_point_map = BTreeMap::new();

        self.connection_infos.clear();
        self.added_connections.clear();

        for triangle_index in 0..self.triangles.len() {
            // Copied out (all `Copy`) so the `&mut self` calls below
            // compile; same values the C++ references read.
            let corner_uvs = [
                self.triangle_uvs[triangle_index][0],
                self.triangle_uvs[triangle_index][1],
                self.triangle_uvs[triangle_index][2],
            ];
            let corner_indices = [
                self.triangles[triangle_index][0],
                self.triangles[triangle_index][1],
                self.triangles[triangle_index][2],
            ];

            // Extract intersections of isolines with edges
            let mut lines: [BTreeMap<i32, Vec<Vec<CrossPoint>>>; 2] =
                [BTreeMap::new(), BTreeMap::new()];
            let mut edge_collapsed = [[false; 3]; 2];
            for i in 0..2 {
                for j in 0..3 {
                    let k = (j + 1) % 3;
                    let current = corner_uvs[j];
                    let next = corner_uvs[k];
                    if is_zero((current[i] as i32) as f64 - current[i])
                        && is_zero(current[i] - next[i])
                    {
                        let integer = current[i] as i32;
                        edge_collapsed[i][j] = true;
                        let from_point = CrossPoint {
                            position3: self.vertices[corner_indices[j]],
                            position2: corner_uvs[j],
                            integer,
                        };
                        let to_point = CrossPoint {
                            position3: self.vertices[corner_indices[k]],
                            position2: corner_uvs[k],
                            integer,
                        };
                        lines[i]
                            .entry(integer)
                            .or_default()
                            .push(vec![from_point, to_point]);
                    }
                }
                let mut points: BTreeMap<i32, Vec<CrossPoint>> = BTreeMap::new();
                for j in 0..3 {
                    let k = (j + 1) % 3;
                    let current = corner_uvs[j];
                    let next = corner_uvs[k];
                    let distance = (current[i] - next[i]).abs();
                    if current[i] as i32 != next[i] as i32 || (current[i] > 0.0) != (next[i] > 0.0)
                    {
                        let (
                            low_integer,
                            high_integer,
                            from_position,
                            _to_position,
                            from_index,
                            to_index,
                        );
                        if current[i] < next[i] {
                            low_integer = current[i] as i32;
                            high_integer = next[i] as i32;
                            from_position = current[i];
                            _to_position = next[i];
                            from_index = j;
                            to_index = k;
                        } else {
                            low_integer = next[i] as i32;
                            high_integer = current[i] as i32;
                            from_position = next[i];
                            _to_position = current[i];
                            from_index = k;
                            to_index = j;
                        }
                        for integer in low_integer..=high_integer {
                            let ratio = (integer as f64 - from_position) / distance;
                            if ratio < 0.0 || ratio > 1.0 {
                                continue;
                            }
                            if (is_zero(ratio) || is_zero(ratio - 1.0)) && edge_collapsed[i][j] {
                                continue;
                            }
                            let point = CrossPoint {
                                // Unfused operator lerp (see module FMA note).
                                position3: lerp_vec3(
                                    self.vertices[corner_indices[from_index]],
                                    self.vertices[corner_indices[to_index]],
                                    ratio,
                                ),
                                position2: lerp_vec2(
                                    corner_uvs[from_index],
                                    corner_uvs[to_index],
                                    ratio,
                                ),
                                integer,
                            };
                            points.entry(integer).or_default().push(point);
                        }
                    }
                }
                for (integer, point_list) in &points {
                    for point_index in 0..point_list.len() {
                        let next_point_index = (point_index + 1) % point_list.len();
                        let point = point_list[point_index];
                        let next_point = point_list[next_point_index];
                        lines[i]
                            .entry(*integer)
                            .or_default()
                            .push(vec![point, next_point]);
                    }
                }
            }

            // Segment lines by isolines
            for i in 0..2 {
                let j = (i + 1) % 2;
                for (target_integer, target_list) in &lines[i] {
                    for target in target_list {
                        let mut segments = vec![target.clone()];
                        for split_list in lines[j].values() {
                            let split = &split_list[0];
                            let coord_index = j;
                            let segment_position = split[0].position2[coord_index];
                            for segment_index in (0..segments.len()).rev() {
                                // Read phase: decide the split from a copy
                                // (the C++ mutates the segment and pushes a
                                // new one while holding the reference).
                                let outcome = {
                                    let segment = &segments[segment_index];
                                    let uv0 = segment[0].position2;
                                    let uv1 = segment[1].position2;
                                    let distance = (uv0[coord_index] - uv1[coord_index]).abs();
                                    if is_zero(distance) {
                                        None
                                    } else {
                                        let (from_position, to_position, from_index, to_index);
                                        if uv0[coord_index] < uv1[coord_index] {
                                            from_position = uv0[coord_index];
                                            to_position = uv1[coord_index];
                                            from_index = 0;
                                            to_index = 1;
                                        } else {
                                            from_position = uv1[coord_index];
                                            to_position = uv0[coord_index];
                                            from_index = 1;
                                            to_index = 0;
                                        }
                                        if segment_position < from_position
                                            || segment_position > to_position
                                        {
                                            None
                                        } else {
                                            let ratio =
                                                (segment_position - from_position) / distance;
                                            // Unfused operator lerps (see module FMA note).
                                            let position3 = lerp_vec3(
                                                segment[from_index].position3,
                                                segment[to_index].position3,
                                                ratio,
                                            );
                                            let position2 = lerp_vec2(
                                                segment[from_index].position2,
                                                segment[to_index].position2,
                                                ratio,
                                            );
                                            let integer = segment[to_index].integer;
                                            let new_from_point = CrossPoint {
                                                position3,
                                                position2,
                                                integer,
                                            };
                                            let new_to_point = segment[to_index];
                                            Some((to_index, new_from_point, new_to_point))
                                        }
                                    }
                                };
                                if let Some((to_index, new_from_point, new_to_point)) = outcome {
                                    segments[segment_index][to_index] = new_from_point;
                                    segments.push(vec![new_from_point, new_to_point]);
                                }
                            }
                        }
                        for segment in &segments {
                            // Argument order audit: Clang evaluates the two
                            // addCrossPoint calls left-to-right (verified in
                            // -O3 IR), same as these statements.
                            let from = Self::add_cross_point(
                                &mut cross_point_map,
                                cross_points,
                                source_triangles,
                                segment[0].position3,
                                triangle_index,
                            );
                            let to = Self::add_cross_point(
                                &mut cross_point_map,
                                cross_points,
                                source_triangles,
                                segment[1].position3,
                                triangle_index,
                            );
                            self.add_connection(
                                connections,
                                from,
                                to,
                                triangle_index,
                                i as i32,
                                *target_integer,
                            );
                        }
                    }
                }
            }
        }
    }

    /// Normalized undirected edge (mirrors the `makeEdge`/`edgeOf` lambdas
    /// repeated across the C++; one shared helper, identical behavior).
    #[inline]
    fn edge_of(a: usize, b: usize) -> (usize, usize) {
        (a.min(b), a.max(b))
    }

    #[allow(clippy::too_many_arguments)]
    fn split_at_connection(
        connection_infos: &mut BTreeMap<(usize, usize), ConnectionInfo>,
        cross_point_map: &mut BTreeMap<PositionKey, usize>,
        cross_points: &mut Vec<Vector3>,
        source_triangles: &mut Vec<usize>,
        connections: &mut BTreeSet<(usize, usize)>,
        branches_of_point: &mut CxxMap<CxxSet>,
        crossing_position: Vector3,
        crossing_edge: (usize, usize),
    ) -> usize {
        let Some(info) = connection_infos.get(&crossing_edge).copied() else {
            return usize::MAX;
        };
        if let Some(existing) = cross_point_map.get(&PositionKey::from_vector(&crossing_position)) {
            return *existing;
        }
        let new_point_index = cross_points.len();
        cross_point_map.insert(
            PositionKey::from_vector(&crossing_position),
            new_point_index,
        );
        cross_points.push(crossing_position);
        source_triangles.push(info.triangle_index);
        let (edge_first, edge_second) = crossing_edge;
        connections.remove(&(edge_first, edge_second));
        connections.remove(&(edge_second, edge_first));
        connection_infos.remove(&crossing_edge);
        // C++ `operator[]` + erase (both endpoints are present: the branch
        // map tracks `connections` exactly).
        branches_of_point
            .get_or_default(edge_first)
            .remove(&edge_second);
        branches_of_point
            .get_or_default(edge_second)
            .remove(&edge_first);
        for endpoint in [edge_first, edge_second] {
            connections.insert((endpoint, new_point_index));
            connection_infos.insert(Self::edge_of(endpoint, new_point_index), info);
            branches_of_point
                .get_or_default(endpoint)
                .insert(new_point_index);
            branches_of_point
                .get_or_default(new_point_index)
                .insert(endpoint);
        }
        new_point_index
    }

    #[allow(clippy::too_many_arguments)]
    fn add_walk_connection(
        connection_infos: &mut BTreeMap<(usize, usize), ConnectionInfo>,
        added_connections: &mut BTreeSet<(usize, usize)>,
        connections: &mut BTreeSet<(usize, usize)>,
        source_triangles: &[usize],
        branches_of_point: &mut CxxMap<CxxSet>,
        from_point_index: usize,
        to_point_index: usize,
    ) -> bool {
        if from_point_index == to_point_index {
            return false;
        }
        let edge = Self::edge_of(from_point_index, to_point_index);
        if connection_infos.contains_key(&edge) {
            return false;
        }
        connections.insert((from_point_index, to_point_index));
        connection_infos.insert(
            edge,
            ConnectionInfo {
                triangle_index: source_triangles[to_point_index],
                coord_index: -1,
                integer: 0,
            },
        );
        added_connections.insert(edge);
        branches_of_point
            .get_or_default(from_point_index)
            .insert(to_point_index);
        branches_of_point
            .get_or_default(to_point_index)
            .insert(from_point_index);
        true
    }

    #[allow(clippy::too_many_arguments)]
    fn find_node_ahead(
        cross_points: &[Vector3],
        branches_of_point: &CxxMap<CxxSet>,
        local_edges: &BTreeMap<(usize, usize), ConnectionInfo>,
        behind_points: &CxxSet,
        nearby_radius: f64,
        ahead_cosine_threshold: f64,
        position: Vector3,
        direction: Vector3,
    ) -> usize {
        let mut nearest = usize::MAX;
        let mut nearest_distance = nearby_radius;
        for edge in local_edges.keys() {
            for endpoint in [edge.0, edge.1] {
                if behind_points.contains(&endpoint) {
                    continue;
                }
                let Some(branches) = branches_of_point.get(&endpoint) else {
                    continue;
                };
                if branches.len() <= 2 {
                    continue;
                }
                let offset = cross_points[endpoint] - position;
                let distance = offset.length();
                if is_zero(distance) || distance >= nearest_distance {
                    continue;
                }
                if Vector3::dot_product(&(offset / distance), &direction) < ahead_cosine_threshold {
                    continue;
                }
                nearest = endpoint;
                nearest_distance = distance;
            }
        }
        nearest
    }

    #[allow(clippy::too_many_arguments)]
    fn find_crossing(
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        cross_points: &[Vector3],
        local_edges: &BTreeMap<(usize, usize), ConnectionInfo>,
        behind_points: &CxxSet,
        parallel_cosine_threshold: f64,
        tolerance: f64,
        position: Vector3,
        direction: Vector3,
        limit_distance: f64,
        also_blocked_point: usize,
    ) -> Option<(Vector3, (usize, usize), usize)> {
        let mut found = None;
        let mut nearest_distance = limit_distance;
        for (edge, info) in local_edges {
            let mut blocked = false;
            for endpoint in [edge.0, edge.1] {
                if endpoint == also_blocked_point || behind_points.contains(&endpoint) {
                    blocked = true;
                    break;
                }
            }
            if blocked {
                continue;
            }
            let from = cross_points[edge.0];
            let to = cross_points[edge.1];
            let edge_vector = to - from;
            if (Vector3::dot_product(&direction, &edge_vector.normalized())).abs()
                > parallel_cosine_threshold
            {
                continue;
            }
            let offset = from - position;
            let a = Vector3::dot_product(&edge_vector, &edge_vector);
            let b = Vector3::dot_product(&direction, &edge_vector);
            let c = Vector3::dot_product(&direction, &offset);
            let d = Vector3::dot_product(&edge_vector, &offset);
            // FMA: `a - b * b` and `(b * c - d) / denominator`.
            let denominator = (-b).mul_add(b, a);
            let mut edge_ratio = if is_zero(denominator) {
                0.0
            } else {
                b.mul_add(c, -d) / denominator
            };
            // Manual clamp like the C++ (NaN passes through on both sides).
            #[allow(clippy::manual_clamp)]
            if edge_ratio < 0.0 {
                edge_ratio = 0.0;
            } else if edge_ratio > 1.0 {
                edge_ratio = 1.0;
            }
            // Unfused `from + edgeVector * edgeRatio` (see module FMA note).
            let point_on_edge = add_scaled_vec3(from, edge_vector, edge_ratio);
            let distance = Vector3::dot_product(&(point_on_edge - position), &direction);
            if distance <= tolerance || distance >= nearest_distance {
                continue;
            }
            // Unfused `position + direction * distance` (see module FMA note).
            let point_on_ray = add_scaled_vec3(position, direction, distance);
            let miss = point_on_edge - point_on_ray;
            let miss_triangle = &triangles[info.triangle_index];
            let miss_normal = Vector3::normal(
                &vertices[miss_triangle[0]],
                &vertices[miss_triangle[1]],
                &vertices[miss_triangle[2]],
            );
            // Unfused `miss - missNormal * dot` (see module FMA note).
            let projected_miss =
                sub_scaled_vec3(miss, miss_normal, Vector3::dot_product(&miss, &miss_normal));
            if projected_miss.length() > tolerance {
                continue;
            }
            found = Some((point_on_edge, *edge, info.triangle_index));
            nearest_distance = distance;
        }
        found
    }

    fn hold_singular_lines(
        &mut self,
        cross_points: &mut Vec<Vector3>,
        source_triangles: &mut Vec<usize>,
        connections: &mut BTreeSet<(usize, usize)>,
    ) {
        let Some(singular_vertices) = self.singular_vertices else {
            return;
        };
        if singular_vertices.is_empty() {
            return;
        }

        const RING_COUNT: usize = 10;
        const MAX_WALK_STEPS: usize = 32;
        let ahead_cosine_threshold = (PI * 30.0 / 180.0).cos();
        const PARALLEL_COSINE_THRESHOLD: f64 = 0.9;

        let mut cross_point_map = BTreeMap::new();
        for (i, point) in cross_points.iter().enumerate() {
            // First index wins on duplicates (C++ `std::map::insert`).
            cross_point_map
                .entry(PositionKey::from_vector(point))
                .or_insert(i);
        }

        let mut branches_of_point: CxxMap<CxxSet> = CxxMap::new();
        for (first, second) in connections.iter() {
            branches_of_point.get_or_default(*first).insert(*second);
            branches_of_point.get_or_default(*second).insert(*first);
        }

        let mut triangles_around_vertex: CxxMap<Vec<usize>> = CxxMap::new();
        for (triangle_index, triangle) in self.triangles.iter().enumerate() {
            for vertex_index in triangle {
                triangles_around_vertex
                    .get_or_default(*vertex_index)
                    .push(triangle_index);
            }
        }

        let mut added_connections = 0;

        let mut starved_cones = 0;
        let mut walked_cones = 0;
        // Copied out: the loop calls `&mut self` helpers.
        let singular_list = singular_vertices.to_vec();
        for singular_vertex_index in singular_list {
            if singular_vertex_index >= self.vertices.len() {
                continue;
            }
            let singular_position = self.vertices[singular_vertex_index];

            let singular_point_index =
                match cross_point_map.get(&PositionKey::from_vector(&singular_position)) {
                    Some(index) => *index,
                    None => continue,
                };
            let coming_from_point_index = match branches_of_point.get(&singular_point_index) {
                Some(branches) if branches.len() == 1 => {
                    // Invariant: exactly one branch (checked above).
                    match branches.iter().next().copied() {
                        Some(only) => only,
                        None => continue,
                    }
                }
                _ => continue,
            };
            starved_cones += 1;

            let mut neighbor_triangles = CxxSet::new();
            {
                let mut ring_vertices = CxxSet::from([singular_vertex_index]);
                for _ in 0..RING_COUNT {
                    let mut next_ring_vertices = CxxSet::new();
                    for vertex_index in &ring_vertices {
                        let Some(find_triangles) = triangles_around_vertex.get(vertex_index) else {
                            continue;
                        };
                        for triangle_index in find_triangles {
                            neighbor_triangles.insert(*triangle_index);
                            for corner in &self.triangles[*triangle_index] {
                                next_ring_vertices.insert(*corner);
                            }
                        }
                    }
                    ring_vertices = next_ring_vertices;
                }
            }
            if neighbor_triangles.is_empty() {
                continue;
            }

            let mut local_edges = BTreeMap::new();
            let mut total_edge_length = 0.0;
            for (first_point, second_point) in connections.iter() {
                let edge = Self::edge_of(*first_point, *second_point);
                let Some(find_info) = self.connection_infos.get(&edge) else {
                    continue;
                };
                if !neighbor_triangles.contains(&find_info.triangle_index) {
                    continue;
                }
                // First wins (C++ `insert`); skip on duplicate.
                if local_edges.contains_key(&edge) {
                    continue;
                }
                local_edges.insert(edge, *find_info);
                total_edge_length += (cross_points[edge.0] - cross_points[edge.1]).length();
            }
            if local_edges.len() < 3 {
                continue;
            }
            let average_edge_length = total_edge_length / local_edges.len() as f64;
            if is_zero(average_edge_length) {
                continue;
            }
            let tolerance = 0.5 * average_edge_length;
            let nearby_radius = 12.0 * average_edge_length;

            let mut cone_normal = Vector3::default();
            {
                let Some(find_triangles) = triangles_around_vertex.get(&singular_vertex_index)
                else {
                    continue;
                };
                for triangle_index in find_triangles {
                    let triangle = &self.triangles[*triangle_index];
                    cone_normal += Vector3::normal(
                        &self.vertices[triangle[0]],
                        &self.vertices[triangle[1]],
                        &self.vertices[triangle[2]],
                    );
                }
                cone_normal = cone_normal.normalized();
                if cone_normal.is_zero() {
                    continue;
                }
            }
            // Deferred init: the C++ seeds this with the coming-from point
            // and unconditionally overwrites it below.
            let seed_tail;
            {
                let mut coord_index = -1;
                let mut integer = 0;
                if let Some(find_info) = self.connection_infos.get(&Self::edge_of(
                    singular_point_index,
                    coming_from_point_index,
                )) {
                    coord_index = find_info.coord_index;
                    integer = find_info.integer;
                }
                let mut previous = singular_point_index;
                let mut current = coming_from_point_index;
                for _ in 0..MAX_WALK_STEPS {
                    let Some(find_branches) = branches_of_point.get(&current) else {
                        break;
                    };
                    if find_branches.len() > 2 {
                        break;
                    }
                    let mut next = usize::MAX;
                    for neighbor in find_branches {
                        if *neighbor == previous {
                            continue;
                        }
                        let Some(find_next_info) = self
                            .connection_infos
                            .get(&Self::edge_of(current, *neighbor))
                        else {
                            continue;
                        };
                        if coord_index >= 0
                            && (find_next_info.coord_index != coord_index
                                || find_next_info.integer != integer)
                        {
                            continue;
                        }
                        next = *neighbor;
                        break;
                    }
                    if usize::MAX == next {
                        break;
                    }
                    previous = current;
                    current = next;
                }
                seed_tail = cross_points[current];
            }
            let mut seed_direction = singular_position - seed_tail;
            // Unfused `seedDirection - coneNormal * dot` (see module FMA note).
            seed_direction = sub_scaled_vec3(
                seed_direction,
                cone_normal,
                Vector3::dot_product(&seed_direction, &cone_normal),
            )
            .normalized();
            if seed_direction.is_zero() {
                continue;
            }

            let behind_points = CxxSet::from([singular_point_index, coming_from_point_index]);

            #[derive(Clone, Copy)]
            struct WalkCrossing {
                position: Vector3,
                edge: (usize, usize),
                triangle_index: usize,
            }

            let mut path = Vec::new();
            let mut node_point_index = usize::MAX;
            {
                let mut walk_position = singular_position;
                let mut walk_direction = seed_direction;
                for _ in 0..MAX_WALK_STEPS {
                    let mut limit_distance = f64::MAX;
                    if usize::MAX == node_point_index {
                        node_point_index = Self::find_node_ahead(
                            cross_points,
                            &branches_of_point,
                            &local_edges,
                            &behind_points,
                            nearby_radius,
                            ahead_cosine_threshold,
                            walk_position,
                            walk_direction,
                        );
                    }
                    if usize::MAX != node_point_index {
                        let offset = cross_points[node_point_index] - walk_position;
                        limit_distance = offset.length();
                        walk_direction = offset.normalized();
                    }
                    let Some((crossing_position, crossing_edge, crossing_triangle)) =
                        Self::find_crossing(
                            self.vertices,
                            self.triangles,
                            cross_points,
                            &local_edges,
                            &behind_points,
                            PARALLEL_COSINE_THRESHOLD,
                            tolerance,
                            walk_position,
                            walk_direction,
                            limit_distance,
                            node_point_index,
                        )
                    else {
                        break;
                    };
                    let crossing = WalkCrossing {
                        position: crossing_position,
                        edge: crossing_edge,
                        triangle_index: crossing_triangle,
                    };
                    path.push(crossing);
                    walk_position = crossing.position;
                    let triangle = &self.triangles[crossing.triangle_index];
                    let triangle_normal = Vector3::normal(
                        &self.vertices[triangle[0]],
                        &self.vertices[triangle[1]],
                        &self.vertices[triangle[2]],
                    );
                    // Unfused `walkDirection - triangleNormal * dot` (see module FMA note).
                    let flattened = sub_scaled_vec3(
                        walk_direction,
                        triangle_normal,
                        Vector3::dot_product(&walk_direction, &triangle_normal),
                    );
                    if !flattened.is_zero() {
                        walk_direction = flattened.normalized();
                    }
                }
            }
            if usize::MAX == node_point_index {
                continue;
            }

            let mut previous_point_index = singular_point_index;
            for crossing in &path {
                let point_index = Self::split_at_connection(
                    &mut self.connection_infos,
                    &mut cross_point_map,
                    cross_points,
                    source_triangles,
                    connections,
                    &mut branches_of_point,
                    crossing.position,
                    crossing.edge,
                );
                if usize::MAX == point_index {
                    continue;
                }
                if Self::add_walk_connection(
                    &mut self.connection_infos,
                    &mut self.added_connections,
                    connections,
                    source_triangles,
                    &mut branches_of_point,
                    previous_point_index,
                    point_index,
                ) {
                    added_connections += 1;
                }
                previous_point_index = point_index;
            }
            if Self::add_walk_connection(
                &mut self.connection_infos,
                &mut self.added_connections,
                connections,
                source_triangles,
                &mut branches_of_point,
                previous_point_index,
                node_point_index,
            ) {
                added_connections += 1;
            }
            walked_cones += 1;
        }

        self.diagnose(|| {
            format!(
                "Hold singular lines walked {walked_cones} of {starved_cones} starved cone(s), added {added_connections} connection(s)\n"
            )
        });
    }

    fn test_point_in_triangle(
        points: &[Vector3],
        triangle: &[usize],
        test_points: &[usize],
    ) -> bool {
        let triangle_normal = Vector3::normal(
            &points[triangle[0]],
            &points[triangle[1]],
            &points[triangle[2]],
        );
        let mut points_in_3d = Vec::new();
        for it in triangle {
            points_in_3d.push(points[*it]);
        }
        for it in test_points {
            points_in_3d.push(points[*it]);
        }
        let mut points_in_2d = Vec::new();
        let origin = (points[triangle[0]] + points[triangle[1]] + points[triangle[2]]) / 3.0;
        let axis = (points[triangle[0]] - origin).normalized();
        Vector3::project_to_2d(
            &points_in_3d,
            &mut points_in_2d,
            &triangle_normal,
            &axis,
            &origin,
        );
        let a = points_in_2d[0];
        let b = points_in_2d[1];
        let c = points_in_2d[2];
        for point in points_in_2d.iter().skip(3) {
            if Vector2::is_in_triangle(&a, &b, &c, point) {
                return true;
            }
        }
        false
    }

    fn rebuild_half_edges(&mut self) {
        self.half_edges.clear();
        for face in &self.remeshed_polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                self.half_edges.insert((face[i], face[j]));
            }
        }
    }

    fn fix_holes(&mut self) {
        let mut loops = Vec::new();
        // Restructure: cloned (the C++ passes the member by const ref while
        // `fix_hole_with_quads` mutates it through `this`).
        self.search_boundaries(&self.half_edges.clone(), &mut loops);
        for loop_ in &mut loops {
            if loop_.len() > 65 {
                self.diagnose(|| format!("Ignore long hole at length:{}\n", loop_.len()));
                continue;
            }
            self.diagnose(|| format!("Fixing hole at length:{}...\n", loop_.len()));
            self.fix_hole_with_quads(loop_, true);
            // fixHoleWithQuads returns with 3 or 4 hole verts left only
            // after pushing that final cap, so a second pass there re-emits
            // the identical quad (doubled face, use-count-3 edges).
            // Continue only on a genuine uncapped remainder.
            if loop_.len() > 4 {
                self.fix_hole_with_quads(loop_, false);
            }
        }
    }

    fn record_half_edges_of_last_polygon(&mut self) {
        let last = self.remeshed_polygons.len() - 1;
        for i in 0..self.remeshed_polygons[last].len() {
            let face_len = self.remeshed_polygons[last].len();
            let j = (i + 1) % face_len;
            let edge = (
                self.remeshed_polygons[last][i],
                self.remeshed_polygons[last][j],
            );
            self.half_edges.insert(edge);
        }
    }

    fn fix_hole_with_quads(&mut self, hole: &mut Vec<usize>, check_score: bool) {
        loop {
            if hole.len() <= 2 {
                self.diagnose(|| {
                    format!("fixHoleWithQuads cancel on edge length:{}\n", hole.len())
                });
                return;
            }

            if 3 == hole.len() {
                self.remeshed_polygons.push(vec![hole[2], hole[1], hole[0]]);
                self.record_half_edges_of_last_polygon();
                return;
            }

            if 4 == hole.len() {
                self.remeshed_polygons
                    .push(vec![hole[3], hole[2], hole[1], hole[0]]);
                self.record_half_edges_of_last_polygon();
                return;
            }

            let mut edge_scores = Vec::with_capacity(hole.len());
            for i in 0..hole.len() {
                let h = (i + hole.len() - 1) % hole.len();
                let j = (i + 1) % hole.len();
                let k = (j + 1) % hole.len();
                let left = (self.remeshed_vertices[hole[h]] - self.remeshed_vertices[hole[i]])
                    .normalized();
                let right = (self.remeshed_vertices[hole[k]] - self.remeshed_vertices[hole[j]])
                    .normalized();
                edge_scores.push((i, Vector3::dot_product(&left, &right)));
            }
            // Restructure: `std::sort` (libc++ introsort) becomes
            // `sort_unstable_by` (pdqsort). Both deterministic, but they
            // order score ties differently; the oracle measures the fallout
            // on symmetric holes (see the report).
            edge_scores.sort_unstable_by(|first, second| {
                first.1.partial_cmp(&second.1).unwrap_or(Ordering::Equal)
            });
            let mut hole_changed = false;
            for edge_index in (0..edge_scores.len()).rev() {
                let score = edge_scores[edge_index];
                if check_score && score.1 <= 0.0 {
                    self.diagnose(|| {
                        format!("fixHoleWithQuads failed, highest score(dot):{}\n", score.1)
                    });
                    return;
                }
                let i = score.0;
                let h = (i + hole.len() - 1) % hole.len();
                let j = (i + 1) % hole.len();
                let k = (j + 1) % hole.len();
                let candidate = vec![hole[k], hole[j], hole[i], hole[h]];
                if self.half_edges.contains(&(candidate[0], candidate[1]))
                    || self.half_edges.contains(&(candidate[1], candidate[2]))
                    || self.half_edges.contains(&(candidate[2], candidate[3]))
                    || self.half_edges.contains(&(candidate[3], candidate[0]))
                {
                    self.diagnose(|| {
                        format!(
                            "fixHoleWithQuads ignore score:{} because conflicts with existed quads\n",
                            score.1
                        )
                    });
                    continue;
                }
                let mut remain_points = Vec::new();
                for (w, point) in hole.iter().enumerate() {
                    if w == i || w == j || w == h || w == k {
                        continue;
                    }
                    remain_points.push(*point);
                }
                if Self::test_point_in_triangle(
                    &self.remeshed_vertices,
                    &[candidate[0], candidate[1], candidate[2]],
                    &remain_points,
                ) || Self::test_point_in_triangle(
                    &self.remeshed_vertices,
                    &[candidate[2], candidate[3], candidate[0]],
                    &remain_points,
                ) {
                    self.diagnose(|| {
                        format!(
                            "fixHoleWithQuads ignore score:{} because other point in the same loop fall into quad plane\n",
                            score.1
                        )
                    });
                    continue;
                }
                self.remeshed_polygons.push(candidate);
                self.record_half_edges_of_last_polygon();

                let mut new_hole = Vec::new();
                for (w, point) in hole.iter().enumerate() {
                    if w == i || w == j {
                        continue;
                    }
                    new_hole.push(*point);
                }
                *hole = new_hole;
                hole_changed = true;
                break;
            }
            if !hole_changed {
                break;
            }
        }
    }

    fn search_boundaries(
        &self,
        half_edges: &BTreeSet<(usize, usize)>,
        loops: &mut Vec<Vec<usize>>,
    ) {
        self.diagnose(|| "Searching boundaries...\n".to_string());

        let mut next_map: CxxMap<CxxSet> = CxxMap::new();
        for (from, to) in half_edges {
            if half_edges.contains(&(*to, *from)) {
                continue;
            }
            next_map.get_or_default(*from).insert(*to);
        }

        while !next_map.is_empty() {
            // libc++ `nextMap.begin()`: first node in hash order.
            let Some(start_vertex) = next_map.first_key().copied() else {
                break;
            };
            let mut loop_ = Vec::new();
            let mut validate = false;
            self.diagnose(|| format!("Searching loop from:{start_vertex}\n"));
            let mut current = Some(start_vertex);
            while let Some(vertex) = current {
                if start_vertex == vertex && loop_.len() >= 3 {
                    self.diagnose(|| format!("Found valid loop, size:{}\n", loop_.len()));
                    validate = true;
                    break;
                }
                self.diagnose(|| format!("Loop add vertex:{vertex}\n"));
                loop_.push(vertex);
                let Some(nexts) = next_map.get(&vertex) else {
                    break;
                };
                if nexts.len() != 1 {
                    self.diagnose(|| format!("Break loop, because of next size:{}\n", nexts.len()));
                    break;
                }
                // Invariant: exactly one next vertex (checked above).
                current = nexts.iter().next().copied();
            }
            for v in &loop_ {
                next_map.remove(v);
            }
            if validate {
                loops.push(loop_);
            }
        }

        self.diagnose(|| "Searching boundaries done\n".to_string());
    }

    fn remove_isolated_faces(&mut self) -> bool {
        let mut quads_islands = Vec::new();
        MeshSeparator::split_to_islands(&self.remeshed_polygons, &mut quads_islands);
        if quads_islands.is_empty() {
            return false;
        }
        // `std::max_element` returns the FIRST maximum; Rust's `max_by`
        // returns the last, so this stays a manual strict-`<` loop.
        let mut biggest = 0;
        for (index, island) in quads_islands.iter().enumerate() {
            if quads_islands[biggest].len() < island.len() {
                biggest = index;
            }
        }
        self.remeshed_polygons = std::mem::take(&mut quads_islands[biggest]);
        true
    }

    fn remove_non_manifold_faces(&mut self) -> bool {
        let mut changed = false;
        let mut edge_to_face_map = BTreeMap::new();
        MeshSeparator::build_edge_to_face_map(&self.remeshed_polygons, &mut edge_to_face_map);
        let mut vertex_open_boundary_count_map: BTreeMap<usize, usize> = BTreeMap::new();
        for edge in edge_to_face_map.keys() {
            if edge_to_face_map.contains_key(&(edge.1, edge.0)) {
                continue;
            }
            *vertex_open_boundary_count_map.entry(edge.0).or_default() += 1;
            *vertex_open_boundary_count_map.entry(edge.1).or_default() += 1;
        }
        let mut manifold_faces = Vec::new();
        for face in &self.remeshed_polygons {
            let mut is_non_manifold = false;
            for vertex in face {
                let Some(find_count) = vertex_open_boundary_count_map.get(vertex) else {
                    continue;
                };
                if *find_count > 2 {
                    is_non_manifold = true;
                    break;
                }
            }
            if is_non_manifold {
                changed = true;
                continue;
            }
            manifold_faces.push(face.clone());
        }
        self.remeshed_polygons = manifold_faces;
        changed
    }

    fn smooth_and_project(
        &mut self,
        iterations: usize,
        // `BTreeSet`: membership-only use (`contains`, `is_empty`),
        // exact for the C++ `unordered_set` (container audit).
        movable_vertices: Option<&BTreeSet<usize>>,
    ) {
        if 0 == iterations
            || self.remeshed_vertices.is_empty()
            || self.remeshed_polygons.is_empty()
            || self.triangles.is_empty()
        {
            return;
        }

        let mut neighbors: Vec<CxxSet> = vec![CxxSet::new(); self.remeshed_vertices.len()];
        let mut edge_use_count: BTreeMap<(usize, usize), usize> = BTreeMap::new();
        for face in &self.remeshed_polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                if face[i] >= neighbors.len() || face[j] >= neighbors.len() {
                    continue;
                }
                neighbors[face[i]].insert(face[j]);
                neighbors[face[j]].insert(face[i]);
                *edge_use_count
                    .entry((face[i].min(face[j]), face[i].max(face[j])))
                    .or_default() += 1;
            }
        }
        let mut locked = vec![false; self.remeshed_vertices.len()];
        if let Some(movable) = movable_vertices {
            for (index, lock) in locked.iter_mut().enumerate() {
                *lock = !movable.contains(&index);
            }
        }
        for (edge, use_count) in &edge_use_count {
            if 1 == *use_count {
                locked[edge.0] = true;
                locked[edge.1] = true;
            }
        }

        let mut triangle_boxes = vec![AxisAlignedBoundingBox::default(); self.triangles.len()];
        let mut triangle_indices = vec![0; self.triangles.len()];
        let mut group_box = AxisAlignedBoundingBox::default();
        for (i, triangle) in self.triangles.iter().enumerate() {
            for k in 0..3 {
                triangle_boxes[i].update(&self.vertices[triangle[k]]);
                group_box.update(&self.vertices[triangle[k]]);
            }
            triangle_boxes[i].update_center();
            triangle_indices[i] = i;
        }
        group_box.update_center();
        let tree = AxisAlignedBoundingBoxTree::new(triangle_boxes, triangle_indices, group_box);

        // Average quad edge length drives the initial search radius
        let mut total_edge_length = 0.0;
        let mut edge_num = 0;
        for edge in edge_use_count.keys() {
            total_edge_length +=
                (self.remeshed_vertices[edge.0] - self.remeshed_vertices[edge.1]).length();
            edge_num += 1;
        }
        if 0 == edge_num {
            return;
        }
        let average_edge_length = total_edge_length / edge_num as f64;
        if average_edge_length <= 0.0 {
            return;
        }

        // Both passes below already read one buffer and write another, and
        // the projection only reads the bounding box tree, so each vertex
        // is independent and the parallel result is the same as the serial
        // one. (Restructure: `parallel_each` for `tbb::parallel_for`.)
        const SMOOTH_FACTOR: f64 = 0.5;
        for _ in 0..iterations {
            let mut smoothed_vertices = self.remeshed_vertices.clone();
            parallel_each(&mut smoothed_vertices, |i, smoothed| {
                if locked[i] || neighbors[i].is_empty() {
                    return;
                }
                let mut center = Vector3::default();
                for neighbor in &neighbors[i] {
                    center += self.remeshed_vertices[*neighbor];
                }
                center /= neighbors[i].len() as f64;
                // Unfused `v + smoothFactor * (center - v)` (see module FMA note).
                *smoothed = add_scaled_vec3(
                    self.remeshed_vertices[i],
                    center - self.remeshed_vertices[i],
                    SMOOTH_FACTOR,
                );
            });
            parallel_each(&mut smoothed_vertices, |i, smoothed| {
                if locked[i] || neighbors[i].is_empty() {
                    return;
                }
                if let Some(projected) = Self::project_to_target_mesh(
                    self.vertices,
                    self.triangles,
                    &tree,
                    average_edge_length,
                    *smoothed,
                ) {
                    *smoothed = projected;
                }
            });
            self.remeshed_vertices = smoothed_vertices;
        }
    }

    fn project_to_target_mesh(
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        tree: &AxisAlignedBoundingBoxTree,
        average_edge_length: f64,
        position: Vector3,
    ) -> Option<Vector3> {
        let mut radius = average_edge_length;
        while radius <= average_edge_length * 8.0 {
            let mut query_boxes = vec![AxisAlignedBoundingBox::default()];
            query_boxes[0].update(&Vector3::new(
                position.x() - radius,
                position.y() - radius,
                position.z() - radius,
            ));
            query_boxes[0].update(&Vector3::new(
                position.x() + radius,
                position.y() + radius,
                position.z() + radius,
            ));
            query_boxes[0].update_center();
            let outer = query_boxes[0].clone();
            let query_tree = AxisAlignedBoundingBoxTree::new(query_boxes, vec![0], outer);
            let mut pairs = Vec::new();
            tree.test(tree.root(), &query_tree, query_tree.root(), &mut pairs);
            let mut min_distance2 = f64::MAX;
            let mut projected = None;
            for (key, _) in &pairs {
                let triangle = &triangles[*key];
                let candidate = closest_point_on_triangle(
                    position,
                    vertices[triangle[0]],
                    vertices[triangle[1]],
                    vertices[triangle[2]],
                );
                let distance2 = (candidate - position).length_squared();
                if distance2 < min_distance2 {
                    min_distance2 = distance2;
                    projected = Some(candidate);
                }
            }
            if min_distance2 < f64::MAX {
                return projected;
            }
            radius *= 2.0;
        }
        None
    }

    fn compute_remeshed_vertex_uvs(&mut self) {
        self.remeshed_vertex_uvs.clear();
        if self.remeshed_vertices.is_empty()
            || self.triangles.is_empty()
            || self.triangle_uvs.len() != self.triangles.len()
        {
            return;
        }

        // Bounding-box tree over the source triangles, same construction as
        // smoothAndProject: output vertices lie on the source surface, so
        // an expanding-radius query finds the home triangle in a few
        // probes.
        let mut triangle_boxes = vec![AxisAlignedBoundingBox::default(); self.triangles.len()];
        let mut triangle_indices = vec![0; self.triangles.len()];
        let mut group_box = AxisAlignedBoundingBox::default();
        for (i, triangle) in self.triangles.iter().enumerate() {
            for k in 0..3 {
                triangle_boxes[i].update(&self.vertices[triangle[k]]);
                group_box.update(&self.vertices[triangle[k]]);
            }
            triangle_boxes[i].update_center();
            triangle_indices[i] = i;
        }
        group_box.update_center();
        let tree = AxisAlignedBoundingBoxTree::new(triangle_boxes, triangle_indices, group_box);

        let mut total_edge_length = 0.0;
        let mut edge_num = 0;
        for face in &self.remeshed_polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                if face[i] >= self.remeshed_vertices.len()
                    || face[j] >= self.remeshed_vertices.len()
                {
                    continue;
                }
                total_edge_length +=
                    (self.remeshed_vertices[face[i]] - self.remeshed_vertices[face[j]]).length();
                edge_num += 1;
            }
        }
        if 0 == edge_num {
            return;
        }
        let average_edge_length = total_edge_length / edge_num as f64;
        if average_edge_length <= 0.0 {
            return;
        }

        // Each vertex is independent (reads the tree and the source mesh,
        // writes its own slot), so the parallel result matches the serial
        // one exactly. (Restructure: `parallel_each` for `tbb::parallel_for`.)
        let mut uvs = vec![Vector2::default(); self.remeshed_vertices.len()];
        parallel_each(&mut uvs, |i, uv| {
            let mut projected = self.remeshed_vertices[i];
            let home = Self::find_home_triangle(
                self.vertices,
                self.triangles,
                &tree,
                average_edge_length,
                self.remeshed_vertices[i],
                &mut projected,
            );
            let triangle = &self.triangles[home];
            let corner_uvs = &self.triangle_uvs[home];
            if corner_uvs.len() < 3 {
                *uv = if corner_uvs.is_empty() {
                    Vector2::default()
                } else {
                    corner_uvs[0]
                };
                return;
            }
            let area = Vector3::area(
                &self.vertices[triangle[0]],
                &self.vertices[triangle[1]],
                &self.vertices[triangle[2]],
            );
            // `!(area > ...)` like the C++ (NaN takes this arm on both sides).
            #[allow(clippy::neg_cmp_op_on_partial_ord)]
            if !(area > 1e-18) {
                *uv = Vector2::new(
                    (corner_uvs[0].x() + corner_uvs[1].x() + corner_uvs[2].x()) / 3.0,
                    (corner_uvs[0].y() + corner_uvs[1].y() + corner_uvs[2].y()) / 3.0,
                );
                return;
            }
            let bary = Vector3::barycentric_coordinates(
                &self.vertices[triangle[0]],
                &self.vertices[triangle[1]],
                &self.vertices[triangle[2]],
                &projected,
            );
            // FMA: three left-nested products fuse outside-in, `fma(bz,
            // uz, fma(bx, ux, by * uy))` per the probe (same shape as the
            // dot-product chain).
            *uv = Vector2::new(
                bary.z().mul_add(
                    corner_uvs[2].x(),
                    fma_first(bary.x(), corner_uvs[0].x(), bary.y(), corner_uvs[1].x()),
                ),
                bary.z().mul_add(
                    corner_uvs[2].y(),
                    fma_first(bary.x(), corner_uvs[0].y(), bary.y(), corner_uvs[1].y()),
                ),
            );
        });

        // Normalize to 0..1 over this island's UV bounding box. Non-finite
        // interpolants (a degenerate parameterization corner) collapse to
        // the box center so the accessor never emits NaN or infinity.
        let mut min_u = 0.0;
        let mut max_u = 0.0;
        let mut min_v = 0.0;
        let mut max_v = 0.0;
        let mut have_finite = false;
        for uv in &uvs {
            if !uv.x().is_finite() || !uv.y().is_finite() {
                continue;
            }
            if !have_finite {
                min_u = uv.x();
                max_u = uv.x();
                min_v = uv.y();
                max_v = uv.y();
                have_finite = true;
            } else {
                min_u = min_u.min(uv.x());
                max_u = max_u.max(uv.x());
                min_v = min_v.min(uv.y());
                max_v = max_v.max(uv.y());
            }
        }
        if !have_finite {
            uvs = vec![Vector2::new(0.5, 0.5); uvs.len()];
            self.remeshed_vertex_uvs = uvs;
            return;
        }
        let range_u = max_u - min_u;
        let range_v = max_v - min_v;
        let center_u = (min_u + max_u) * 0.5;
        let center_v = (min_v + max_v) * 0.5;
        for uv in &mut uvs {
            let mut u = if uv.x().is_finite() { uv.x() } else { center_u };
            let mut v = if uv.y().is_finite() { uv.y() } else { center_v };
            u = if range_u > 1e-12 {
                (u - min_u) / range_u
            } else {
                0.5
            };
            v = if range_v > 1e-12 {
                (v - min_v) / range_v
            } else {
                0.5
            };
            *uv = Vector2::new(u, v);
        }
        self.remeshed_vertex_uvs = uvs;
    }

    #[allow(clippy::too_many_arguments)]
    fn find_home_triangle(
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        tree: &AxisAlignedBoundingBoxTree,
        average_edge_length: f64,
        position: Vector3,
        projected: &mut Vector3,
    ) -> usize {
        let mut home = 0;
        let mut radius = average_edge_length;
        while radius <= average_edge_length * 8.0 {
            let mut query_boxes = vec![AxisAlignedBoundingBox::default()];
            query_boxes[0].update(&Vector3::new(
                position.x() - radius,
                position.y() - radius,
                position.z() - radius,
            ));
            query_boxes[0].update(&Vector3::new(
                position.x() + radius,
                position.y() + radius,
                position.z() + radius,
            ));
            query_boxes[0].update_center();
            let outer = query_boxes[0].clone();
            let query_tree = AxisAlignedBoundingBoxTree::new(query_boxes, vec![0], outer);
            let mut pairs = Vec::new();
            tree.test(tree.root(), &query_tree, query_tree.root(), &mut pairs);
            let mut min_distance2 = f64::MAX;
            for (key, _) in &pairs {
                let triangle = &triangles[*key];
                let candidate = closest_point_on_triangle(
                    position,
                    vertices[triangle[0]],
                    vertices[triangle[1]],
                    vertices[triangle[2]],
                );
                let distance2 = (candidate - position).length_squared();
                if distance2 < min_distance2 {
                    min_distance2 = distance2;
                    *projected = candidate;
                    home = *key;
                }
            }
            if min_distance2 < f64::MAX {
                return home;
            }
            radius *= 2.0;
        }
        // Straggler (smoothing pushed it past the search radius): fall back
        // to a full scan so every vertex still gets a home triangle.
        let mut min_distance2 = f64::MAX;
        for (key, triangle) in triangles.iter().enumerate() {
            let candidate = closest_point_on_triangle(
                position,
                vertices[triangle[0]],
                vertices[triangle[1]],
                vertices[triangle[2]],
            );
            let distance2 = (candidate - position).length_squared();
            if distance2 < min_distance2 {
                min_distance2 = distance2;
                *projected = candidate;
                home = key;
            }
        }
        home
    }

    /// Shared `hasRepeatedVertex` lambda (repeated verbatim in every
    /// cleanup pass in the C++).
    fn has_repeated_vertex(face: &[usize]) -> bool {
        let unique: CxxSet = face.iter().copied().collect();
        unique.len() != face.len()
    }

    /// Shared `canonicalFace` lambda: lexicographically smallest rotation
    /// of either winding.
    fn canonical_face(face: &[usize]) -> Vec<usize> {
        let mut best = Vec::new();
        let reversed: Vec<usize> = face.iter().rev().copied().collect();
        for winding in [face, &reversed] {
            for start in 0..winding.len() {
                let mut candidate = Vec::with_capacity(winding.len());
                for i in 0..winding.len() {
                    candidate.push(winding[(start + i) % winding.len()]);
                }
                if best.is_empty() || candidate < best {
                    best = candidate;
                }
            }
        }
        best
    }

    /// Shared `valenceScore` lambda: distance from the four neighbors a
    /// quad point wants.
    fn valence_score(valence: usize) -> i32 {
        if valence > 4 {
            (valence - 4) as i32
        } else {
            (4 - valence) as i32
        }
    }

    /// Shared plain `faceNormal` lambda (fan around `face[0]`, positions
    /// read straight from the mesh).
    fn face_normal_of(vertices: &[Vector3], face: &[usize]) -> Vector3 {
        let mut normal = Vector3::default();
        // `saturating_sub` keeps the C++ loop condition (`i + 1 < size`,
        // empty for short faces) total on degenerate input.
        for i in 1..face.len().saturating_sub(1) {
            normal += Vector3::cross_product(
                &(vertices[face[i]] - vertices[face[0]]),
                &(vertices[face[i + 1]] - vertices[face[0]]),
            );
        }
        normal
    }

    /// Shared `faceNormal` lambda with a phantom vertex (the `addedVertex`
    /// / `movedVertex` pattern): `added_vertex` reads `added_position`.
    fn face_normal_with_added(
        vertices: &[Vector3],
        face: &[usize],
        added_vertex: usize,
        added_position: Vector3,
    ) -> Vector3 {
        let position_of = |vertex: usize| {
            if vertex == added_vertex {
                added_position
            } else {
                vertices[vertex]
            }
        };
        let mut normal = Vector3::default();
        if face.len() < 3 {
            return normal;
        }
        let origin = position_of(face[0]);
        for i in 1..face.len() - 1 {
            normal += Vector3::cross_product(
                &(position_of(face[i]) - origin),
                &(position_of(face[i + 1]) - origin),
            );
        }
        normal
    }

    /// Shared `cornerScore` lambda (mean |dot| of corner legs, negated).
    fn corner_score_of(vertices: &[Vector3], quad: &[usize]) -> f64 {
        let mut total = 0.0;
        for i in 0..quad.len() {
            let h = (i + quad.len() - 1) % quad.len();
            let j = (i + 1) % quad.len();
            let left = (vertices[quad[h]] - vertices[quad[i]]).normalized();
            let right = (vertices[quad[j]] - vertices[quad[i]]).normalized();
            total += Vector3::dot_product(&left, &right).abs();
        }
        -total / quad.len() as f64
    }

    /// Shared `flowScoreAt` lambda (best incoming-edge alignment outside
    /// the skipped neighbors, 0 when none).
    fn flow_score_at(
        vertices: &[Vector3],
        vertex_neighbors: &BTreeMap<usize, BTreeSet<usize>>,
        vertex: usize,
        direction: Vector3,
        skip_neighbors: &BTreeSet<usize>,
    ) -> f64 {
        let mut best = -1.0;
        let Some(find_neighbors) = vertex_neighbors.get(&vertex) else {
            return 0.0;
        };
        let mut found = false;
        for neighbor in find_neighbors {
            if skip_neighbors.contains(neighbor) {
                continue;
            }
            let incoming = (vertices[vertex] - vertices[*neighbor]).normalized();
            let score = Vector3::dot_product(&incoming, &direction);
            if score > best {
                best = score;
            }
            found = true;
        }
        if found { best } else { 0.0 }
    }

    fn split_six_edge_faces(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        for face in &self.remeshed_polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
            }
        }

        let mut split_num = 0;
        let mut polygons = Vec::with_capacity(self.remeshed_polygons.len());
        for face in &self.remeshed_polygons {
            if 6 != face.len() {
                polygons.push(face.clone());
                continue;
            }
            if Self::has_repeated_vertex(face) {
                polygons.push(face.clone());
                continue;
            }

            let mut best_corner = -1;
            let mut best_score = 0.0;
            for i in 0..3 {
                let a = face[i];
                let b = face[i + 3];
                if vertex_neighbors
                    .get(&a)
                    .is_some_and(|neighbors| neighbors.contains(&b))
                {
                    continue;
                }
                let direction =
                    (self.remeshed_vertices[b] - self.remeshed_vertices[a]).normalized();
                let a_face_neighbors = BTreeSet::from([face[(i + 5) % 6], face[(i + 1) % 6], b]);
                let b_face_neighbors = BTreeSet::from([face[(i + 2) % 6], face[(i + 4) % 6], a]);
                let mut score = Self::flow_score_at(
                    &self.remeshed_vertices,
                    &vertex_neighbors,
                    a,
                    direction,
                    &a_face_neighbors,
                ) + Self::flow_score_at(
                    &self.remeshed_vertices,
                    &vertex_neighbors,
                    b,
                    -direction,
                    &b_face_neighbors,
                );
                score += Self::corner_score_of(
                    &self.remeshed_vertices,
                    &[face[i], face[(i + 1) % 6], face[(i + 2) % 6], face[i + 3]],
                );
                score += Self::corner_score_of(
                    &self.remeshed_vertices,
                    &[face[i + 3], face[(i + 4) % 6], face[(i + 5) % 6], face[i]],
                );
                if -1 == best_corner || score > best_score {
                    best_corner = i as i32;
                    best_score = score;
                }
            }
            if -1 == best_corner {
                self.diagnose(|| "Six edge face kept, no diagonal available\n".to_string());
                polygons.push(face.clone());
                continue;
            }

            let i = best_corner as usize;
            polygons.push(vec![
                face[i],
                face[(i + 1) % 6],
                face[(i + 2) % 6],
                face[i + 3],
            ]);
            polygons.push(vec![
                face[i + 3],
                face[(i + 4) % 6],
                face[(i + 5) % 6],
                face[i],
            ]);
            split_num += 1;
            vertex_neighbors
                .entry(face[i])
                .or_default()
                .insert(face[i + 3]);
            vertex_neighbors
                .entry(face[i + 3])
                .or_default()
                .insert(face[i]);
        }

        if 0 == split_num {
            return;
        }

        self.diagnose(|| format!("Split six edge faces:{split_num}\n"));
        self.remeshed_polygons = polygons;
        self.rebuild_half_edges();
    }

    fn split_seven_edge_faces(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        for face in &self.remeshed_polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
            }
        }

        let mut split_num = 0;
        let mut polygons = Vec::with_capacity(self.remeshed_polygons.len());
        for face in &self.remeshed_polygons {
            if 7 != face.len() {
                polygons.push(face.clone());
                continue;
            }
            if Self::has_repeated_vertex(face) {
                polygons.push(face.clone());
                continue;
            }

            let mut best_corner = -1;
            let mut best_score = 0.0;
            for i in 0..7 {
                let a = face[i];
                let b = face[(i + 3) % 7];
                if vertex_neighbors
                    .get(&a)
                    .is_some_and(|neighbors| neighbors.contains(&b))
                {
                    continue;
                }
                let direction =
                    (self.remeshed_vertices[b] - self.remeshed_vertices[a]).normalized();
                let a_face_neighbors = BTreeSet::from([face[(i + 6) % 7], face[(i + 1) % 7], b]);
                let b_face_neighbors = BTreeSet::from([face[(i + 2) % 7], face[(i + 4) % 7], a]);
                let mut score = Self::flow_score_at(
                    &self.remeshed_vertices,
                    &vertex_neighbors,
                    a,
                    direction,
                    &a_face_neighbors,
                ) + Self::flow_score_at(
                    &self.remeshed_vertices,
                    &vertex_neighbors,
                    b,
                    -direction,
                    &b_face_neighbors,
                );
                score += Self::corner_score_of(
                    &self.remeshed_vertices,
                    &[
                        face[i],
                        face[(i + 1) % 7],
                        face[(i + 2) % 7],
                        face[(i + 3) % 7],
                    ],
                );
                score += Self::corner_score_of(
                    &self.remeshed_vertices,
                    &[
                        face[(i + 3) % 7],
                        face[(i + 4) % 7],
                        face[(i + 5) % 7],
                        face[(i + 6) % 7],
                        face[i],
                    ],
                );
                if -1 == best_corner || score > best_score {
                    best_corner = i as i32;
                    best_score = score;
                }
            }
            if -1 == best_corner {
                self.diagnose(|| "Seven edge face kept, no diagonal available\n".to_string());
                polygons.push(face.clone());
                continue;
            }

            let i = best_corner as usize;
            polygons.push(vec![
                face[i],
                face[(i + 1) % 7],
                face[(i + 2) % 7],
                face[(i + 3) % 7],
            ]);
            polygons.push(vec![
                face[(i + 3) % 7],
                face[(i + 4) % 7],
                face[(i + 5) % 7],
                face[(i + 6) % 7],
                face[i],
            ]);
            split_num += 1;
            vertex_neighbors
                .entry(face[i])
                .or_default()
                .insert(face[(i + 3) % 7]);
            vertex_neighbors
                .entry(face[(i + 3) % 7])
                .or_default()
                .insert(face[i]);
        }

        if 0 == split_num {
            return;
        }

        self.diagnose(|| format!("Split seven edge faces:{split_num}\n"));
        self.remeshed_polygons = polygons;
        self.rebuild_half_edges();
    }

    /// Shared `fanAround` lambda (identical in the three fan passes):
    /// walks the closed face fan around `vertex` through shared edges.
    /// The C++ reads `edgeFaces` through non-const `operator[]`, inserting
    /// an empty entry on missing keys; the mirror uses `get`. The inserted
    /// empties are unobservable: a missing edge reads as size != 2 either
    /// way (failing the fan walk and, in the one pass whose main loop
    /// iterates `edgeFaces`, its `2 != faces.size()` guard), and the maps
    /// are rebuilt every round.
    fn fan_around(
        polygons: &[Vec<usize>],
        vertex_faces: &BTreeMap<usize, Vec<usize>>,
        edge_faces: &BTreeMap<(usize, usize), Vec<usize>>,
        vertex: usize,
        fan: &mut Vec<usize>,
    ) -> bool {
        fan.clear();
        let Some(find_faces) = vertex_faces.get(&vertex) else {
            return false;
        };
        if find_faces.is_empty() {
            return false;
        }
        let start_face = find_faces[0];
        let mut current_face = start_face;
        loop {
            fan.push(current_face);
            if fan.len() > find_faces.len() {
                return false;
            }
            let face = &polygons[current_face];
            let mut at = face.len();
            for (i, corner) in face.iter().enumerate() {
                if vertex == *corner {
                    at = i;
                    break;
                }
            }
            if at >= face.len() {
                return false;
            }
            let next_edge = Self::edge_of(vertex, face[(at + 1) % face.len()]);
            let Some(incident) = edge_faces.get(&next_edge) else {
                return false;
            };
            if 2 != incident.len() {
                return false;
            }
            current_face = if incident[0] == current_face {
                incident[1]
            } else {
                incident[0]
            };
            if current_face == start_face {
                break;
            }
        }
        fan.len() == find_faces.len()
    }

    fn convert_triangle_and_five_edge_fans(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const MAX_VALENCE: usize = 6;
        const MIN_FAN_VALENCE: usize = 4;
        const MAX_FAN_VALENCE: usize = 6;

        let mut convert_num = 0;
        let mut converted_vertices = BTreeSet::new();
        loop {
            let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
            let mut vertex_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    edge_faces
                        .entry(Self::edge_of(face[i], face[j]))
                        .or_default()
                        .push(face_index);
                    vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                    vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
                }
                for vertex in face {
                    vertex_faces.entry(*vertex).or_default().push(face_index);
                }
            }

            let mut border_vertices = BTreeSet::new();
            for (edge, faces) in &edge_faces {
                if 2 == faces.len() {
                    continue;
                }
                border_vertices.insert(edge.0);
                border_vertices.insert(edge.1);
            }

            let mut touched_vertices = BTreeSet::new();
            let mut round_convert_num = 0;
            let mut fan_vertices = BTreeSet::new();
            for (source, neighbors) in &vertex_neighbors {
                if neighbors.len() >= MIN_FAN_VALENCE && neighbors.len() <= MAX_FAN_VALENCE {
                    fan_vertices.insert(*source);
                }
            }
            for vertex in fan_vertices {
                if touched_vertices.contains(&vertex) || border_vertices.contains(&vertex) {
                    continue;
                }
                let mut fan = Vec::new();
                let neighbor_count = vertex_neighbors
                    .get(&vertex)
                    .map_or(0, |neighbors| neighbors.len());
                if !Self::fan_around(
                    &self.remeshed_polygons,
                    &vertex_faces,
                    &edge_faces,
                    vertex,
                    &mut fan,
                ) || fan.len() != neighbor_count
                {
                    continue;
                }

                let mut triangle_position = fan.len();
                let mut pentagon_position = fan.len();
                let mut well_formed = true;
                for i in 0..fan.len() {
                    if !well_formed {
                        break;
                    }
                    let face = &self.remeshed_polygons[fan[i]];
                    if Self::has_repeated_vertex(face) {
                        well_formed = false;
                    } else if 3 == face.len() {
                        if triangle_position < fan.len() {
                            well_formed = false;
                        }
                        triangle_position = i;
                    } else if 5 == face.len() {
                        if pentagon_position < fan.len() {
                            well_formed = false;
                        }
                        pentagon_position = i;
                    } else if 4 != face.len() {
                        well_formed = false;
                    }
                }
                if !well_formed || triangle_position >= fan.len() || pentagon_position >= fan.len()
                {
                    continue;
                }

                let forward_gap = (pentagon_position + fan.len() - triangle_position) % fan.len();
                let backward_gap = fan.len() - forward_gap;
                if 2 != forward_gap.min(backward_gap) {
                    continue;
                }
                let run_start = if forward_gap <= backward_gap {
                    triangle_position
                } else {
                    pentagon_position
                };
                let mut run = Vec::with_capacity(3);
                for i in 0..3 {
                    run.push(fan[(run_start + i) % fan.len()]);
                }

                let mut directed_edges = BTreeSet::new();
                for face_index in &run {
                    let face = &self.remeshed_polygons[*face_index];
                    for i in 0..face.len() {
                        directed_edges.insert((face[i], face[(i + 1) % face.len()]));
                    }
                }
                let mut boundary_next: BTreeMap<usize, usize> = BTreeMap::new();
                let mut buried_edges = Vec::new();
                let mut simple_boundary = true;
                for (from, to) in &directed_edges {
                    if directed_edges.contains(&(*to, *from)) {
                        if from < to {
                            buried_edges.push(Self::edge_of(*from, *to));
                        }
                        continue;
                    }
                    if boundary_next.insert(*from, *to).is_some() {
                        simple_boundary = false;
                        break;
                    }
                }
                if !simple_boundary || 8 != boundary_next.len() || 2 != buried_edges.len() {
                    continue;
                }
                let mut buried_at_fan_vertex = true;
                for edge in &buried_edges {
                    if vertex != edge.0 && vertex != edge.1 {
                        buried_at_fan_vertex = false;
                        break;
                    }
                }
                if !buried_at_fan_vertex {
                    continue;
                }

                let mut octagon = Vec::with_capacity(8);
                let mut walk = vertex;
                for _ in 0..8 {
                    octagon.push(walk);
                    let Some(find_next) = boundary_next.get(&walk) else {
                        break;
                    };
                    walk = *find_next;
                }
                if 8 != octagon.len() || walk != vertex || Self::has_repeated_vertex(&octagon) {
                    continue;
                }
                let mut touched = false;
                for corner in &octagon {
                    if touched_vertices.contains(corner) {
                        touched = true;
                        break;
                    }
                }
                if touched {
                    continue;
                }

                let mut lopsided = false;
                for i in (0..8).step_by(2) {
                    if lopsided {
                        break;
                    }
                    // Wrapping: mirrors C++ unsigned arithmetic exactly
                    // (underflow is believed impossible here — buried edges
                    // are mesh edges, a subset of the counted neighbors —
                    // but wrap matches the C++ either way, where plain `-`
                    // would panic in debug builds).
                    let mut valence = vertex_neighbors
                        .get(&octagon[i])
                        .map_or(0, |neighbors| neighbors.len());
                    for edge in &buried_edges {
                        if octagon[i] == edge.0 || octagon[i] == edge.1 {
                            valence = valence.wrapping_sub(1);
                        }
                    }
                    valence = valence.wrapping_add(1);
                    if valence > MAX_VALENCE {
                        lopsided = true;
                    }
                }
                if lopsided {
                    continue;
                }

                let added_vertex = self.remeshed_vertices.len();
                let mut added_position = Vector3::default();
                for i in (0..8).step_by(2) {
                    added_position += self.remeshed_vertices[octagon[i]];
                }
                added_position /= 4.0;

                let mut quads = Vec::with_capacity(4);
                for i in (0..8).step_by(2) {
                    quads.push(vec![
                        octagon[i],
                        octagon[(i + 1) % 8],
                        octagon[(i + 2) % 8],
                        added_vertex,
                    ]);
                }

                let mut old_normal = Vector3::default();
                for face_index in &run {
                    old_normal += Self::face_normal_with_added(
                        &self.remeshed_vertices,
                        &self.remeshed_polygons[*face_index],
                        added_vertex,
                        added_position,
                    );
                }
                let mut shaped = true;
                for quad in &quads {
                    if Vector3::dot_product(
                        &old_normal,
                        &Self::face_normal_with_added(
                            &self.remeshed_vertices,
                            quad,
                            added_vertex,
                            added_position,
                        ),
                    ) <= 0.0
                    {
                        shaped = false;
                        break;
                    }
                }
                if !shaped {
                    continue;
                }

                self.remeshed_vertices.push(added_position);
                for (i, quad) in quads.into_iter().enumerate() {
                    if i < run.len() {
                        self.remeshed_polygons[run[i]] = quad;
                    } else {
                        self.remeshed_polygons.push(quad);
                    }
                }
                touched_vertices.extend(octagon.iter().copied());
                converted_vertices.extend(octagon.iter().copied());
                converted_vertices.insert(added_vertex);
                round_convert_num += 1;
            }

            if 0 == round_convert_num {
                break;
            }
            convert_num += round_convert_num;
        }

        if 0 == convert_num {
            return;
        }

        self.diagnose(|| format!("Convert triangle and five edge fans:{convert_num}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&converted_vertices, 3, 5);
    }

    /// Shared vertex-compaction epilogue (identical in the five collapse /
    /// merge passes): drops vertices no face references (first-use order),
    /// rewrites faces, and remaps the seed set. Returns the remapped seeds.
    fn compact_vertices(&mut self, seed_vertices: &BTreeSet<usize>) -> BTreeSet<usize> {
        let mut old_to_new = BTreeMap::new();
        let mut compacted_vertices = Vec::new();
        for face in &self.remeshed_polygons {
            for vertex in face {
                if old_to_new.contains_key(vertex) {
                    continue;
                }
                old_to_new.insert(*vertex, compacted_vertices.len());
                compacted_vertices.push(self.remeshed_vertices[*vertex]);
            }
        }
        for face in &mut self.remeshed_polygons {
            for vertex in face {
                *vertex = old_to_new[vertex];
            }
        }
        self.remeshed_vertices = compacted_vertices;
        let mut compacted_seeds = BTreeSet::new();
        for vertex in seed_vertices {
            if let Some(new) = old_to_new.get(vertex) {
                compacted_seeds.insert(*new);
            }
        }
        compacted_seeds
    }

    fn collapse_three_valence_diagonals(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const NO_VERTEX: usize = usize::MAX;
        const NO_FACE: usize = usize::MAX;
        // The quad closes up along the diagonal, so the two corners left
        // standing each hand one neighbor over to the point in the middle.
        // Three neighbors is as low as a corner may go before it turns into
        // a doublet, so the sides need four
        const MIN_SIDE_VALENCE: usize = 4;

        // Two three valence points facing each other across a quad are the
        // two halves of a single point which the extraction split apart,
        // the quad between them is the gap. Collapsing the diagonal puts a
        // point back in the middle carrying the four neighbors the pair had
        // between them, and takes the quad away along with both
        // singularities, the sides paying for it with one neighbor each
        let mut rejected_diagonals = BTreeSet::new();
        let mut collapsed_vertices = BTreeSet::new();
        let mut collapse_count = 0;
        loop {
            // Sorted-vector rebuild (identical keys/values/order to the
            // `BTreeMap` builds, a handful of allocs instead of 200k+).
            let edge_faces = EdgeFaceIndex::build(&self.remeshed_polygons);
            let vertex_neighbors = NeighborIndex::build(&self.remeshed_polygons);
            let vertex_face_counts = FaceCountIndex::build(&self.remeshed_polygons);

            // A three valence point on a border is what a border looks
            // like, not a defect
            let mut border_vertices = BTreeSet::new();
            for (edge, faces) in edge_faces.groups() {
                if 2 == faces.len() {
                    continue;
                }
                border_vertices.insert(edge.0);
                border_vertices.insert(edge.1);
            }

            let neighbor_count = |vertex_neighbors: &NeighborIndex, vertex: usize| {
                vertex_neighbors
                    .get(&vertex)
                    .map_or(0, |neighbors| neighbors.len())
            };

            let mut collapsing_face = NO_FACE;
            let mut first = NO_VERTEX;
            let mut second = NO_VERTEX;
            let mut left = NO_VERTEX;
            let mut right = NO_VERTEX;
            for face_index in 0..self.remeshed_polygons.len() {
                if NO_FACE != collapsing_face {
                    break;
                }
                let face = &self.remeshed_polygons[face_index];
                if 4 != face.len() || Self::has_repeated_vertex(face) {
                    continue;
                }
                let mut on_border = false;
                for vertex in face {
                    if border_vertices.contains(vertex) {
                        on_border = true;
                        break;
                    }
                }
                if on_border {
                    continue;
                }
                for i in 0..2 {
                    let diagonal_first = face[i];
                    let diagonal_second = face[i + 2];
                    if rejected_diagonals.contains(&Self::edge_of(diagonal_first, diagonal_second))
                    {
                        continue;
                    }
                    if 3 != neighbor_count(&vertex_neighbors, diagonal_first)
                        || 3 != neighbor_count(&vertex_neighbors, diagonal_second)
                    {
                        continue;
                    }
                    // A closed fan of three faces is the only shape a three
                    // valence point may have here, anything else is not the
                    // pair this is looking for
                    if 3 != vertex_face_counts
                        .get(&diagonal_first)
                        .copied()
                        .unwrap_or(0)
                        || 3 != vertex_face_counts
                            .get(&diagonal_second)
                            .copied()
                            .unwrap_or(0)
                    {
                        continue;
                    }
                    let diagonal_left = face[(i + 1) % 4];
                    let diagonal_right = face[(i + 3) % 4];
                    if neighbor_count(&vertex_neighbors, diagonal_left) < MIN_SIDE_VALENCE
                        || neighbor_count(&vertex_neighbors, diagonal_right) < MIN_SIDE_VALENCE
                    {
                        continue;
                    }
                    // The two fans are only allowed to meet at the sides of
                    // the quad, any other point shared between them, an edge
                    // included, would fold the faces of the merged point
                    // over each other
                    if vertex_neighbors
                        .get(&diagonal_first)
                        .is_some_and(|neighbors| neighbors.contains(&diagonal_second))
                    {
                        continue;
                    }
                    let mut shared_num = 0;
                    if let Some(neighbors) = vertex_neighbors.get(&diagonal_first) {
                        for neighbor in neighbors {
                            if vertex_neighbors
                                .get(&diagonal_second)
                                .is_some_and(|second_neighbors| second_neighbors.contains(neighbor))
                            {
                                shared_num += 1;
                            }
                        }
                    }
                    if 2 != shared_num {
                        continue;
                    }
                    collapsing_face = face_index;
                    first = diagonal_first;
                    second = diagonal_second;
                    left = diagonal_left;
                    right = diagonal_right;
                    break;
                }
            }
            if NO_FACE == collapsing_face {
                break;
            }

            let added_vertex = self.remeshed_vertices.len();
            let added_position =
                (self.remeshed_vertices[first] + self.remeshed_vertices[second]) * 0.5;

            // Per-face remap is independent given the fixed collapse
            // pair; validation keeps its serial face-order scan (same
            // checks, same order, same early exit), so the outcome is
            // identical.
            let mut remapped: Vec<(Option<Vec<usize>>, bool)> =
                vec![(None, false); self.remeshed_polygons.len()];
            parallel_each(&mut remapped, |face_index, slot| {
                if collapsing_face == face_index {
                    return;
                }
                let face = &self.remeshed_polygons[face_index];
                let mut face_affected = false;
                for vertex in face {
                    if first == *vertex || second == *vertex {
                        face_affected = true;
                        break;
                    }
                }
                if !face_affected {
                    *slot = (Some(face.clone()), false);
                    return;
                }
                let mut candidate = Vec::with_capacity(face.len());
                for vertex in face {
                    let rewritten_vertex = if first == *vertex || second == *vertex {
                        added_vertex
                    } else {
                        *vertex
                    };
                    if candidate.is_empty() || candidate[candidate.len() - 1] != rewritten_vertex {
                        candidate.push(rewritten_vertex);
                    }
                }
                if candidate.len() > 1 && candidate[0] == candidate[candidate.len() - 1] {
                    candidate.pop();
                }
                *slot = (Some(candidate), true);
            });
            let mut rewritten: Vec<Vec<usize>> = Vec::with_capacity(self.remeshed_polygons.len());
            let mut affected = Vec::with_capacity(self.remeshed_polygons.len());
            let mut valid = true;
            for (face_index, (candidate, face_affected)) in remapped.into_iter().enumerate() {
                if !valid {
                    break;
                }
                let Some(candidate) = candidate else {
                    continue;
                };
                let face = &self.remeshed_polygons[face_index];
                if !face_affected {
                    rewritten.push(candidate);
                    affected.push(false);
                    continue;
                }
                // The quad is the only face allowed to disappear, the fans
                // around the pair keep every side they came in with
                if candidate.len() != face.len() || Self::has_repeated_vertex(&candidate) {
                    valid = false;
                    break;
                }
                if Vector3::dot_product(
                    &Self::face_normal_with_added(
                        &self.remeshed_vertices,
                        face,
                        added_vertex,
                        added_position,
                    ),
                    &Self::face_normal_with_added(
                        &self.remeshed_vertices,
                        &candidate,
                        added_vertex,
                        added_position,
                    ),
                ) <= 0.0
                {
                    valid = false;
                    break;
                }
                rewritten.push(candidate);
                affected.push(true);
            }

            if valid {
                let mut unique_faces = BTreeSet::new();
                for (face_index, face) in rewritten.iter().enumerate() {
                    if !affected[face_index] {
                        unique_faces.insert(Self::canonical_face(face));
                    }
                }
                let mut added_edge_counts: BTreeMap<(usize, usize), usize> = BTreeMap::new();
                for (face_index, face) in rewritten.iter().enumerate() {
                    if !valid {
                        break;
                    }
                    if affected[face_index] && !unique_faces.insert(Self::canonical_face(face)) {
                        valid = false;
                        break;
                    }
                    for i in 0..face.len() {
                        let edge = Self::edge_of(face[i], face[(i + 1) % face.len()]);
                        if added_vertex != edge.0 && added_vertex != edge.1 {
                            continue;
                        }
                        *added_edge_counts.entry(edge).or_default() += 1;
                        if added_edge_counts[&edge] > 2 {
                            valid = false;
                            break;
                        }
                    }
                }
            }
            if !valid {
                rejected_diagonals.insert(Self::edge_of(first, second));
                continue;
            }

            self.remeshed_vertices.push(added_position);
            self.remeshed_polygons = rewritten;
            collapsed_vertices.insert(added_vertex);
            collapsed_vertices.insert(left);
            collapsed_vertices.insert(right);
            collapse_count += 1;
        }

        if 0 == collapse_count {
            return;
        }

        // The two collapsed points are left with no face of their own
        let compacted = self.compact_vertices(&collapsed_vertices);

        self.diagnose(|| format!("Collapse three valence diagonals:{collapse_count}\n"));
        self.rebuild_half_edges();

        // The point in the middle came from the diagonal, not from the
        // source mesh, pull the patch which closed up around it back onto
        // the surface
        self.smooth_around_vertices(&compacted, 3, 5);
    }

    fn merge_double_shared_edge_quads(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        let index_of_vertex = |face: &[usize], vertex: usize| {
            for (i, corner) in face.iter().enumerate() {
                if vertex == *corner {
                    return i;
                }
            }
            face.len()
        };

        let mut merge_num = 0;
        let mut merged_vertices = BTreeSet::new();
        loop {
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
            let mut vertex_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                    vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
                }
                for vertex in face {
                    vertex_faces.entry(*vertex).or_default().push(face_index);
                }
            }

            let mut middle_vertices = BTreeSet::new();
            for (source, neighbors) in &vertex_neighbors {
                if 2 != neighbors.len() {
                    continue;
                }
                match vertex_faces.get(source) {
                    Some(find_faces) if 2 == find_faces.len() => {}
                    _ => continue,
                }
                middle_vertices.insert(*source);
            }

            let mut existing_faces = BTreeSet::new();
            for face in &self.remeshed_polygons {
                existing_faces.insert(Self::canonical_face(face));
            }

            let mut touched_vertices = BTreeSet::new();
            let mut replaced_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            let mut removed_faces = BTreeSet::new();
            let mut round_merge_num = 0;
            for middle in middle_vertices {
                // Present with two faces by construction (checked above).
                let faces = match vertex_faces.get(&middle) {
                    Some(faces) if 2 == faces.len() => faces.clone(),
                    _ => continue,
                };
                let first_face_index = faces[0];
                let second_face_index = faces[1];
                if first_face_index == second_face_index {
                    continue;
                }
                let first_face = &self.remeshed_polygons[first_face_index];
                let second_face = &self.remeshed_polygons[second_face_index];
                if 4 != first_face.len() || 4 != second_face.len() {
                    continue;
                }
                if Self::has_repeated_vertex(first_face) || Self::has_repeated_vertex(second_face) {
                    continue;
                }
                let first_at = index_of_vertex(first_face, middle);
                let second_at = index_of_vertex(second_face, middle);
                if first_at >= first_face.len() || second_at >= second_face.len() {
                    continue;
                }
                let previous = first_face[(first_at + 3) % 4];
                let next = first_face[(first_at + 1) % 4];
                if previous != second_face[(second_at + 1) % 4]
                    || next != second_face[(second_at + 3) % 4]
                {
                    continue;
                }
                let first_apex = first_face[(first_at + 2) % 4];
                let second_apex = second_face[(second_at + 2) % 4];
                if first_apex == second_apex {
                    continue;
                }
                let mut touched = false;
                for vertex in [middle, previous, next, first_apex, second_apex] {
                    if touched_vertices.contains(&vertex) {
                        touched = true;
                        break;
                    }
                }
                if touched {
                    continue;
                }

                let merged = vec![first_apex, previous, second_apex, next];
                if Self::has_repeated_vertex(&merged) {
                    continue;
                }
                if existing_faces.contains(&Self::canonical_face(&merged)) {
                    continue;
                }
                let first_normal = Self::face_normal_of(&self.remeshed_vertices, first_face);
                let second_normal = Self::face_normal_of(&self.remeshed_vertices, second_face);
                if Vector3::dot_product(
                    &(first_normal + second_normal),
                    &Self::face_normal_of(&self.remeshed_vertices, &merged),
                ) <= 0.0
                {
                    continue;
                }

                existing_faces.insert(Self::canonical_face(&merged));
                // First wins (C++ `insert`); keys are unique here (the
                // touched guard skips faces claimed by an earlier middle),
                // so this never overwrites either way.
                replaced_faces
                    .entry(first_face_index)
                    .or_insert(merged.clone());
                removed_faces.insert(second_face_index);
                touched_vertices.extend(merged.iter().copied());
                touched_vertices.insert(middle);
                merged_vertices.extend(merged.iter().copied());
                round_merge_num += 1;
            }

            if 0 == round_merge_num {
                break;
            }

            let mut rewritten =
                Vec::with_capacity(self.remeshed_polygons.len().saturating_sub(round_merge_num));
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                if removed_faces.contains(&face_index) {
                    continue;
                }
                if let Some(find_replaced) = replaced_faces.get(&face_index) {
                    rewritten.push(find_replaced.clone());
                    continue;
                }
                rewritten.push(face.clone());
            }
            self.remeshed_polygons = rewritten;
            merge_num += round_merge_num;
        }

        if 0 == merge_num {
            return;
        }

        let compacted = self.compact_vertices(&merged_vertices);

        self.diagnose(|| format!("Merge double shared edge quads:{merge_num}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&compacted, 3, 5);
    }

    fn merge_three_and_five_valence_triangles(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const MIN_VALENCE: usize = 3;

        let mut merge_num = 0;
        let mut merged_vertices = BTreeSet::new();
        loop {
            let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
            let mut vertex_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    edge_faces
                        .entry(Self::edge_of(face[i], face[j]))
                        .or_default()
                        .push(face_index);
                    vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                    vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
                }
                for vertex in face {
                    vertex_faces.entry(*vertex).or_default().push(face_index);
                }
            }

            let mut border_vertices = BTreeSet::new();
            for (edge, faces) in &edge_faces {
                if 2 == faces.len() {
                    continue;
                }
                border_vertices.insert(edge.0);
                border_vertices.insert(edge.1);
            }

            let mut existing_faces = BTreeSet::new();
            for face in &self.remeshed_polygons {
                existing_faces.insert(Self::canonical_face(face));
            }

            let mut touched_vertices = BTreeSet::new();
            let mut replaced_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            let mut removed_faces = BTreeSet::new();
            let mut round_merge_num = 0;
            for (edge, faces) in &edge_faces {
                if 2 != faces.len() {
                    continue;
                }

                let mut three = edge.0;
                let mut five = edge.1;
                // Present by construction (edge endpoints are used).
                if 3 != vertex_neighbors[&three].len() {
                    std::mem::swap(&mut three, &mut five);
                }
                if 3 != vertex_neighbors[&three].len() || 5 != vertex_neighbors[&five].len() {
                    continue;
                }
                if border_vertices.contains(&three) || border_vertices.contains(&five) {
                    continue;
                }

                let mut three_fan = Vec::new();
                let mut five_fan = Vec::new();
                if !Self::fan_around(
                    &self.remeshed_polygons,
                    &vertex_faces,
                    &edge_faces,
                    three,
                    &mut three_fan,
                ) || 3 != three_fan.len()
                {
                    continue;
                }
                if !Self::fan_around(
                    &self.remeshed_polygons,
                    &vertex_faces,
                    &edge_faces,
                    five,
                    &mut five_fan,
                ) || 5 != five_fan.len()
                {
                    continue;
                }
                let three_triangle_num =
                    match Self::count_fan_triangles(&self.remeshed_polygons, &three_fan) {
                        Some(three_triangle_num) => three_triangle_num,
                        _ => continue,
                    };
                let five_triangle_num =
                    match Self::count_fan_triangles(&self.remeshed_polygons, &five_fan) {
                        Some(five_triangle_num) => five_triangle_num,
                        _ => continue,
                    };
                if 1 != three_triangle_num || 1 != five_triangle_num {
                    continue;
                }

                let shared_faces = BTreeSet::from([faces[0], faces[1]]);
                let mut shared_at = five_fan.len();
                for i in 0..five_fan.len() {
                    if shared_faces.contains(&five_fan[i])
                        && shared_faces.contains(&five_fan[(i + 1) % five_fan.len()])
                    {
                        shared_at = i;
                        break;
                    }
                }
                if shared_at >= five_fan.len() {
                    continue;
                }
                let triangle_face = five_fan[(shared_at + 3) % five_fan.len()];
                if 3 != self.remeshed_polygons[triangle_face].len() {
                    continue;
                }

                let mut best_score = 0;
                let mut best_quads: Vec<Vec<usize>> = Vec::new();
                let mut best_run = Vec::new();
                let mut best_octagon = Vec::new();
                let mut best_position = Vector3::default();
                for direction in 0..2 {
                    let middle_face =
                        five_fan[(shared_at + if 0 == direction { 2 } else { 4 }) % five_fan.len()];
                    if 4 != self.remeshed_polygons[middle_face].len() {
                        continue;
                    }
                    let mut run = three_fan.clone();
                    run.push(middle_face);
                    run.push(triangle_face);

                    let mut directed_edges = BTreeSet::new();
                    for face_index in &run {
                        let face = &self.remeshed_polygons[*face_index];
                        for i in 0..face.len() {
                            directed_edges.insert((face[i], face[(i + 1) % face.len()]));
                        }
                    }
                    let mut boundary_next = BTreeMap::new();
                    let mut buried_edges = Vec::new();
                    let mut simple_boundary = true;
                    for directed_edge in &directed_edges {
                        let (from, to) = *directed_edge;
                        if directed_edges.contains(&(to, from)) {
                            if from < to {
                                buried_edges.push(Self::edge_of(from, to));
                            }
                            continue;
                        }
                        // Rust `insert` overwrites on a duplicate key where
                        // C++ `insert` keeps the first, but the map is
                        // discarded on this path either way.
                        if boundary_next.insert(from, to).is_some() {
                            simple_boundary = false;
                            break;
                        }
                    }
                    if !simple_boundary || 8 != boundary_next.len() || 5 != buried_edges.len() {
                        continue;
                    }
                    let mut buried_at_defect = true;
                    let mut buried_counts: BTreeMap<usize, usize> = BTreeMap::new();
                    for buried in &buried_edges {
                        let (first, second) = *buried;
                        if three != first && three != second && five != first && five != second {
                            buried_at_defect = false;
                            break;
                        }
                        *buried_counts.entry(first).or_insert(0) += 1;
                        *buried_counts.entry(second).or_insert(0) += 1;
                    }
                    // Present-or-zero (C++ `operator[]` on a count map).
                    if !buried_at_defect || 3 != buried_counts.get(&three).copied().unwrap_or(0) {
                        continue;
                    }

                    let mut octagon = Vec::with_capacity(8);
                    let mut walk = five;
                    for _ in 0..8 {
                        octagon.push(walk);
                        match boundary_next.get(&walk) {
                            Some(next) => walk = *next,
                            _ => break,
                        }
                    }
                    if 8 != octagon.len() || walk != five || Self::has_repeated_vertex(&octagon) {
                        continue;
                    }
                    let mut touched = false;
                    for corner in &octagon {
                        if touched_vertices.contains(corner) {
                            touched = true;
                            break;
                        }
                    }
                    if touched || touched_vertices.contains(&three) {
                        continue;
                    }

                    let mut new_score = 0;
                    let mut lopsided = false;
                    for (i, corner) in octagon.iter().enumerate() {
                        // Present by construction (octagon corners are used
                        // vertices); buried edges are distinct mesh edges,
                        // so the count never exceeds the valence.
                        let valence = vertex_neighbors[corner].len()
                            - buried_counts.get(corner).copied().unwrap_or(0)
                            + usize::from(0 == i % 2);
                        if valence < MIN_VALENCE {
                            lopsided = true;
                        }
                        new_score += Self::valence_score(valence);
                        if lopsided {
                            break;
                        }
                    }
                    if lopsided {
                        continue;
                    }
                    // Present by construction (three is used).
                    let mut old_score = Self::valence_score(vertex_neighbors[&three].len());
                    for corner in &octagon {
                        old_score += Self::valence_score(vertex_neighbors[corner].len());
                    }
                    if new_score > old_score {
                        continue;
                    }
                    if !best_quads.is_empty() && new_score >= best_score {
                        continue;
                    }

                    let added_vertex = self.remeshed_vertices.len();
                    let mut added_position = Vector3::default();
                    for i in (0..8).step_by(2) {
                        added_position += self.remeshed_vertices[octagon[i]];
                    }
                    added_position /= 4.0;

                    let mut quads = Vec::with_capacity(4);
                    for i in (0..8).step_by(2) {
                        quads.push(vec![
                            octagon[i],
                            octagon[(i + 1) % 8],
                            octagon[(i + 2) % 8],
                            added_vertex,
                        ]);
                    }

                    let mut old_normal = Vector3::default();
                    for face_index in &run {
                        old_normal += Self::face_normal_with_added(
                            &self.remeshed_vertices,
                            &self.remeshed_polygons[*face_index],
                            added_vertex,
                            added_position,
                        );
                    }
                    let mut shaped = true;
                    for quad in &quads {
                        if Vector3::dot_product(
                            &old_normal,
                            &Self::face_normal_with_added(
                                &self.remeshed_vertices,
                                quad,
                                added_vertex,
                                added_position,
                            ),
                        ) <= 0.0
                            || existing_faces.contains(&Self::canonical_face(quad))
                        {
                            shaped = false;
                            break;
                        }
                    }
                    if !shaped {
                        continue;
                    }

                    best_score = new_score;
                    best_quads = quads;
                    best_run = run;
                    best_octagon = octagon;
                    best_position = added_position;
                }
                if best_quads.is_empty() {
                    continue;
                }

                let added_vertex = self.remeshed_vertices.len();
                self.remeshed_vertices.push(best_position);
                for quad in &best_quads {
                    existing_faces.insert(Self::canonical_face(quad));
                }
                for (i, face_index) in best_run.iter().enumerate() {
                    // First wins (C++ `insert`); the run faces are
                    // distinct (the middle/triangle faces sit outside
                    // the edge's two faces), so this never overwrites
                    // either way.
                    if i < best_quads.len() {
                        replaced_faces
                            .entry(*face_index)
                            .or_insert(best_quads[i].clone());
                    } else {
                        removed_faces.insert(*face_index);
                    }
                }
                touched_vertices.extend(best_octagon.iter().copied());
                touched_vertices.insert(three);
                merged_vertices.extend(best_octagon.iter().copied());
                merged_vertices.insert(added_vertex);
                round_merge_num += 1;
            }

            if 0 == round_merge_num {
                break;
            }

            let mut rewritten = Vec::with_capacity(self.remeshed_polygons.len());
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                if removed_faces.contains(&face_index) {
                    continue;
                }
                if let Some(find_replaced) = replaced_faces.get(&face_index) {
                    rewritten.push(find_replaced.clone());
                    continue;
                }
                rewritten.push(face.clone());
            }
            self.remeshed_polygons = rewritten;
            merge_num += round_merge_num;
        }

        if 0 == merge_num {
            return;
        }

        let compacted = self.compact_vertices(&merged_vertices);

        self.diagnose(|| format!("Merge three and five valence triangles:{merge_num}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&compacted, 3, 5);
    }

    /// The 3/5-valence merge's `countTriangles`: number of triangles in
    /// `fan`, or `None` when a face repeats a vertex or is neither a
    /// triangle nor a quad.
    fn count_fan_triangles(polygons: &[Vec<usize>], fan: &[usize]) -> Option<usize> {
        let mut triangle_num = 0;
        for face_index in fan {
            let face = &polygons[*face_index];
            if Self::has_repeated_vertex(face) {
                return None;
            }
            if 3 == face.len() {
                triangle_num += 1;
            } else if 4 != face.len() {
                return None;
            }
        }
        Some(triangle_num)
    }

    fn split_high_valence_triangle_fans(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const MIN_VALENCE: usize = 3;
        const MIN_FAN_VALENCE: usize = 5;
        const TRIANGLE_PAYMENT: i32 = 2;

        let mut split_num = 0;
        let mut split_vertices = BTreeSet::new();
        loop {
            let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
            let mut vertex_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    edge_faces
                        .entry(Self::edge_of(face[i], face[j]))
                        .or_default()
                        .push(face_index);
                    vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                    vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
                }
                for vertex in face {
                    vertex_faces.entry(*vertex).or_default().push(face_index);
                }
            }

            let mut border_vertices = BTreeSet::new();
            for (edge, faces) in &edge_faces {
                if 2 == faces.len() {
                    continue;
                }
                border_vertices.insert(edge.0);
                border_vertices.insert(edge.1);
            }

            let mut touched_vertices = BTreeSet::new();
            let mut round_split_num = 0;
            let mut fan_vertices = BTreeSet::new();
            for (source, neighbors) in &vertex_neighbors {
                if neighbors.len() >= MIN_FAN_VALENCE {
                    fan_vertices.insert(*source);
                }
            }
            for vertex in fan_vertices {
                if touched_vertices.contains(&vertex) || border_vertices.contains(&vertex) {
                    continue;
                }
                let mut fan = Vec::new();
                // Present by construction (fan vertices come from the map).
                let valence = vertex_neighbors[&vertex].len();
                if !Self::fan_around(
                    &self.remeshed_polygons,
                    &vertex_faces,
                    &edge_faces,
                    vertex,
                    &mut fan,
                ) || fan.len() != valence
                {
                    continue;
                }

                let mut best_score = 0;
                let mut best_quads: Vec<Vec<usize>> = Vec::new();
                let mut best_run = Vec::new();
                let mut best_octagon = Vec::new();
                let mut best_position = Vector3::default();
                for run_start in 0..fan.len() {
                    let odd_size = self.remeshed_polygons[fan[run_start]].len();
                    if 3 != odd_size && 4 != odd_size {
                        continue;
                    }
                    let even_size = if 3 == odd_size { 4 } else { 3 };
                    let mut run = Vec::with_capacity(4);
                    for i in 0..4 {
                        let face_index = fan[(run_start + i) % fan.len()];
                        let face = &self.remeshed_polygons[face_index];
                        if face.len() != (if 0 == i % 2 { odd_size } else { even_size })
                            || Self::has_repeated_vertex(face)
                        {
                            break;
                        }
                        run.push(face_index);
                    }
                    if 4 != run.len() {
                        continue;
                    }

                    let mut directed_edges = BTreeSet::new();
                    for face_index in &run {
                        let face = &self.remeshed_polygons[*face_index];
                        for i in 0..face.len() {
                            directed_edges.insert((face[i], face[(i + 1) % face.len()]));
                        }
                    }
                    let mut boundary_next = BTreeMap::new();
                    let mut buried_edges = Vec::new();
                    let mut simple_boundary = true;
                    for directed_edge in &directed_edges {
                        let (from, to) = *directed_edge;
                        if directed_edges.contains(&(to, from)) {
                            if from < to {
                                buried_edges.push(Self::edge_of(from, to));
                            }
                            continue;
                        }
                        // Rust `insert` overwrites on a duplicate key where
                        // C++ `insert` keeps the first, but the map is
                        // discarded on this path either way.
                        if boundary_next.insert(from, to).is_some() {
                            simple_boundary = false;
                            break;
                        }
                    }
                    if !simple_boundary || 8 != boundary_next.len() || 3 != buried_edges.len() {
                        continue;
                    }
                    let mut buried_at_fan_vertex = true;
                    let mut buried_counts: BTreeMap<usize, usize> = BTreeMap::new();
                    for buried in &buried_edges {
                        let (first, second) = *buried;
                        if vertex != first && vertex != second {
                            buried_at_fan_vertex = false;
                            break;
                        }
                        *buried_counts.entry(first).or_insert(0) += 1;
                        *buried_counts.entry(second).or_insert(0) += 1;
                    }
                    if !buried_at_fan_vertex {
                        continue;
                    }

                    let mut octagon = Vec::with_capacity(8);
                    let mut walk = vertex;
                    for _ in 0..8 {
                        octagon.push(walk);
                        match boundary_next.get(&walk) {
                            Some(next) => walk = *next,
                            _ => break,
                        }
                    }
                    if 8 != octagon.len() || walk != vertex || Self::has_repeated_vertex(&octagon) {
                        continue;
                    }
                    let broken_at = octagon[4];
                    if !buried_edges.contains(&Self::edge_of(vertex, broken_at)) {
                        continue;
                    }
                    let mut touched = false;
                    for corner in &octagon {
                        if touched_vertices.contains(corner) {
                            touched = true;
                            break;
                        }
                    }
                    if touched {
                        continue;
                    }

                    let mut new_score = 0;
                    let mut lopsided = false;
                    for (i, corner) in octagon.iter().enumerate() {
                        // Present by construction (octagon corners are used
                        // vertices); buried edges are distinct mesh edges,
                        // so the count never exceeds the valence.
                        let valence = vertex_neighbors[corner].len()
                            - buried_counts.get(corner).copied().unwrap_or(0)
                            + usize::from(0 == i % 2);
                        if valence < MIN_VALENCE {
                            lopsided = true;
                        }
                        new_score += Self::valence_score(valence);
                        if lopsided {
                            break;
                        }
                    }
                    if lopsided {
                        continue;
                    }
                    let mut old_score = TRIANGLE_PAYMENT;
                    for corner in &octagon {
                        old_score += Self::valence_score(vertex_neighbors[corner].len());
                    }
                    if new_score > old_score {
                        continue;
                    }
                    if !best_quads.is_empty() && new_score >= best_score {
                        continue;
                    }

                    let added_vertex = self.remeshed_vertices.len();
                    let added_position =
                        (self.remeshed_vertices[vertex] + self.remeshed_vertices[broken_at]) * 0.5;

                    let mut quads = Vec::with_capacity(4);
                    for i in (0..8).step_by(2) {
                        quads.push(vec![
                            octagon[i],
                            octagon[(i + 1) % 8],
                            octagon[(i + 2) % 8],
                            added_vertex,
                        ]);
                    }

                    let mut old_normal = Vector3::default();
                    for face_index in &run {
                        old_normal += Self::face_normal_with_added(
                            &self.remeshed_vertices,
                            &self.remeshed_polygons[*face_index],
                            added_vertex,
                            added_position,
                        );
                    }
                    let mut shaped = true;
                    for quad in &quads {
                        if Vector3::dot_product(
                            &old_normal,
                            &Self::face_normal_with_added(
                                &self.remeshed_vertices,
                                quad,
                                added_vertex,
                                added_position,
                            ),
                        ) <= 0.0
                        {
                            shaped = false;
                            break;
                        }
                    }
                    if !shaped {
                        continue;
                    }

                    best_score = new_score;
                    best_quads = quads;
                    best_run = run;
                    best_octagon = octagon;
                    best_position = added_position;
                }
                if best_quads.is_empty() {
                    continue;
                }

                split_vertices.insert(self.remeshed_vertices.len());
                self.remeshed_vertices.push(best_position);
                for (i, face_index) in best_run.iter().enumerate() {
                    self.remeshed_polygons[*face_index] = best_quads[i].clone();
                }
                touched_vertices.extend(best_octagon.iter().copied());
                split_vertices.extend(best_octagon.iter().copied());
                round_split_num += 1;
            }

            if 0 == round_split_num {
                break;
            }
            split_num += round_split_num;
        }

        if 0 == split_num {
            return;
        }

        self.diagnose(|| format!("Split high valence triangle fans:{split_num}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&split_vertices, 3, 5);
    }

    fn collapse_three_valence_edge_pairs(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const MIN_VALENCE: usize = 3;
        const HEXAGON_SIZE: usize = 6;
        const PATCH_FACE_NUM: usize = 4;
        const PATCH_BURIED_EDGE_NUM: usize = 5;

        let mut collapse_count = 0;
        let mut collapsed_vertices = BTreeSet::new();
        loop {
            let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
            let mut vertex_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    edge_faces
                        .entry(Self::edge_of(face[i], face[j]))
                        .or_default()
                        .push(face_index);
                    vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                    vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
                }
                for vertex in face {
                    vertex_faces.entry(*vertex).or_default().push(face_index);
                }
            }

            let mut border_vertices = BTreeSet::new();
            for (edge, faces) in &edge_faces {
                if 2 == faces.len() {
                    continue;
                }
                border_vertices.insert(edge.0);
                border_vertices.insert(edge.1);
            }

            let mut touched_vertices = BTreeSet::new();
            let mut removed_faces = BTreeSet::new();
            let mut added_faces: Vec<Vec<usize>> = Vec::new();
            let mut round_collapse_count = 0;
            for (edge, faces) in &edge_faces {
                if 2 != faces.len() {
                    continue;
                }
                let first = edge.0;
                let second = edge.1;
                // Present by construction (edge endpoints are used).
                if 3 != vertex_neighbors[&first].len() || 3 != vertex_neighbors[&second].len() {
                    continue;
                }
                // Present by construction (edge endpoints are used).
                if 3 != vertex_faces[&first].len() || 3 != vertex_faces[&second].len() {
                    continue;
                }

                let mut patch_faces = vertex_faces[&first].clone();
                patch_faces.extend(vertex_faces[&second].iter().copied());
                patch_faces.sort_unstable();
                patch_faces.dedup();
                if PATCH_FACE_NUM != patch_faces.len() {
                    continue;
                }
                let mut usable = true;
                for face_index in &patch_faces {
                    let face = &self.remeshed_polygons[*face_index];
                    if 4 != face.len() || Self::has_repeated_vertex(face) {
                        usable = false;
                        break;
                    }
                    for vertex in face {
                        if border_vertices.contains(vertex) || touched_vertices.contains(vertex) {
                            usable = false;
                            break;
                        }
                    }
                    if !usable {
                        break;
                    }
                }
                if !usable {
                    continue;
                }

                let mut directed_edges = BTreeSet::new();
                for face_index in &patch_faces {
                    let face = &self.remeshed_polygons[*face_index];
                    for i in 0..face.len() {
                        directed_edges.insert((face[i], face[(i + 1) % face.len()]));
                    }
                }
                // `std::map` here (ordered; the convert/merge/split
                // passes use `unordered_map`, but theirs is lookup-only).
                // `buried_counts` is `unordered_map` yet lookup-only, so a
                // `BTreeMap` is exact for both.
                let mut boundary_next = BTreeMap::new();
                let mut buried_counts: BTreeMap<usize, usize> = BTreeMap::new();
                let mut buried_edge_num = 0;
                let mut simple_boundary = true;
                for directed_edge in &directed_edges {
                    let (from, to) = *directed_edge;
                    if directed_edges.contains(&(to, from)) {
                        if from < to {
                            buried_edge_num += 1;
                            *buried_counts.entry(from).or_insert(0) += 1;
                            *buried_counts.entry(to).or_insert(0) += 1;
                        }
                        continue;
                    }
                    // Rust `insert` overwrites on a duplicate key where
                    // C++ `insert` keeps the first, but the map is
                    // discarded on this path either way.
                    if boundary_next.insert(from, to).is_some() {
                        simple_boundary = false;
                        break;
                    }
                }
                if !simple_boundary
                    || HEXAGON_SIZE != boundary_next.len()
                    || PATCH_BURIED_EDGE_NUM != buried_edge_num
                {
                    continue;
                }

                let mut hexagon = Vec::with_capacity(HEXAGON_SIZE);
                // Ordered `std::map`, so `begin()->first` is the smallest
                // boundary key.
                let start = *boundary_next.keys().next().unwrap_or(&usize::MAX);
                let mut walk = start;
                for _ in 0..HEXAGON_SIZE {
                    hexagon.push(walk);
                    match boundary_next.get(&walk) {
                        Some(next) => walk = *next,
                        _ => break,
                    }
                }
                if HEXAGON_SIZE != hexagon.len()
                    || walk != start
                    || Self::has_repeated_vertex(&hexagon)
                {
                    continue;
                }

                let mut old_score = Self::valence_score(vertex_neighbors[&first].len())
                    + Self::valence_score(vertex_neighbors[&second].len());
                for corner in &hexagon {
                    old_score += Self::valence_score(vertex_neighbors[corner].len());
                }
                let mut old_normal = Vector3::default();
                for face_index in &patch_faces {
                    old_normal += Self::face_normal_of(
                        &self.remeshed_vertices,
                        &self.remeshed_polygons[*face_index],
                    );
                }

                let mut best_score = 0;
                let mut best_flow = 0.0;
                let mut best_quads: Vec<Vec<usize>> = Vec::new();
                for i in 0..HEXAGON_SIZE / 2 {
                    let across = i + HEXAGON_SIZE / 2;
                    let corner = hexagon[i];
                    let opposite = hexagon[across];
                    // Present by construction (hexagon corners are used).
                    if vertex_neighbors[&corner].contains(&opposite) {
                        continue;
                    }

                    let mut new_score = 0;
                    let mut lopsided = false;
                    for (j, hexagon_corner) in hexagon.iter().enumerate() {
                        // Present by construction (hexagon corners are
                        // used); buried edges are distinct mesh edges, so
                        // the count never exceeds the valence.
                        let valence = vertex_neighbors[hexagon_corner].len()
                            - buried_counts.get(hexagon_corner).copied().unwrap_or(0)
                            + usize::from(i == j || across == j);
                        if valence < MIN_VALENCE {
                            lopsided = true;
                        }
                        new_score += Self::valence_score(valence);
                        if lopsided {
                            break;
                        }
                    }
                    if lopsided || new_score > old_score {
                        continue;
                    }

                    let quads = vec![
                        vec![
                            corner,
                            hexagon[(i + 1) % HEXAGON_SIZE],
                            hexagon[(i + 2) % HEXAGON_SIZE],
                            opposite,
                        ],
                        vec![
                            opposite,
                            hexagon[(across + 1) % HEXAGON_SIZE],
                            hexagon[(across + 2) % HEXAGON_SIZE],
                            corner,
                        ],
                    ];
                    let mut folded = false;
                    for quad in &quads {
                        if Vector3::dot_product(
                            &old_normal,
                            &Self::face_normal_of(&self.remeshed_vertices, quad),
                        ) <= 0.0
                        {
                            folded = true;
                            break;
                        }
                    }
                    if folded {
                        continue;
                    }

                    let span = self.remeshed_vertices[opposite] - self.remeshed_vertices[corner];
                    let direction = span.normalized();
                    let corner_skip = BTreeSet::from([
                        hexagon[(i + 1) % HEXAGON_SIZE],
                        hexagon[(i + HEXAGON_SIZE - 1) % HEXAGON_SIZE],
                        first,
                        second,
                    ]);
                    let opposite_skip = BTreeSet::from([
                        hexagon[(across + 1) % HEXAGON_SIZE],
                        hexagon[(across + HEXAGON_SIZE - 1) % HEXAGON_SIZE],
                        first,
                        second,
                    ]);
                    let mut new_flow = Self::flow_score_at(
                        &self.remeshed_vertices,
                        &vertex_neighbors,
                        corner,
                        direction,
                        &corner_skip,
                    ) + Self::flow_score_at(
                        &self.remeshed_vertices,
                        &vertex_neighbors,
                        opposite,
                        -direction,
                        &opposite_skip,
                    );
                    for quad in &quads {
                        new_flow += Self::corner_score_of(&self.remeshed_vertices, quad);
                    }
                    if !best_quads.is_empty()
                        && (new_score > best_score
                            || (new_score == best_score && new_flow <= best_flow))
                    {
                        continue;
                    }

                    best_score = new_score;
                    best_flow = new_flow;
                    best_quads = quads;
                }
                if best_quads.is_empty() {
                    continue;
                }

                for face_index in &patch_faces {
                    removed_faces.insert(*face_index);
                    for vertex in &self.remeshed_polygons[*face_index] {
                        touched_vertices.insert(*vertex);
                    }
                }
                for quad in best_quads {
                    added_faces.push(quad);
                }
                collapsed_vertices.extend(hexagon.iter().copied());
                round_collapse_count += 1;
            }

            if 0 == round_collapse_count {
                break;
            }

            let mut rewritten =
                Vec::with_capacity(self.remeshed_polygons.len() + added_faces.len());
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                if removed_faces.contains(&face_index) {
                    continue;
                }
                rewritten.push(face.clone());
            }
            for face in added_faces {
                rewritten.push(face);
            }
            self.remeshed_polygons = rewritten;
            collapse_count += round_collapse_count;
        }

        if 0 == collapse_count {
            return;
        }

        let compacted = self.compact_vertices(&collapsed_vertices);

        self.diagnose(|| format!("Collapse three valence edge pairs:{collapse_count}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&compacted, 3, 5);
    }

    /// Corner index where the directed edge `from -> to` enters `face`
    /// (`face.len()` on a miss): the edge switch's `findDirectedEdge`.
    fn find_directed_entry(face: &[usize], from: usize, to: usize) -> usize {
        for i in 0..face.len() {
            if from == face[i] && to == face[(i + 1) % face.len()] {
                return i;
            }
        }
        face.len()
    }

    fn switch_high_valence_edges(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        // Two quads sharing an edge make a hexagon with three diagonals,
        // the shared edge being one of them. Switching to another
        // diagonal takes one neighbor away from each end of the shared
        // edge and hands one to each end of the new diagonal, no point is
        // added, removed or moved
        let mut switch_num = 0;
        let mut switched_vertices = BTreeSet::new();
        loop {
            let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    edge_faces
                        .entry(Self::edge_of(face[i], face[j]))
                        .or_default()
                        .push(face_index);
                    vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                    vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
                }
            }

            // Four neighbors is the wrong target on a border, leave those
            // alone
            let mut border_vertices = BTreeSet::new();
            for (edge, faces) in &edge_faces {
                if 2 == faces.len() {
                    continue;
                }
                border_vertices.insert(edge.0);
                border_vertices.insert(edge.1);
            }

            let mut existing_faces = BTreeSet::new();
            for face in &self.remeshed_polygons {
                existing_faces.insert(Self::canonical_face(face));
            }

            // A second switch reaching into the same hexagon would work
            // off the face indices and the valences the first one left
            // behind
            let mut touched_vertices = BTreeSet::new();
            let mut round_switch_num = 0;
            for (key, value) in &edge_faces {
                if 2 != value.len() {
                    continue;
                }

                let a = key.0;
                let b = key.1;
                // Present by construction (edge endpoints are used).
                let a_valence = vertex_neighbors[&a].len();
                let b_valence = vertex_neighbors[&b].len();
                if a_valence <= 4 || b_valence <= 4 {
                    continue;
                }
                if border_vertices.contains(&a) || border_vertices.contains(&b) {
                    continue;
                }

                let mut first = value[0];
                let mut second = value[1];
                if 4 != self.remeshed_polygons[first].len()
                    || 4 != self.remeshed_polygons[second].len()
                {
                    continue;
                }
                let mut first_entry =
                    Self::find_directed_entry(&self.remeshed_polygons[first], a, b);
                if first_entry >= self.remeshed_polygons[first].len() {
                    std::mem::swap(&mut first, &mut second);
                    first_entry = Self::find_directed_entry(&self.remeshed_polygons[first], a, b);
                }
                let second_entry = Self::find_directed_entry(&self.remeshed_polygons[second], b, a);
                if first_entry >= self.remeshed_polygons[first].len()
                    || second_entry >= self.remeshed_polygons[second].len()
                {
                    continue;
                }

                // The hexagon runs b, c, d, a, e, f, keeping the winding
                // of both quads
                let first_face = &self.remeshed_polygons[first];
                let second_face = &self.remeshed_polygons[second];
                let c = first_face[(first_entry + 2) % 4];
                let d = first_face[(first_entry + 3) % 4];
                let e = second_face[(second_entry + 2) % 4];
                let f = second_face[(second_entry + 3) % 4];
                let hexagon = BTreeSet::from([a, b, c, d, e, f]);
                if 6 != hexagon.len() {
                    continue;
                }
                let mut touched = false;
                for vertex in &hexagon {
                    if touched_vertices.contains(vertex) {
                        touched = true;
                        break;
                    }
                }
                if touched {
                    continue;
                }

                let old_normal = Self::face_normal_of(&self.remeshed_vertices, first_face)
                    + Self::face_normal_of(&self.remeshed_vertices, second_face);
                let old_valence_score =
                    Self::valence_score(a_valence) + Self::valence_score(b_valence);

                let mut best_gain = 0;
                let mut best_shape = 0.0;
                let mut best_face1 = Vec::new();
                let mut best_face2 = Vec::new();
                for candidate in 0..2 {
                    let x = if 0 == candidate { c } else { d };
                    let y = if 0 == candidate { e } else { f };
                    if border_vertices.contains(&x) || border_vertices.contains(&y) {
                        continue;
                    }
                    // The new diagonal would land on an edge which is
                    // already there
                    // Present by construction (hexagon corners are used).
                    if vertex_neighbors[&x].contains(&y) {
                        continue;
                    }
                    let x_valence = vertex_neighbors[&x].len();
                    let y_valence = vertex_neighbors[&y].len();
                    let gain = old_valence_score
                        + Self::valence_score(x_valence)
                        + Self::valence_score(y_valence)
                        - (Self::valence_score(a_valence - 1)
                            + Self::valence_score(b_valence - 1)
                            + Self::valence_score(x_valence + 1)
                            + Self::valence_score(y_valence + 1));
                    if gain <= 0 {
                        continue;
                    }
                    let face1 = if 0 == candidate {
                        vec![c, d, a, e]
                    } else {
                        vec![d, a, e, f]
                    };
                    let face2 = if 0 == candidate {
                        vec![e, f, b, c]
                    } else {
                        vec![f, b, c, d]
                    };
                    if Vector3::dot_product(
                        &old_normal,
                        &Self::face_normal_of(&self.remeshed_vertices, &face1),
                    ) <= 0.0
                        || Vector3::dot_product(
                            &old_normal,
                            &Self::face_normal_of(&self.remeshed_vertices, &face2),
                        ) <= 0.0
                    {
                        continue;
                    }
                    if existing_faces.contains(&Self::canonical_face(&face1))
                        || existing_faces.contains(&Self::canonical_face(&face2))
                    {
                        continue;
                    }
                    let shape = Self::corner_score_of(&self.remeshed_vertices, &face1)
                        + Self::corner_score_of(&self.remeshed_vertices, &face2);
                    if gain < best_gain || (gain == best_gain && shape <= best_shape) {
                        continue;
                    }
                    best_gain = gain;
                    best_shape = shape;
                    best_face1 = face1;
                    best_face2 = face2;
                }
                if best_face1.is_empty() {
                    continue;
                }

                existing_faces.remove(&Self::canonical_face(&self.remeshed_polygons[first]));
                existing_faces.remove(&Self::canonical_face(&self.remeshed_polygons[second]));
                existing_faces.insert(Self::canonical_face(&best_face1));
                existing_faces.insert(Self::canonical_face(&best_face2));
                self.remeshed_polygons[first] = best_face1;
                self.remeshed_polygons[second] = best_face2;
                touched_vertices.extend(hexagon.iter().copied());
                switched_vertices.extend(hexagon.iter().copied());
                round_switch_num += 1;
            }

            if 0 == round_switch_num {
                break;
            }
            switch_num += round_switch_num;
        }

        if 0 == switch_num {
            return;
        }

        self.diagnose(|| format!("Switch high valence edges:{switch_num}\n"));
        self.rebuild_half_edges();

        // The switched quads kept their points, pull the reconnected
        // patches back into shape
        self.smooth_around_vertices(&switched_vertices, 3, 5);
    }

    /// Newell's-method fan normal over raw positions (the cleanup pass
    /// compares before/after positions, not mesh faces).
    fn polygon_normal(positions: &[Vector3]) -> Vector3 {
        let mut normal = Vector3::default();
        if positions.len() < 3 {
            return normal;
        }
        for i in 1..positions.len() - 1 {
            normal += Vector3::cross_product(
                &(positions[i] - positions[0]),
                &(positions[i + 1] - positions[0]),
            );
        }
        normal
    }

    /// The cleanup pass's `walkRoute`: the ladder of rungs from
    /// `start_face` across `start_edge` to the nearest sink, or `None`
    /// when the strip folds back on itself or runs too long.
    ///
    /// Returns the rungs, the dissolved faces, and the sink face
    /// (`usize::MAX` for a border sink). The C++ fills out-params and
    /// reports success separately; the caller discards partial outs on
    /// failure, so one `Option` carries the same outcome.
    fn walk_cleanup_route(
        polygons: &[Vec<usize>],
        edge_faces: &EdgeFaceIndex,
        start_face: usize,
        start_edge: (usize, usize),
    ) -> Option<CleanupRoute> {
        const MAX_ROUTE_LENGTH: usize = 20;
        const NO_FACE: usize = usize::MAX;

        let mut rungs = Vec::new();
        let mut dissolved_faces = BTreeSet::new();
        dissolved_faces.insert(start_face);
        let mut sink_face = NO_FACE;
        let mut rung_vertices = BTreeSet::new();
        let mut current_face = start_face;
        let mut rung = start_edge;
        loop {
            if rungs.len() >= MAX_ROUTE_LENGTH {
                return None;
            }
            // Two rungs sharing a vertex would collapse into each
            // other
            if !rung_vertices.insert(rung.0) {
                return None;
            }
            if !rung_vertices.insert(rung.1) {
                return None;
            }
            rungs.push(rung);
            // Every rung is a real mesh edge, so the entry exists; a
            // missing one reads as a non-manifold stop either way.
            let incident = match edge_faces.get(&rung) {
                Some(incident) => incident,
                _ => return None,
            };
            if 1 == incident.len() {
                return Some((rungs, dissolved_faces, sink_face));
            }
            if 2 != incident.len() {
                return None;
            }
            let neighbor = if incident[0] == current_face {
                incident[1]
            } else {
                incident[0]
            };
            if dissolved_faces.contains(&neighbor) {
                return None;
            }
            let neighbor_face = &polygons[neighbor];
            if 4 != neighbor_face.len() {
                sink_face = neighbor;
                return Some((rungs, dissolved_faces, sink_face));
            }
            let mut entry = neighbor_face.len();
            for i in 0..neighbor_face.len() {
                if Self::edge_of(
                    neighbor_face[i],
                    neighbor_face[(i + 1) % neighbor_face.len()],
                ) == rung
                {
                    entry = i;
                    break;
                }
            }
            if entry >= neighbor_face.len() {
                return None;
            }
            dissolved_faces.insert(neighbor);
            current_face = neighbor;
            rung = Self::edge_of(
                neighbor_face[(entry + 2) % 4],
                neighbor_face[(entry + 3) % 4],
            );
        }
    }

    fn cleanup_triangles(&mut self) {
        self.cleanup_routes(false);
    }

    /// Final sweep for defects the pair-wise passes leave behind: isolated
    /// pentagons (and triangles with fresh routes after the later passes)
    /// collapse through a quad strip into a sink, exactly like the
    /// mid-pipeline triangle cleanup, except a pentagon start shrinks to
    /// a quad instead of dissolving. Deliberately emits no progress
    /// events: the engine oracle pins the progress sequence exactly.
    fn cleanup_residual_routes(&mut self) {
        self.cleanup_routes(true);
    }

    fn cleanup_routes(&mut self, pentagon_starts: bool) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const NO_FACE: usize = usize::MAX;

        // The route of a triangle is the ladder of edges crossing the
        // quad strip that leads to the nearest sink, a border or another
        // non quad face. Every rung of the ladder collapses in the same
        // step: the strip closes up, the sink loses one side, and the
        // valence of the vertices along the way is left untouched
        // because each pair of side edges merges together with its rung.
        // Collapsing one rung at a time instead lets the surviving
        // vertex take part in the next collapse as well, which grows a
        // fan of slivers around a single point. A pentagon start works
        // the same way, except the start face gives up one side (5 -> 4)
        // instead of dissolving.
        let mut rejected_edges = BTreeSet::new();
        let mut collapsed_vertices = BTreeSet::new();
        let mut collapse_count = 0;
        let mut pentagon_count = 0;
        loop {
            // Sorted-vector rebuild (identical keys/values/order to the
            // `BTreeMap` build, a handful of allocs instead of 100k+).
            let edge_faces = EdgeFaceIndex::build(&self.remeshed_polygons);

            let mut route: Vec<(usize, usize)> = Vec::new();
            let mut route_faces = BTreeSet::new();
            let mut route_sink = NO_FACE;
            let mut route_start = NO_FACE;
            let mut route_start_len = 0;
            for start_face in 0..self.remeshed_polygons.len() {
                let starter = &self.remeshed_polygons[start_face];
                let starter_len = starter.len();
                if (3 != starter_len && !(pentagon_starts && 5 == starter_len))
                    || Self::has_repeated_vertex(starter)
                {
                    continue;
                }
                for i in 0..starter_len {
                    let start_edge = Self::edge_of(starter[i], starter[(i + 1) % starter_len]);
                    if rejected_edges.contains(&start_edge) {
                        continue;
                    }
                    let (candidate_route, candidate_faces, candidate_sink) =
                        match Self::walk_cleanup_route(
                            &self.remeshed_polygons,
                            &edge_faces,
                            start_face,
                            start_edge,
                        ) {
                            Some(found) => found,
                            _ => continue,
                        };
                    // The shorter the ladder, the less of the
                    // surrounding mesh it takes with it
                    if route.is_empty() || candidate_route.len() < route.len() {
                        route = candidate_route;
                        route_faces = candidate_faces;
                        route_sink = candidate_sink;
                        route_start = start_face;
                        route_start_len = starter_len;
                    }
                }
            }
            if route.is_empty() {
                break;
            }

            let mut merged_into: BTreeMap<usize, usize> = BTreeMap::new();
            let mut merged_positions: BTreeMap<usize, Vector3> = BTreeMap::new();
            for rung in &route {
                let (first, second) = *rung;
                // First wins (C++ `insert`); rungs are vertex-disjoint
                // (the walk rejects shared vertices), so the keys are
                // unique either way.
                merged_into.entry(second).or_insert(first);
                merged_positions.entry(first).or_insert(
                    (self.remeshed_vertices[first] + self.remeshed_vertices[second]) * 0.5,
                );
            }

            // Per-face remap is independent given the fixed route maps;
            // validation keeps its serial face-order scan (same checks,
            // same order, same early exit), so the outcome is identical.
            let mut remapped: Vec<(Vec<usize>, bool, bool)> =
                vec![(Vec::new(), false, false); self.remeshed_polygons.len()];
            parallel_each(&mut remapped, |face_index, slot| {
                let face = &self.remeshed_polygons[face_index];
                let mut candidate = Vec::with_capacity(face.len());
                let mut touched = false;
                for vertex in face {
                    let rewritten_vertex = merged_into.get(vertex).copied().unwrap_or(*vertex);
                    if rewritten_vertex != *vertex || merged_positions.contains_key(vertex) {
                        touched = true;
                    }
                    if candidate.is_empty() || candidate[candidate.len() - 1] != rewritten_vertex {
                        candidate.push(rewritten_vertex);
                    }
                }
                if candidate.len() > 1 && candidate[0] == candidate[candidate.len() - 1] {
                    candidate.pop();
                }
                // The strip faces and a triangle sink are meant to
                // disappear, everything else has to come out of the
                // collapse with the shape it went in with, apart from
                // the sink which gives up exactly one side. A pentagon
                // start is the exception to the strip rule: it gives up
                // one side instead of dissolving.
                let start_dissolves = 3 == route_start_len;
                let dissolving = (route_faces.contains(&face_index)
                    && (face_index != route_start || start_dissolves))
                    || (face_index == route_sink && 3 == face.len());
                *slot = (candidate, touched, dissolving);
            });
            let mut rewritten: Vec<Vec<usize>> = Vec::with_capacity(self.remeshed_polygons.len());
            let mut touched_faces = BTreeSet::new();
            let mut touched_edge_counts: BTreeMap<(usize, usize), usize> = BTreeMap::new();
            let mut valid = true;
            for (face_index, (candidate, touched, dissolving)) in remapped.into_iter().enumerate() {
                let face = &self.remeshed_polygons[face_index];
                if dissolving {
                    if candidate.len() >= 3 {
                        valid = false;
                        break;
                    }
                    continue;
                }
                let expected_size = if face_index == route_start && 5 == route_start_len {
                    route_start_len - 1
                } else if face_index == route_sink {
                    face.len() - 1
                } else {
                    face.len()
                };
                if candidate.len() != expected_size || Self::has_repeated_vertex(&candidate) {
                    valid = false;
                    break;
                }
                if touched {
                    let mut before = Vec::with_capacity(face.len());
                    for vertex in face {
                        before.push(self.remeshed_vertices[*vertex]);
                    }
                    let mut after = Vec::with_capacity(candidate.len());
                    for vertex in &candidate {
                        after.push(
                            merged_positions
                                .get(vertex)
                                .copied()
                                .unwrap_or(self.remeshed_vertices[*vertex]),
                        );
                    }
                    if Vector3::dot_product(
                        &Self::polygon_normal(&before),
                        &Self::polygon_normal(&after),
                    ) <= 0.0
                    {
                        valid = false;
                        break;
                    }
                    // A face that duplicates another one must share
                    // every vertex with it, so both of them are among
                    // the faces touched by the collapse
                    if !touched_faces.insert(Self::canonical_face(&candidate)) {
                        valid = false;
                        break;
                    }
                    for i in 0..candidate.len() {
                        let edge =
                            Self::edge_of(candidate[i], candidate[(i + 1) % candidate.len()]);
                        if !merged_positions.contains_key(&edge.0)
                            && !merged_positions.contains_key(&edge.1)
                        {
                            continue;
                        }
                        // Present-or-zero (C++ `operator[]` on a count
                        // map); the entry exists exactly when an
                        // earlier face touched the same edge.
                        let count = touched_edge_counts.entry(edge).or_insert(0);
                        *count += 1;
                        if *count > 2 {
                            valid = false;
                            break;
                        }
                    }
                    if !valid {
                        break;
                    }
                }
                rewritten.push(candidate);
            }
            if !valid {
                rejected_edges.insert(route[0]);
                continue;
            }

            for (vertex, position) in &merged_positions {
                self.remeshed_vertices[*vertex] = *position;
                collapsed_vertices.insert(*vertex);
            }
            self.remeshed_polygons = rewritten;
            collapse_count += 1;
            if 5 == route_start_len {
                pentagon_count += 1;
            }
        }

        if 0 == collapse_count {
            return;
        }

        let compacted = self.compact_vertices(&collapsed_vertices);

        if pentagon_starts {
            self.diagnose(|| {
                format!("Cleanup residual routes:{collapse_count} (pentagons:{pentagon_count})\n")
            });
        } else {
            self.diagnose(|| format!("Cleanup triangle faces:{collapse_count}\n"));
        }
        self.rebuild_half_edges();

        // The rungs met halfway, pull the closed up strips back onto the
        // source mesh
        self.smooth_around_vertices(&compacted, 3, 5);
    }

    fn smooth_around_vertices(
        &mut self,
        seed_vertices: &BTreeSet<usize>,
        rings: usize,
        iterations: usize,
    ) {
        if seed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        let mut vertex_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
            for vertex in face {
                vertex_faces.entry(*vertex).or_default().push(face_index);
            }
        }

        // Rings of faces grown from the seed points, the vertices
        // sitting on the outer border of the patch anchor the smoothing
        let mut patch_faces = BTreeSet::new();
        let mut frontier = BTreeSet::new();
        for vertex in seed_vertices {
            if !vertex_faces.contains_key(vertex) {
                continue;
            }
            frontier.insert(*vertex);
        }
        // Set contents only: every iteration unions whole faces in, so
        // sorted iteration builds the same patch/border/movable sets as
        // the C++ hash order.
        for _ in 0..rings {
            if frontier.is_empty() {
                break;
            }
            let mut next_frontier = BTreeSet::new();
            for vertex in &frontier {
                // Present by construction (frontier vertices are kept
                // only when they have faces).
                for face_index in &vertex_faces[vertex] {
                    if !patch_faces.insert(*face_index) {
                        continue;
                    }
                    for neighbor_vertex in &self.remeshed_polygons[*face_index] {
                        next_frontier.insert(*neighbor_vertex);
                    }
                }
            }
            frontier = next_frontier;
        }

        let mut movable_vertices = BTreeSet::new();
        for face_index in &patch_faces {
            for vertex in &self.remeshed_polygons[*face_index] {
                movable_vertices.insert(*vertex);
            }
        }
        // Restructure: the C++ erases in place while iterating; the
        // mirror collects the border vertices first (same surviving
        // set), because Rust cannot erase during iteration.
        let mut anchored = Vec::new();
        for vertex in &movable_vertices {
            // Present by construction (movable vertices come from mesh
            // faces).
            let mut on_patch_border = false;
            for face_index in &vertex_faces[vertex] {
                if !patch_faces.contains(face_index) {
                    on_patch_border = true;
                    break;
                }
            }
            if on_patch_border {
                anchored.push(*vertex);
            }
        }
        for vertex in anchored {
            movable_vertices.remove(&vertex);
        }

        if movable_vertices.is_empty() {
            return;
        }

        self.smooth_and_project(iterations, Some(&movable_vertices));
    }

    fn merge_shared_five_edge_faces(&mut self, progress: Option<&ProgressHandler>) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        // Every merge consumes a pair of pentagons, so half the pentagon
        // count is a real ceiling on how many rounds can succeed and
        // makes an honest denominator for the fraction reported below.
        let mut pentagon_count = 0;
        for face in &self.remeshed_polygons {
            if 5 == face.len() {
                pentagon_count += 1;
            }
        }
        let merge_ceiling = (pentagon_count / 2).max(1);

        // The merged point inherits the neighbors of both ends of the
        // collapsed edge. The pair of pentagons is worth trading for a
        // six valence point, past that the singularity left behind is a
        // worse defect than the faces it replaces
        const MAX_MERGED_VALENCE: usize = 6;

        let mut rejected_edges = BTreeSet::new();
        let mut merged_vertices = BTreeSet::new();
        let mut merge_count = 0;
        loop {
            if let Some(handler) = progress {
                handler(
                    f32::min(0.99, merge_count as f32 / merge_ceiling as f32),
                    "Merging shared five edge faces",
                );
            }
            // Sorted-vector rebuild (identical keys/values/order to the
            // `BTreeMap` build, a handful of allocs instead of 150k+).
            let edge_faces = EdgeFaceIndex::build(&self.remeshed_polygons);
            let vertex_neighbors = NeighborIndex::build(&self.remeshed_polygons);

            let mut shared_edge = (0, 0);
            let mut found_shared = false;
            for (edge, faces) in edge_faces.groups() {
                if 2 != faces.len() {
                    continue;
                }
                if rejected_edges.contains(&edge) {
                    continue;
                }
                let mut both_five_edges = true;
                for face_index in faces {
                    let face = &self.remeshed_polygons[*face_index];
                    if 5 != face.len() || Self::has_repeated_vertex(face) {
                        both_five_edges = false;
                        break;
                    }
                }
                if !both_five_edges {
                    continue;
                }
                // Endpoints are present by construction (used vertices).
                let mut neighbors = BTreeSet::new();
                neighbors.extend(vertex_neighbors.get_or_empty(&edge.0).iter().copied());
                neighbors.extend(vertex_neighbors.get_or_empty(&edge.1).iter().copied());
                neighbors.remove(&edge.0);
                neighbors.remove(&edge.1);
                if neighbors.len() > MAX_MERGED_VALENCE {
                    continue;
                }
                shared_edge = edge;
                found_shared = true;
                break;
            }
            if !found_shared {
                break;
            }

            let keep = shared_edge.0;
            let remove = shared_edge.1;
            let unmoved_vertex = self.remeshed_vertices.len();
            let keep_position =
                (self.remeshed_vertices[keep] + self.remeshed_vertices[remove]) * 0.5;
            // Per-face remap is independent given the fixed (keep,
            // remove) pair; validation keeps its serial face-order scan
            // (same checks, same order, same early exit), so the outcome
            // is identical.
            let mut remapped: Vec<(Vec<usize>, bool)> =
                vec![(Vec::new(), false); self.remeshed_polygons.len()];
            parallel_each(&mut remapped, |face_index, slot| {
                let face = &self.remeshed_polygons[face_index];
                let mut face_affected = false;
                for vertex in face {
                    if keep == *vertex || remove == *vertex {
                        face_affected = true;
                        break;
                    }
                }
                let mut candidate = Vec::with_capacity(face.len());
                for vertex in face {
                    let rewritten_vertex = if *vertex == remove { keep } else { *vertex };
                    if candidate.is_empty() || candidate[candidate.len() - 1] != rewritten_vertex {
                        candidate.push(rewritten_vertex);
                    }
                }
                if candidate.len() > 1 && candidate[0] == candidate[candidate.len() - 1] {
                    candidate.pop();
                }
                *slot = (candidate, face_affected);
            });
            let mut rewritten: Vec<Vec<usize>> = Vec::with_capacity(self.remeshed_polygons.len());
            let mut affected = Vec::with_capacity(self.remeshed_polygons.len());
            let mut valid = true;
            for (face, (candidate, face_affected)) in self.remeshed_polygons.iter().zip(remapped) {
                if face_affected {
                    // The two five edge faces become quads, no other face
                    // is allowed to degrade, otherwise the merge is
                    // trading one defect for another
                    if candidate.len() < 3 || (face.len() >= 4 && candidate.len() < 4) {
                        valid = false;
                        break;
                    }
                    if Self::has_repeated_vertex(&candidate) {
                        valid = false;
                        break;
                    }
                    let old_normal = Self::face_normal_with_added(
                        &self.remeshed_vertices,
                        face,
                        unmoved_vertex,
                        Vector3::default(),
                    );
                    let new_normal = Self::face_normal_with_added(
                        &self.remeshed_vertices,
                        &candidate,
                        keep,
                        keep_position,
                    );
                    if Vector3::dot_product(&old_normal, &new_normal) <= 0.0 {
                        valid = false;
                        break;
                    }
                }
                rewritten.push(candidate);
                affected.push(face_affected);
            }

            if valid {
                let mut unique_faces = BTreeSet::new();
                for (face_index, face) in rewritten.iter().enumerate() {
                    if !affected[face_index] {
                        unique_faces.insert(Self::canonical_face(face));
                    }
                }
                let mut keep_edge_counts: BTreeMap<(usize, usize), usize> = BTreeMap::new();
                for (face_index, face) in rewritten.iter().enumerate() {
                    if !valid {
                        break;
                    }
                    if affected[face_index] && !unique_faces.insert(Self::canonical_face(face)) {
                        valid = false;
                        break;
                    }
                    for i in 0..face.len() {
                        let edge = Self::edge_of(face[i], face[(i + 1) % face.len()]);
                        if keep != edge.0 && keep != edge.1 {
                            continue;
                        }
                        // Present-or-zero (C++ `operator[]` on a count
                        // map); the entry exists exactly when an earlier
                        // face touched the same edge.
                        let count = keep_edge_counts.entry(edge).or_insert(0);
                        *count += 1;
                        if *count > 2 {
                            valid = false;
                            break;
                        }
                    }
                }
            }
            if !valid {
                rejected_edges.insert(shared_edge);
                continue;
            }

            self.remeshed_vertices[keep] = keep_position;
            self.remeshed_polygons = rewritten;
            merged_vertices.insert(keep);
            merge_count += 1;
        }

        if 0 == merge_count {
            return;
        }

        let compacted = self.compact_vertices(&merged_vertices);

        // Gate on the parked handler: the caller parks
        // `self.progress_handler` in an `Arc` while this runs (borrow
        // restructure), so `diagnose()` would stay silent here —
        // `progress` is `Some` exactly when the outer handler exists
        // (the C++ `m_progressHandler` gate), with
        // `self.progress_handler` kept as the direct-call fallback.
        if progress.is_some() || self.progress_handler.is_some() {
            eprint!("Merge shared five edge faces:{merge_count}\n");
        }
        self.rebuild_half_edges();

        // The two pentagons closed up around the merged point, pull the
        // patch back onto the source mesh
        self.smooth_around_vertices(&compacted, 3, 5);
    }

    fn collapse_three_valence_corners(&mut self) {
        if self.remeshed_vertices.is_empty() || self.remeshed_polygons.is_empty() {
            return;
        }

        const MIN_SIDE_VALENCE: usize = 5;

        let mut collapse_count = 0;
        let mut collapsed_vertices = BTreeSet::new();
        loop {
            let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
            let mut vertex_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    edge_faces
                        .entry(Self::edge_of(face[i], face[j]))
                        .or_default()
                        .push(face_index);
                    vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                    vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
                }
                for vertex in face {
                    vertex_faces.entry(*vertex).or_default().push(face_index);
                }
            }

            let mut border_vertices = BTreeSet::new();
            for (edge, faces) in &edge_faces {
                if 2 == faces.len() {
                    continue;
                }
                border_vertices.insert(edge.0);
                border_vertices.insert(edge.1);
            }

            let mut touched_vertices = BTreeSet::new();
            let mut replaced_faces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            let mut removed_faces = BTreeSet::new();
            let mut round_collapse_count = 0;
            for face_index in 0..self.remeshed_polygons.len() {
                let quad = &self.remeshed_polygons[face_index];
                if 4 != quad.len() || Self::has_repeated_vertex(quad) {
                    continue;
                }
                let mut on_border = false;
                for vertex in quad {
                    if border_vertices.contains(vertex) {
                        on_border = true;
                        break;
                    }
                }
                if on_border {
                    continue;
                }
                for i in 0..4 {
                    let corner = quad[i];
                    let left_side = quad[(i + 1) % 4];
                    let opposite = quad[(i + 2) % 4];
                    let right_side = quad[(i + 3) % 4];
                    // Present by construction (quad corners are used).
                    if 3 != vertex_neighbors[&corner].len() {
                        continue;
                    }
                    // Present by construction (quad corners are used).
                    if 3 != vertex_faces[&corner].len() {
                        continue;
                    }
                    // Present by construction (quad corners are used).
                    if vertex_neighbors[&left_side].len() < MIN_SIDE_VALENCE
                        || vertex_neighbors[&right_side].len() < MIN_SIDE_VALENCE
                    {
                        continue;
                    }
                    if vertex_neighbors[&corner].contains(&opposite) {
                        continue;
                    }
                    let mut shared_num = 0;
                    for neighbor in &vertex_neighbors[&corner] {
                        if vertex_neighbors[&opposite].contains(neighbor) {
                            shared_num += 1;
                        }
                    }
                    if 2 != shared_num {
                        continue;
                    }

                    let mut affected_faces = vertex_faces[&corner].clone();
                    affected_faces.extend(vertex_faces[&opposite].iter().copied());
                    affected_faces.sort_unstable();
                    affected_faces.dedup();
                    let mut touched = false;
                    for affected in &affected_faces {
                        for vertex in &self.remeshed_polygons[*affected] {
                            if touched_vertices.contains(vertex) {
                                touched = true;
                                break;
                            }
                        }
                        if touched {
                            break;
                        }
                    }
                    if touched {
                        continue;
                    }

                    let added_vertex = self.remeshed_vertices.len();
                    let added_position =
                        (self.remeshed_vertices[corner] + self.remeshed_vertices[opposite]) * 0.5;

                    let mut rewritten = Vec::with_capacity(affected_faces.len());
                    let mut valid = true;
                    for affected in &affected_faces {
                        if face_index == *affected {
                            continue;
                        }
                        let face = &self.remeshed_polygons[*affected];
                        let mut candidate = Vec::with_capacity(face.len());
                        for vertex in face {
                            let rewritten_vertex = if corner == *vertex || opposite == *vertex {
                                added_vertex
                            } else {
                                *vertex
                            };
                            if candidate.is_empty()
                                || candidate[candidate.len() - 1] != rewritten_vertex
                            {
                                candidate.push(rewritten_vertex);
                            }
                        }
                        if candidate.len() > 1 && candidate[0] == candidate[candidate.len() - 1] {
                            candidate.pop();
                        }
                        if candidate.len() != face.len() || Self::has_repeated_vertex(&candidate) {
                            valid = false;
                            break;
                        }
                        if Vector3::dot_product(
                            &Self::face_normal_with_added(
                                &self.remeshed_vertices,
                                face,
                                added_vertex,
                                added_position,
                            ),
                            &Self::face_normal_with_added(
                                &self.remeshed_vertices,
                                &candidate,
                                added_vertex,
                                added_position,
                            ),
                        ) <= 0.0
                        {
                            valid = false;
                            break;
                        }
                        rewritten.push(candidate);
                    }
                    if valid {
                        let mut unique_faces = BTreeSet::new();
                        let mut added_edge_counts: BTreeMap<(usize, usize), usize> =
                            BTreeMap::new();
                        for face in &rewritten {
                            if !unique_faces.insert(Self::canonical_face(face)) {
                                valid = false;
                                break;
                            }
                            for j in 0..face.len() {
                                if !valid {
                                    break;
                                }
                                let edge = Self::edge_of(face[j], face[(j + 1) % face.len()]);
                                if added_vertex != edge.0 && added_vertex != edge.1 {
                                    continue;
                                }
                                // Present-or-zero (C++ `operator[]` on a
                                // count map).
                                let count = added_edge_counts.entry(edge).or_insert(0);
                                *count += 1;
                                if *count > 2 {
                                    valid = false;
                                }
                            }
                            if !valid {
                                break;
                            }
                        }
                    }
                    if !valid {
                        continue;
                    }

                    self.remeshed_vertices.push(added_position);
                    removed_faces.insert(face_index);
                    let mut rewritten_index = 0;
                    for affected in &affected_faces {
                        if face_index == *affected {
                            continue;
                        }
                        for vertex in &self.remeshed_polygons[*affected] {
                            touched_vertices.insert(*vertex);
                        }
                        // First wins (C++ `insert`); affected faces are
                        // distinct within one adoption and the touched
                        // guard blocks overlaps across adoptions, so this
                        // never overwrites either way.
                        replaced_faces
                            .entry(*affected)
                            .or_insert_with(|| rewritten[rewritten_index].clone());
                        rewritten_index += 1;
                    }
                    for vertex in quad {
                        touched_vertices.insert(*vertex);
                    }
                    collapsed_vertices.insert(added_vertex);
                    collapsed_vertices.insert(left_side);
                    collapsed_vertices.insert(right_side);
                    round_collapse_count += 1;
                    break;
                }
            }

            if 0 == round_collapse_count {
                break;
            }

            let mut rewritten = Vec::with_capacity(self.remeshed_polygons.len());
            for (face_index, face) in self.remeshed_polygons.iter().enumerate() {
                if removed_faces.contains(&face_index) {
                    continue;
                }
                if let Some(find_rewritten) = replaced_faces.get(&face_index) {
                    rewritten.push(find_rewritten.clone());
                    continue;
                }
                rewritten.push(face.clone());
            }
            self.remeshed_polygons = rewritten;
            collapse_count += round_collapse_count;
        }

        if 0 == collapse_count {
            return;
        }

        let compacted = self.compact_vertices(&collapsed_vertices);

        self.diagnose(|| format!("Collapse three valence corners:{collapse_count}\n"));
        self.rebuild_half_edges();

        self.smooth_around_vertices(&compacted, 3, 5);
    }
}

/// Closest point on a triangle (mirrors the anonymous-namespace
/// `closestPointOnTriangle`; FMA per the module note).
fn closest_point_on_triangle(p: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = Vector3::dot_product(&ab, &ap);
    let d2 = Vector3::dot_product(&ac, &ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = p - b;
    let d3 = Vector3::dot_product(&ab, &bp);
    let d4 = Vector3::dot_product(&ac, &bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let vc = fma_first_sub(d1, d4, d3, d2);
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denom = d1 - d3;
        let v = if 0.0 != denom { d1 / denom } else { 0.0 };
        return add_scaled_vec3(a, ab, v);
    }

    let cp = p - c;
    let d5 = Vector3::dot_product(&ab, &cp);
    let d6 = Vector3::dot_product(&ac, &cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = fma_first_sub(d5, d2, d1, d6);
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denom = d2 - d6;
        let w = if 0.0 != denom { d2 / denom } else { 0.0 };
        return add_scaled_vec3(a, ac, w);
    }

    let va = fma_first_sub(d3, d6, d5, d4);
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let denom = (d4 - d3) + (d5 - d6);
        let w = if 0.0 != denom { (d4 - d3) / denom } else { 0.0 };
        return add_scaled_vec3(b, c - b, w);
    }

    let denom = va + vb + vc;
    if 0.0 == denom {
        return a;
    }
    let v = vb / denom;
    let w = vc / denom;
    add_two_scaled_vec3(a, ab, v, ac, w)
}

#[cfg(test)]
mod cxx_hash_tests {
    //! Differential oracle for the [`CxxSet`]/[`CxxMap`] emulation: replays
    //! the fixture's CXXHASH op log (bucket count, size, and full iteration
    //! order asserted after every op) and validates [`cxx_next_prime`]
    //! against the NEXTPRIME ranges. `cxx_hash_chains` pins the
    //! front-of-chain rule directly (values from the libc++ probe).

    use super::*;

    const FIXTURE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../tests/fixtures/quadextractor_diff.txt"
    );

    fn parse_usize(token: &str) -> usize {
        token.parse::<usize>().unwrap()
    }

    /// Split an op line at `=>` into (op tokens, `bc`, `n`, raw order tail).
    fn split_state(line: &str) -> (Vec<&str>, usize, usize, &str) {
        let (op, state) = line.split_once("=>").unwrap();
        let op: Vec<&str> = op.split_whitespace().collect();
        let state: Vec<&str> = state.split_whitespace().collect();
        assert_eq!(state.len(), 3, "bad state: {line}");
        let bc = parse_usize(state[0].strip_prefix("bc=").unwrap());
        let n = parse_usize(state[1].strip_prefix("n=").unwrap());
        let order = state[2].strip_prefix("order=").unwrap();
        (op, bc, n, order)
    }

    fn parse_order_set(tail: &str) -> Vec<usize> {
        if tail.is_empty() {
            Vec::new()
        } else {
            tail.split(',').map(parse_usize).collect()
        }
    }

    fn parse_order_map(tail: &str) -> Vec<(usize, usize)> {
        if tail.is_empty() {
            Vec::new()
        } else {
            tail.split(',')
                .map(|pair| {
                    let (k, v) = pair.split_once(':').unwrap();
                    (parse_usize(k), parse_usize(v))
                })
                .collect()
        }
    }

    fn flag(token: &str, name: &str) -> bool {
        token.strip_prefix(name).unwrap() == "1"
    }

    #[test]
    fn cxx_hash_oracle() {
        let text = std::fs::read_to_string(FIXTURE).unwrap();
        let mut section = "";
        let mut set = CxxSet::new();
        let mut map = CxxMap::new();
        let mut set_snaps: Vec<CxxSet> = Vec::new();
        let mut map_snaps: Vec<CxxMap<usize>> = Vec::new();
        let mut set_ops = 0;
        let mut map_ops = 0;
        for line in text.lines() {
            match line {
                "CXXHASH_BEGIN" => section = "hash",
                "CXXHASH_END" => section = "",
                "NEXTPRIME_BEGIN" => section = "prime",
                "NEXTPRIME_END" => section = "",
                _ => {}
            }
            if section != "hash" || (!line.starts_with("HS ") && !line.starts_with("HM ")) {
                continue;
            }
            let (op, bc, n, tail) = split_state(line);
            if op[0] == "HS" {
                set_ops += 1;
                match op[1] {
                    "I" => {
                        let key = parse_usize(op[2]);
                        assert_eq!(set.insert(key), flag(op[3], "ins="), "line: {line}");
                    }
                    "E" => {
                        let key = parse_usize(op[2]);
                        assert_eq!(set.remove(&key), flag(op[3], "erased="), "line: {line}");
                    }
                    "EI" => {
                        let pos = parse_usize(op[2]);
                        assert!(pos < set.order_vec().len(), "line: {line}");
                        let key = parse_usize(op[3].strip_prefix("key=").unwrap());
                        assert_eq!(set.order_vec()[pos], key, "line: {line}");
                        assert!(set.remove(&key), "line: {line}");
                        let next = op[4].strip_prefix("next=").unwrap();
                        if next == "END" {
                            assert_eq!(set.order_vec().len(), pos, "line: {line}");
                        } else {
                            assert_eq!(set.order_vec()[pos], parse_usize(next), "line: {line}");
                        }
                    }
                    "C" => set.clear(),
                    "HAS" => {
                        let key = parse_usize(op[2]);
                        assert_eq!(set.contains(&key), flag(op[3], "has="), "line: {line}");
                    }
                    "SNAP" => {
                        let id = parse_usize(op[2]);
                        assert_eq!(id, set_snaps.len(), "line: {line}");
                        set_snaps.push(set.clone());
                    }
                    "CHECK" => {
                        let id = parse_usize(op[2]);
                        let snap = &set_snaps[id];
                        assert_eq!(snap.buckets(), bc, "line: {line}");
                        assert_eq!(snap.len(), n, "line: {line}");
                        assert_eq!(snap.order_vec(), parse_order_set(tail), "line: {line}");
                        continue;
                    }
                    "RANGE" | "INIT" => {
                        let count = parse_usize(op[2]);
                        let elems: Vec<usize> =
                            op[3..3 + count].iter().map(|t| parse_usize(t)).collect();
                        let fresh: CxxSet = elems.into_iter().collect();
                        assert_eq!(fresh.buckets(), bc, "line: {line}");
                        assert_eq!(fresh.len(), n, "line: {line}");
                        assert_eq!(fresh.order_vec(), parse_order_set(tail), "line: {line}");
                        continue;
                    }
                    _ => panic!("unknown set op: {line}"),
                }
                assert_eq!(set.buckets(), bc, "line: {line}");
                assert_eq!(set.len(), n, "line: {line}");
                assert_eq!(set.order_vec(), parse_order_set(tail), "line: {line}");
            } else {
                map_ops += 1;
                match op[1] {
                    "I" => {
                        let key = parse_usize(op[2]);
                        let value = parse_usize(op[3]);
                        assert_eq!(
                            map.insert_new(key, value),
                            flag(op[4], "ins="),
                            "line: {line}"
                        );
                    }
                    "BSET" => {
                        map.set(parse_usize(op[2]), parse_usize(op[3]));
                    }
                    "BADD" => {
                        let key = parse_usize(op[2]);
                        let delta = parse_usize(op[3]);
                        *map.get_or_default(key) += delta;
                        let want = parse_usize(op[4].strip_prefix("val=").unwrap());
                        assert_eq!(map.get(&key).copied(), Some(want), "line: {line}");
                    }
                    "BREAD" => {
                        let key = parse_usize(op[2]);
                        let want = parse_usize(op[3].strip_prefix("val=").unwrap());
                        assert_eq!(*map.get_or_default(key), want, "line: {line}");
                    }
                    "E" => {
                        let key = parse_usize(op[2]);
                        assert_eq!(
                            map.remove(&key).is_some(),
                            flag(op[3], "erased="),
                            "line: {line}"
                        );
                    }
                    "EI" => {
                        let pos = parse_usize(op[2]);
                        assert!(pos < map.len(), "line: {line}");
                        let key = parse_usize(op[3].strip_prefix("key=").unwrap());
                        assert_eq!(map.order_vec()[pos].0, key, "line: {line}");
                        assert!(map.remove(&key).is_some(), "line: {line}");
                        let next = op[4].strip_prefix("next=").unwrap();
                        if next == "END" {
                            assert_eq!(map.len(), pos, "line: {line}");
                        } else {
                            assert_eq!(map.order_vec()[pos].0, parse_usize(next), "line: {line}");
                        }
                    }
                    "C" => map.clear(),
                    "FIND" => {
                        let key = parse_usize(op[2]);
                        if flag(op[3], "found=") {
                            let want = parse_usize(op[4].strip_prefix("val=").unwrap());
                            assert_eq!(map.get(&key).copied(), Some(want), "line: {line}");
                        } else {
                            assert_eq!(map.get(&key), None, "line: {line}");
                        }
                    }
                    "SNAP" => {
                        let id = parse_usize(op[2]);
                        assert_eq!(id, map_snaps.len(), "line: {line}");
                        map_snaps.push(map.clone());
                    }
                    "CHECK" => {
                        let id = parse_usize(op[2]);
                        let snap = &map_snaps[id];
                        assert_eq!(snap.buckets(), bc, "line: {line}");
                        assert_eq!(snap.len(), n, "line: {line}");
                        let pairs: Vec<(usize, usize)> = snap.order_vec();
                        assert_eq!(pairs, parse_order_map(tail), "line: {line}");
                        continue;
                    }
                    "RANGEINS" => {
                        let count = parse_usize(op[2]);
                        let mut fresh = CxxMap::new();
                        for token in &op[3..3 + count] {
                            let key = parse_usize(token);
                            fresh.insert_new(key, (key % 100003) * 10 + 1);
                        }
                        assert_eq!(fresh.buckets(), bc, "line: {line}");
                        assert_eq!(fresh.len(), n, "line: {line}");
                        let pairs: Vec<(usize, usize)> = fresh.order_vec();
                        assert_eq!(pairs, parse_order_map(tail), "line: {line}");
                        continue;
                    }
                    _ => panic!("unknown map op: {line}"),
                }
                assert_eq!(map.buckets(), bc, "line: {line}");
                assert_eq!(map.len(), n, "line: {line}");
                let pairs: Vec<(usize, usize)> = map.order_vec();
                assert_eq!(pairs, parse_order_map(tail), "line: {line}");
            }
        }
        assert!(set_ops > 1000, "set ops replayed: {set_ops}");
        assert!(map_ops > 1000, "map ops replayed: {map_ops}");
        println!("cxx_hash_oracle: {set_ops} set ops + {map_ops} map ops replayed");
    }

    #[test]
    fn cxx_next_prime_oracle() {
        let text = std::fs::read_to_string(FIXTURE).unwrap();
        let mut section = "";
        let mut ranges = 0;
        let mut tails = 0;
        for line in text.lines() {
            match line {
                "NEXTPRIME_BEGIN" => section = "prime",
                "NEXTPRIME_END" => section = "",
                _ => {}
            }
            if section != "prime" {
                continue;
            }
            if let Some(rest) = line.strip_prefix("NP ") {
                let parts: Vec<&str> = rest.split_whitespace().collect();
                let (lo, hi, p) = (
                    parse_usize(parts[0]),
                    parse_usize(parts[1]),
                    parse_usize(parts[2]),
                );
                // `next_prime` is nondecreasing and every range ends at its
                // prime, so the endpoints prove the whole range.
                assert_eq!(cxx_next_prime(lo), p, "range {lo}..={hi}");
                assert_eq!(cxx_next_prime(hi), p, "range {lo}..={hi}");
                ranges += 1;
            } else if let Some(rest) = line.strip_prefix("NP_TAIL ") {
                let parts: Vec<&str> = rest.split_whitespace().collect();
                assert_eq!(
                    cxx_next_prime(parse_usize(parts[0])),
                    parse_usize(parts[1]),
                    "tail {rest}"
                );
                tails += 1;
            }
        }
        assert!(ranges > 20000, "ranges checked: {ranges}");
        assert_eq!(tails, 7, "tails checked: {tails}");
        assert_eq!(cxx_next_prime(0), 0);
        println!("cxx_next_prime_oracle: {ranges} ranges + {tails} tails checked");
    }

    /// Front-of-chain placement, pinned without the fixture (values from the
    /// libc++ probe: D map20/brackets/copy+ins, E coll12).
    #[test]
    fn cxx_hash_chains() {
        let mut map = CxxMap::new();
        for i in 0..20 {
            map.insert_new(i * 3, i * 100);
        }
        assert_eq!(map.buckets(), 23);
        let keys: Vec<usize> = map.iter().map(|(k, _)| *k).collect();
        assert_eq!(
            keys,
            vec![
                57, 54, 51, 48, 45, 42, 39, 36, 33, 30, 27, 24, 21, 18, 15, 12, 9, 6, 3, 0
            ]
        );
        // 999 lands in 33's chain (both = 10 mod 23): ahead of 33.
        map.set(999, 5);
        let keys: Vec<usize> = map.iter().map(|(k, _)| *k).collect();
        assert_eq!(keys[8], 999);
        assert_eq!(keys[9], 33);
        // 1000 opens a chain with the head (both = 11 mod 23): new head.
        map.get_or_default(1000);
        assert_eq!(map.order_vec()[0].0, 1000);
        assert_eq!(map.order_vec()[1].0, 57);
        // Copies preserve order and count; the copy then diverges alone.
        let mut copy = map.clone();
        assert_eq!(copy.buckets(), map.buckets());
        copy.set(7, 700);
        let copy_keys: Vec<usize> = copy.iter().map(|(k, _)| *k).collect();
        let pos33 = copy_keys.iter().position(|k| *k == 33).unwrap();
        assert_eq!(copy_keys[pos33 + 1], 7);
        assert_eq!(copy_keys[pos33 + 2], 30);
        assert!(!map.contains_key(&7));
        // Collision chain: all multiples of 23 share one chain, newest first.
        let mut set = CxxSet::new();
        for i in 0..12 {
            set.insert(i * 23);
        }
        assert_eq!(set.buckets(), 23);
        assert_eq!(
            set.order_vec(),
            vec![253, 230, 207, 184, 161, 138, 115, 92, 69, 46, 23, 0]
        );
        // Erase unlinks; clear keeps the count; reuse continues there.
        assert!(set.remove(&0));
        assert!(!set.remove(&999999));
        assert_eq!(set.buckets(), 23);
        set.clear();
        assert_eq!(set.buckets(), 23);
        assert!(set.is_empty());
        set.insert(42);
        assert_eq!(set.order_vec(), vec![42]);
    }

    /// The sorted per-round indexes reproduce the `BTreeMap` builds
    /// exactly: identical keys, values, group order, and lookup behavior
    /// (hits, misses, and face-list order) over polygon soups including
    /// degenerate faces.
    #[test]
    fn fixpoint_indexes_match_btree() {
        let cases: Vec<Vec<Vec<usize>>> = vec![
            vec![],
            vec![vec![0, 1, 2]],
            vec![vec![0, 1, 2, 3], vec![3, 2, 4], vec![4, 5, 6, 7, 8]],
            // Shared edges in both windings, repeated vertices, a
            // two-sided sliver, and an unused vertex id (9) for misses.
            vec![
                vec![0, 1, 2, 3],
                vec![3, 2, 1, 0],
                vec![2, 3, 4],
                vec![4, 4, 5],
                vec![6, 7],
                vec![7, 6, 7],
            ],
            // A grid-ish strip (every interior edge shared by two faces).
            (0..40)
                .map(|r| (0..4).map(|c| r * 4 + c).collect::<Vec<_>>())
                .collect(),
        ];
        for (case_index, polygons) in cases.iter().enumerate() {
            let context = format!("case {case_index}");
            // Reference: the exact serial `BTreeMap` builds the passes used.
            let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
            let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
            let mut vertex_face_counts: BTreeMap<usize, usize> = BTreeMap::new();
            for (face_index, face) in polygons.iter().enumerate() {
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    edge_faces
                        .entry(QuadExtractor::edge_of(face[i], face[j]))
                        .or_default()
                        .push(face_index);
                    vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                    vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
                }
                for vertex in face {
                    *vertex_face_counts.entry(*vertex).or_default() += 1;
                }
            }
            // Edge groups: same keys, same ascending face vecs, same order.
            let edge_index = EdgeFaceIndex::build(polygons);
            let expected_groups: Vec<((usize, usize), Vec<usize>)> =
                edge_faces.into_iter().collect();
            let index_groups: Vec<((usize, usize), Vec<usize>)> = edge_index
                .groups()
                .map(|(edge, faces)| (edge, faces.to_vec()))
                .collect();
            assert_eq!(index_groups, expected_groups, "groups {context}");
            // Edge lookups: same hits (order included) and misses.
            let mut probe_edges: Vec<(usize, usize)> =
                index_groups.iter().map(|(edge, _)| *edge).collect();
            probe_edges.extend([(0, 9), (9, 9), (100, 200)]);
            for edge in &probe_edges {
                let expected = expected_groups
                    .iter()
                    .find(|(key, _)| key == edge)
                    .map(|(_, faces)| faces.as_slice());
                assert_eq!(edge_index.get(edge), expected, "edge {edge:?} {context}");
            }
            // Neighbor lookups: same sets (ascending) and misses.
            let neighbor_index = NeighborIndex::build(polygons);
            for vertex in 0..12 {
                let expected: Option<Vec<usize>> = vertex_neighbors
                    .get(&vertex)
                    .map(|set| set.iter().copied().collect());
                assert_eq!(
                    neighbor_index.get(&vertex).map(|s| s.to_vec()),
                    expected,
                    "neighbors {vertex} {context}"
                );
                assert_eq!(
                    neighbor_index.get_or_empty(&vertex),
                    expected.as_deref().unwrap_or(&[]),
                    "neighbors-or-empty {vertex} {context}"
                );
            }
            // Face counts: same keys, counts, and misses.
            let count_index = FaceCountIndex::build(polygons);
            for vertex in 0..12 {
                assert_eq!(
                    count_index.get(&vertex).copied(),
                    vertex_face_counts.get(&vertex).copied(),
                    "counts {vertex} {context}"
                );
            }
        }
    }
}
