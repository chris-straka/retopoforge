//! TEMPORARY reference copy of the pre-optimization `CxxSet`/`CxxMap`
//! containers (exact pre-change code, renamed), used only by the randomized
//! old-vs-new differential test. Deleted once the new containers are proven.
//!
//! This file is scaffolding for lane/par-single-island verification and must
//! not survive the lane.

use std::collections::{BTreeMap, BTreeSet};

/// `std::__constrain_hash`, literal (the `bc == 0` arm yields `h`, exactly
/// like the C++; unreachable here — every caller rehashes from 0 first).
#[inline]
pub fn ref_constrain_hash(hash: usize, buckets: usize) -> usize {
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
pub fn ref_is_hash_pow2(buckets: usize) -> bool {
    buckets > 2 && buckets & (buckets - 1) == 0
}

/// Smallest prime `>= n` (`std::__next_prime`, probed; `next_prime(0) == 0`
/// is a dylib quirk the mirror keeps). Primality is deterministic
/// Miller-Rabin (7-base set, exact for all `u64`).
pub fn ref_next_prime(n: usize) -> usize {
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
        if ref_is_prime(candidate as u64) {
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
pub fn ref_is_prime(n: u64) -> bool {
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
        let mut x = ref_mod_pow(a, d, n);
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

pub fn ref_mod_pow(mut base: u64, mut exp: u64, modulus: u64) -> u64 {
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

/// `__do_rehash` unique-keys path, literal: regroup `order` in place under
/// the new bucket count. `chain_start` plays `__bucket_list_` (run-start
/// index per chain; chains stay contiguous, so runs and chains coincide).
pub fn ref_regroup(order: &mut Vec<usize>, buckets: usize) {
    if order.is_empty() {
        return;
    }
    let mut chain_start: BTreeMap<usize, usize> = BTreeMap::new();
    let mut previous_hash = ref_constrain_hash(order[0], buckets);
    chain_start.insert(previous_hash, 0);
    let mut i = 1;
    while i < order.len() {
        let chained = ref_constrain_hash(order[i], buckets);
        if chained == previous_hash {
            i += 1;
        } else if !chain_start.contains_key(&chained) {
            chain_start.insert(chained, i);
            previous_hash = chained;
            i += 1;
        } else {
            let front = chain_start[&chained];
            debug_assert!(front < i);
            let node = order.remove(i);
            order.insert(front, node);
            // Refresh run starts over the shifted window only.
            chain_start.retain(|_, index| *index < front || *index > i);
            for k in front..=i {
                if k == 0
                    || ref_constrain_hash(order[k], buckets)
                        != ref_constrain_hash(order[k - 1], buckets)
                {
                    chain_start.insert(ref_constrain_hash(order[k], buckets), k);
                }
            }
        }
    }
}

/// Grow step shared by both containers: mirrors the `size + 1 > bc *
/// max_load` trigger plus `__rehash_unique(max(2 * bc + !is_pow2(bc),
/// size + 1))`. (`usize` math is exact here; the C++ float spell agrees
/// below 2^24 elements.) Returns whether a rehash regrouped `order`.
pub fn ref_maybe_rehash(order: &mut Vec<usize>, buckets: &mut usize) -> bool {
    if order.len() + 1 > *buckets {
        let arg = (2 * *buckets + usize::from(!ref_is_hash_pow2(*buckets))).max(order.len() + 1);
        let grown = if arg == 1 {
            2
        } else if arg & (arg - 1) != 0 {
            ref_next_prime(arg)
        } else {
            arg
        };
        debug_assert!(grown > *buckets);
        *buckets = grown;
        ref_regroup(order, grown);
        return true;
    }
    false
}

/// `__emplace_unique` position rule: before the chain's first node, or at
/// the list front when the chain is empty.
pub fn ref_insert_position(order: &[usize], buckets: usize, key: usize) -> usize {
    let chained = ref_constrain_hash(key, buckets);
    order
        .iter()
        .position(|k| ref_constrain_hash(*k, buckets) == chained)
        .unwrap_or(0)
}

/// Emulated `std::unordered_set<size_t>`: `order` is the node list
/// (iteration order), `present` the membership index, `buckets` the table's
/// bucket count. Iteration yields list order, like the C++ iterators.
#[derive(Debug, Default)]
pub struct RefSet {
    order: Vec<usize>,
    present: BTreeSet<usize>,
    buckets: usize,
}

impl Clone for RefSet {
    /// Copy ctor: a copy of an EMPTY table is fresh (`bc == 0`) — the C++
    /// copy ctor early-returns before allocating buckets when the source
    /// has no nodes. Non-empty tables keep list order and the count.
    fn clone(&self) -> Self {
        if self.order.is_empty() {
            Self::new()
        } else {
            Self {
                order: self.order.clone(),
                present: self.present.clone(),
                buckets: self.buckets,
            }
        }
    }
}

impl RefSet {
    pub fn new() -> Self {
        Self {
            order: Vec::new(),
            present: BTreeSet::new(),
            buckets: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.order.len()
    }

    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    pub fn contains(&self, key: &usize) -> bool {
        self.present.contains(key)
    }

    /// C++ `insert`: no-op (returns `false`) when the key is present.
    pub fn insert(&mut self, key: usize) -> bool {
        if self.present.contains(&key) {
            return false;
        }
        ref_maybe_rehash(&mut self.order, &mut self.buckets);
        let at = ref_insert_position(&self.order, self.buckets, key);
        self.order.insert(at, key);
        self.present.insert(key);
        true
    }

    pub fn remove(&mut self, key: &usize) -> bool {
        if !self.present.remove(key) {
            return false;
        }
        // Unlink: survivors keep order, the count never shrinks (`remove`).
        if let Some(at) = self.order.iter().position(|k| k == key) {
            debug_assert!(self.order[at] == *key);
            self.order.remove(at);
        }
        true
    }

    /// `clear`: empties the table but keeps the bucket count.
    #[cfg_attr(not(test), allow(dead_code))] // container oracle tests
    pub fn clear(&mut self) {
        self.order.clear();
        self.present.clear();
    }

    pub fn iter(&self) -> std::slice::Iter<'_, usize> {
        self.order.iter()
    }
}

impl<'a> IntoIterator for &'a RefSet {
    type Item = &'a usize;
    type IntoIter = std::slice::Iter<'a, usize>;

    fn into_iter(self) -> Self::IntoIter {
        self.order.iter()
    }
}

impl FromIterator<usize> for RefSet {
    /// Range/iterator insert: sequential per-element inserts in order.
    fn from_iter<I: IntoIterator<Item = usize>>(iter: I) -> Self {
        let mut set = Self::new();
        set.extend(iter);
        set
    }
}

impl<const N: usize> From<[usize; N]> for RefSet {
    fn from(values: [usize; N]) -> Self {
        values.into_iter().collect()
    }
}

impl Extend<usize> for RefSet {
    fn extend<I: IntoIterator<Item = usize>>(&mut self, iter: I) {
        for key in iter {
            self.insert(key);
        }
    }
}

/// Emulated `std::unordered_map<size_t, V>`: same list/count machine as
/// [`RefSet`], with values stored inline so iteration needs no lookup.
/// There is deliberately no `insert` or `entry` (see the section note);
/// every write names its C++ counterpart.
#[derive(Debug, Default)]
pub struct RefMap<V> {
    order: Vec<(usize, V)>,
    index: BTreeMap<usize, usize>,
    buckets: usize,
}

impl<V: Clone> Clone for RefMap<V> {
    /// Copy ctor (see [`RefSet`]): empty tables copy fresh (`bc == 0`).
    fn clone(&self) -> Self {
        if self.order.is_empty() {
            Self::new()
        } else {
            Self {
                order: self.order.clone(),
                index: self.index.clone(),
                buckets: self.buckets,
            }
        }
    }
}

/// Borrowed `(key, value)` pairs in list order.
pub struct RefMapIter<'a, V> {
    inner: std::slice::Iter<'a, (usize, V)>,
}

impl<'a, V> Iterator for RefMapIter<'a, V> {
    type Item = (&'a usize, &'a V);

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|(key, value)| (key, value))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.inner.size_hint()
    }
}

impl<'a, V> ExactSizeIterator for RefMapIter<'a, V> {}

impl<V> RefMap<V> {
    pub fn new() -> Self {
        Self {
            order: Vec::new(),
            index: BTreeMap::new(),
            buckets: 0,
        }
    }

    #[cfg_attr(not(test), allow(dead_code))] // container oracle tests
    pub fn len(&self) -> usize {
        self.order.len()
    }

    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    pub fn contains_key(&self, key: &usize) -> bool {
        self.index.contains_key(key)
    }

    pub fn get(&self, key: &usize) -> Option<&V> {
        self.index.get(key).map(|at| &self.order[*at].1)
    }

    pub fn get_mut(&mut self, key: &usize) -> Option<&mut V> {
        let at = *self.index.get(key)?;
        Some(&mut self.order[at].1)
    }

    /// Insert the key at list position `at`, shifting later indices.
    pub fn insert_at(&mut self, at: usize, key: usize, value: V) {
        for slot in self.index.values_mut() {
            if *slot >= at {
                *slot += 1;
            }
        }
        self.order.insert(at, (key, value));
        self.index.insert(key, at);
    }

    /// The key list, for the shared order helpers.
    pub fn key_list(&self) -> Vec<usize> {
        self.order.iter().map(|(key, _)| *key).collect()
    }

    /// C++ `insert`/`emplace`: keeps the old value (returns `false`) when
    /// the key is present.
    pub fn insert_new(&mut self, key: usize, value: V) -> bool {
        if self.index.contains_key(&key) {
            return false;
        }
        self.insert_fresh(key, value);
        true
    }

    /// C++ `operator[] = value`: overwrites when present, inserts (with the
    /// same order effects) when absent.
    #[cfg_attr(not(test), allow(dead_code))] // container oracle tests
    pub fn set(&mut self, key: usize, value: V) {
        if let Some(at) = self.index.get(&key) {
            self.order[*at].1 = value;
            return;
        }
        self.insert_fresh(key, value);
    }

    /// C++ `operator[]` for read-mutate: default-inserts on absence.
    pub fn get_or_default(&mut self, key: usize) -> &mut V
    where
        V: Default,
    {
        if let Some(&at) = self.index.get(&key) {
            return &mut self.order[at].1;
        }
        let at = self.insert_fresh(key, V::default());
        &mut self.order[at].1
    }

    /// Fresh-key insert with the C++ order effects (rehash, then
    /// front-of-chain placement). The key must be absent. Returns the list
    /// position the key landed on.
    pub fn insert_fresh(&mut self, key: usize, value: V) -> usize {
        debug_assert!(!self.index.contains_key(&key));
        let mut keys = self.key_list();
        if ref_maybe_rehash(&mut keys, &mut self.buckets) {
            self.reorder_like(&keys);
        }
        let at = ref_insert_position(&self.key_list(), self.buckets, key);
        self.insert_at(at, key, value);
        at
    }

    /// Reorder value pairs (and the index) to match a regrouped key list.
    pub fn reorder_like(&mut self, keys: &[usize]) {
        debug_assert_eq!(keys.len(), self.order.len());
        let mut pairs: BTreeMap<usize, (usize, V)> = BTreeMap::new();
        for (key, value) in std::mem::take(&mut self.order) {
            pairs.insert(key, (key, value));
        }
        let mut reordered = Vec::with_capacity(keys.len());
        for key in keys {
            if let Some(pair) = pairs.remove(key) {
                reordered.push(pair);
            }
        }
        debug_assert!(pairs.is_empty() && reordered.len() == keys.len());
        self.order = reordered;
        self.index.clear();
        for (at, (key, _)) in self.order.iter().enumerate() {
            self.index.insert(*key, at);
        }
    }

    pub fn remove(&mut self, key: &usize) -> Option<V> {
        let at = self.index.remove(key)?;
        debug_assert!(at < self.order.len() && self.order[at].0 == *key);
        for slot in self.index.values_mut() {
            if *slot > at {
                *slot -= 1;
            }
        }
        Some(self.order.remove(at).1)
    }

    /// `clear`: empties the table but keeps the bucket count.
    #[cfg_attr(not(test), allow(dead_code))] // container oracle tests
    pub fn clear(&mut self) {
        self.order.clear();
        self.index.clear();
    }

    pub fn iter(&self) -> RefMapIter<'_, V> {
        RefMapIter {
            inner: self.order.iter(),
        }
    }

    /// First key in list order (`nextMap.begin()`), or `None` when empty.
    pub fn first_key(&self) -> Option<&usize> {
        self.order.first().map(|(key, _)| key)
    }
}

impl<'a, V> IntoIterator for &'a RefMap<V> {
    type Item = (&'a usize, &'a V);
    type IntoIter = RefMapIter<'a, V>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<V> FromIterator<(usize, V)> for RefMap<V> {
    /// Range/iterator insert: sequential per-element inserts in order.
    fn from_iter<I: IntoIterator<Item = (usize, V)>>(iter: I) -> Self {
        let mut map = Self::new();
        map.extend(iter);
        map
    }
}

impl<V> Extend<(usize, V)> for RefMap<V> {
    fn extend<I: IntoIterator<Item = (usize, V)>>(&mut self, iter: I) {
        for (key, value) in iter {
            self.insert_new(key, value);
        }
    }
}

impl RefSet {
    pub fn buckets(&self) -> usize {
        self.buckets
    }
}

impl<V> RefMap<V> {
    pub fn buckets(&self) -> usize {
        self.buckets
    }
}
