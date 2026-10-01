//! Shared deterministic parallelism helpers (std scoped threads, no rayon).
//!
//! Moved verbatim from `auto_remesher` (the approved pattern) so every
//! data-parallel loop in the crate shares one chunking discipline: fixed
//! disjoint partitioning, per-index independent bodies, no cross-thread
//! FP accumulation, and results that are identical for any chunking —
//! including one chunk per worker on any machine. Single-island runs stay
//! bit-deterministic run to run under all three helpers.

use std::thread;

/// Chunk length for one worker: the C++ sides use `tbb::parallel_for`
/// over the whole range and let TBB partition; here the range splits into
/// one chunk per worker thread. All uses are per-index independent, so the
/// partitioning is unobservable.
pub(crate) fn worker_chunk_len(n: usize) -> usize {
    debug_assert!(n > 0);
    let workers = thread::available_parallelism().map_or(1, |p| p.get());
    n.div_ceil(workers).max(1)
}

/// Mirrors `tbb::parallel_for(tbb::blocked_range<size_t>(0, items.len()),
/// ...)`: calls `f(i, &mut items[i])` for every index, spread over scoped
/// worker threads that each own disjoint `chunks_mut` slices.
pub(crate) fn parallel_each<T, F>(items: &mut [T], f: F)
where
    T: Send,
    F: Fn(usize, &mut T) + Send + Sync,
{
    if items.is_empty() {
        return;
    }
    let chunk_len = worker_chunk_len(items.len());
    // Share the worker body by reference: a `move` closure per chunk would
    // otherwise try to move `f` into the first thread.
    let f = &f;
    thread::scope(|s| {
        for (chunk_index, chunk) in items.chunks_mut(chunk_len).enumerate() {
            let base = chunk_index * chunk_len;
            s.spawn(move || {
                for (k, item) in chunk.iter_mut().enumerate() {
                    f(base + k, item);
                }
            });
        }
    });
}

/// Two-slice variant of [`parallel_each`] for loops that mutate two vectors
/// in lockstep (face normals + areas, normals + neighbor rings). Both
/// slices share the chunking, so worker `k` owns the same index range of
/// each. Panics on length mismatch like a `zip` that must stay aligned —
/// every call site passes same-length slices by construction.
pub(crate) fn parallel_each_zip2<T, U, F>(a: &mut [T], b: &mut [U], f: F)
where
    T: Send,
    U: Send,
    F: Fn(usize, &mut T, &mut U) + Send + Sync,
{
    debug_assert_eq!(a.len(), b.len(), "parallel zip slices must align");
    if a.is_empty() {
        return;
    }
    let chunk_len = worker_chunk_len(a.len());
    let f = &f;
    thread::scope(|s| {
        for (chunk_index, (chunk_a, chunk_b)) in a
            .chunks_mut(chunk_len)
            .zip(b.chunks_mut(chunk_len))
            .enumerate()
        {
            let base = chunk_index * chunk_len;
            s.spawn(move || {
                for (k, (x, y)) in chunk_a.iter_mut().zip(chunk_b.iter_mut()).enumerate() {
                    f(base + k, x, y);
                }
            });
        }
    });
}

/// Chunk-level variant: runs `f(base, chunk)` once per disjoint chunk and
/// collects the per-chunk results in chunk order (workers join in order,
/// so collection order never depends on scheduling).
///
/// For map phases whose partial results merge deterministically
/// (order-free merges or in-order concatenation): the merged result is
/// then identical for any chunking, which the caller's tests prove by
/// replaying with every chunk length (see `chunking_independence`).
/// `chunk_len_override` is that test hook (`None` sizes one chunk per
/// worker); production call sites always pass `None`.
pub(crate) fn parallel_chunks<T, R, F>(
    items: &[T],
    chunk_len_override: Option<usize>,
    f: F,
) -> Vec<R>
where
    T: Send + Sync,
    R: Send,
    F: Fn(usize, &[T]) -> R + Send + Sync,
{
    if items.is_empty() {
        return Vec::new();
    }
    let chunk_len = chunk_len_override
        .filter(|len| *len > 0)
        .unwrap_or_else(|| worker_chunk_len(items.len()));
    let f = &f;
    thread::scope(|s| {
        let mut handles = Vec::new();
        for (chunk_index, chunk) in items.chunks(chunk_len).enumerate() {
            let base = chunk_index * chunk_len;
            handles.push(s.spawn(move || f(base, chunk)));
        }
        handles
            .into_iter()
            .map(|handle| handle.join().expect("par worker panicked"))
            .collect()
    })
}

#[cfg(test)]
mod tests {
    use super::{parallel_chunks, parallel_each};

    /// The helpers themselves are chunking-independent: same body, every
    /// chunk length, identical results (the property every caller relies
    /// on for thread-count independence).
    #[test]
    fn chunking_independence() {
        let input: Vec<usize> = (0..1000).collect();
        let mut serial = vec![0usize; 1000];
        for (i, slot) in serial.iter_mut().enumerate() {
            *slot = i.wrapping_mul(31).wrapping_add(7);
        }
        for chunk_len in [1, 2, 3, 7, 16, 100, 333, 1000, 5000] {
            let parts = parallel_chunks(&input, Some(chunk_len), |base, chunk| {
                chunk
                    .iter()
                    .enumerate()
                    .map(|(k, _)| (base + k).wrapping_mul(31).wrapping_add(7))
                    .collect::<Vec<_>>()
            });
            let merged: Vec<usize> = parts.into_iter().flatten().collect();
            assert_eq!(merged, serial, "chunk_len {chunk_len}");
        }
        // `parallel_each` matches the serial loop too (default chunking).
        let mut par = vec![0usize; 1000];
        parallel_each(&mut par, |i, slot| {
            *slot = i.wrapping_mul(31).wrapping_add(7);
        });
        assert_eq!(par, serial);
        // Empty inputs stay empty (no workers spawned).
        let empty: Vec<usize> = Vec::new();
        assert!(parallel_chunks(&empty, None, |_, _| 1).is_empty());
    }
}
