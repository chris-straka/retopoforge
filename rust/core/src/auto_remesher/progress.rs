use super::{
    AutoRemesherProgressHandler, PARALLEL_PHASE_BEGIN, PARALLEL_PHASE_END, cxx_max, cxx_min,
};
use std::ffi::c_void;
use std::sync::Mutex;

pub(crate) struct StageTime {
    pub(crate) name: String,
    pub(crate) order: f32,
    pub(crate) microseconds: i64,
}

/// The mutex-guarded progress members (`m_threadProgress` and friends plus
/// the handler/tag pair). The C++ keeps these as bare members and captures
/// `this` in the stage closures; here they live behind an `Arc` so the
/// `'static` [`ProgressHandler`] closures can own them.
pub(crate) struct ProgressData {
    pub(crate) handler: Option<AutoRemesherProgressHandler>,
    pub(crate) tag: *mut c_void,
    pub(crate) thread_progress: Vec<f32>,
    pub(crate) thread_progress_weights: Vec<f32>,
    // Owned strings where the C++ stores borrowed `const char*` (see the
    // module docs); `None` mirrors `nullptr`.
    pub(crate) thread_status: Vec<Option<String>>,
    // The weighted sum of thread_progress, kept incrementally so that a
    // fine-grained update stays O(1) rather than a scan of every island.
    pub(crate) progress_sum: f64,
    pub(crate) reported_permille: i32,
    pub(crate) reported_status: Option<String>,
}

// SAFETY: `ProgressData` crosses threads only behind a `Mutex` (inside
// `ProgressState`), and the tag is an opaque cookie the handler interprets
// — the same contract as the C++ `void*`.
unsafe impl Send for ProgressData {}

pub(crate) struct ProgressState {
    pub(crate) progress_mutex: Mutex<ProgressData>,
    pub(crate) stage_timing_mutex: Mutex<Vec<StageTime>>,
}

impl ProgressState {
    pub(crate) fn new() -> Self {
        Self {
            progress_mutex: Mutex::new(ProgressData {
                handler: None,
                tag: std::ptr::null_mut(),
                thread_progress: Vec::new(),
                thread_progress_weights: Vec::new(),
                thread_status: Vec::new(),
                progress_sum: 0.0,
                reported_permille: -1,
                reported_status: None,
            }),
            stage_timing_mutex: Mutex::new(Vec::new()),
        }
    }

    /// Locks, ignoring poisoning: a poisoned mutex means a worker thread
    /// panicked, which the port treats as unreachable (workers only fail
    /// through `bool` returns, like the C++); the C++ `std::mutex` has no
    /// poisoning, so carrying on mirrors it.
    pub(crate) fn lock_progress(&self) -> std::sync::MutexGuard<'_, ProgressData> {
        self.progress_mutex
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    pub(crate) fn lock_stages(&self) -> std::sync::MutexGuard<'_, Vec<StageTime>> {
        self.stage_timing_mutex
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    /// Mirrors the handler-taking half of `updateProgress` (called with the
    /// lock already held, like the C++ `lock_guard` that spans the call).
    pub(crate) fn update_locked(
        data: &mut ProgressData,
        thread_index: usize,
        progress: f32,
        status: Option<&str>,
    ) {
        if data.handler.is_none() {
            return;
        }
        if thread_index >= data.thread_progress.len() {
            return;
        }
        if let Some(name) = status
            && !name.is_empty()
        {
            data.thread_status[thread_index] = Some(name.to_string());
        }
        if progress > data.thread_progress[thread_index] {
            // FMA audit (site B): Clang fuses to
            // llvm.fmuladd(delta, weight, sum) (both f32 operands
            // extended first, then one fused op). Transcribed explicitly.
            data.progress_sum = f64::from(progress - data.thread_progress[thread_index]).mul_add(
                f64::from(data.thread_progress_weights[thread_index]),
                data.progress_sum,
            );
            data.thread_progress[thread_index] = progress;
        }

        // FMA audit (site C): Clang fuses to
        // llvm.fmuladd(clamped, end - begin, begin). Transcribed explicitly.
        let overall = cxx_min(1.0, cxx_max(0.0, data.progress_sum)).mul_add(
            f64::from(PARALLEL_PHASE_END - PARALLEL_PHASE_BEGIN),
            f64::from(PARALLEL_PHASE_BEGIN),
        );

        // Steps now report many times per island, so only wake the UI when
        // the bar would actually move or the status line would change.
        let permille = (overall * 1000.0) as i32;
        let island_status = data.thread_status[thread_index].clone();
        if permille == data.reported_permille && island_status == data.reported_status {
            return;
        }
        data.reported_permille = permille;
        data.reported_status = island_status;

        // With several islands in flight, the run as a whole is only as far
        // along as its slowest island, so that is the step worth naming.
        let mut slowest = thread_index;
        for i in 0..data.thread_progress.len() {
            if data.thread_progress[i] < data.thread_progress[slowest] {
                slowest = i;
            }
        }
        let name = data.thread_status[slowest].clone().unwrap_or_default();
        let tag = data.tag;
        if let Some(handler) = data.handler {
            handler(tag, overall as f32, &name);
        }
    }

    /// Mirrors the direct `m_progressHandler(m_tag, progress, status)` calls
    /// (entry/exit announcements, never gated by the permille filter).
    pub(crate) fn report_direct(&self, progress: f32, status: &str) {
        let data = self.lock_progress();
        if let Some(handler) = data.handler {
            handler(data.tag, progress, status);
        }
    }

    /// The shared body of `accumulate_stage_time`, callable both through
    /// `&AutoRemesher` and from the stage closures that own only the `Arc`.
    pub(crate) fn accumulate(&self, name: &str, order: f32, microseconds: i64) {
        if name.is_empty() {
            return;
        }
        let mut stages = self.lock_stages();
        for it in stages.iter_mut() {
            if it.name == name {
                it.microseconds += microseconds;
                return;
            }
        }
        stages.push(StageTime {
            name: name.to_string(),
            order,
            microseconds,
        });
    }
}
