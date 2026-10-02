//! Engine progress reporting: `N% done. <status>` lines on stdout.
//!
//! The engine calls back with a progress fraction and a status string;
//! repeats of the same (percent, status) pair are suppressed. The state
//! sits behind a `Mutex` because the callback fires from worker threads.

use retopo_core::auto_remesher::AutoRemesher;
use std::ffi::c_void;
use std::io::Write;
use std::sync::Mutex;

pub(crate) struct ProgressState {
    last_percent: i32,
    last_status: String,
}

impl Default for ProgressState {
    fn default() -> Self {
        Self {
            last_percent: -1,
            last_status: String::new(),
        }
    }
}

fn report_progress(tag: *mut c_void, progress: f32, status: &str) {
    let state = unsafe { &*(tag as *const Mutex<ProgressState>) };
    let mut state = state.lock().unwrap();
    let percent = (progress * 100.0) as i32;
    if percent == state.last_percent && status == state.last_status {
        return;
    }
    state.last_percent = percent;
    state.last_status = status.to_string();
    let mut out = std::io::stdout().lock();
    if status.is_empty() {
        let _ = writeln!(out, "{percent}% done.");
    } else {
        let _ = writeln!(out, "{percent}% done. {status}");
    }
    let _ = out.flush();
}

/// Attach progress reporting to a remesher. The caller must hold `state`
/// in a local that outlives the `remesh()` call without moving.
pub(crate) fn attach_progress(remesher: &mut AutoRemesher, state: &Mutex<ProgressState>) {
    // SAFETY: the engine only dereferences the tag while `remesh()` runs,
    // synchronously, and the caller's local outlives that call; the
    // pointed-to `Mutex<ProgressState>` is never moved or dropped first.
    remesher.set_tag(state as *const Mutex<ProgressState> as *mut c_void);
    remesher.set_progress_handler(Some(report_progress));
}
