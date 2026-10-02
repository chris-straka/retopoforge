//! Engine progress reporting: `N% done. <status>` lines on stderr.
//!
//! The engine calls back with a progress fraction and a status string;
//! at most one line prints per percent-point per status (a set, so
//! non-adjacent repeats like the old `3% … / 3% …` stutter stay dead
//! too). The state sits behind a `Mutex` because the callback fires
//! from worker threads. One line per update, no `\r` tricks — logs
//! stay greppable.

use retopo_core::auto_remesher::AutoRemesher;
use std::collections::HashSet;
use std::ffi::c_void;
use std::io::Write;
use std::sync::Mutex;

#[derive(Default)]
pub(crate) struct ProgressState {
    seen: HashSet<(i32, String)>,
}

fn report_progress(tag: *mut c_void, progress: f32, status: &str) {
    let state = unsafe { &*(tag as *const Mutex<ProgressState>) };
    let mut state = state.lock().unwrap();
    let percent = (progress * 100.0) as i32;
    if !state.seen.insert((percent, status.to_string())) {
        return;
    }
    let mut err = std::io::stderr().lock();
    if status.is_empty() {
        let _ = writeln!(err, "{percent}% done.");
    } else {
        let _ = writeln!(err, "{percent}% done. {status}");
    }
    let _ = err.flush();
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
