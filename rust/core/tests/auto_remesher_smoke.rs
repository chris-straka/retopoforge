// TEMPORARY smoke probe (deleted before the lane lands): one small remesh
// end to end, exercising the FFI decimation path off, threads, progress
// handler + tag, and the phase report.
use retopo_core::auto_remesher::{AutoRemesher, AutoRemesherProgressHandler};
use retopo_core::vector3::Vector3;
use std::ffi::c_void;
use std::sync::{Arc, Mutex};

fn tag_box() -> (*mut c_void, Arc<Mutex<Vec<(f32, String)>>>) {
    let events: Arc<Mutex<Vec<(f32, String)>>> = Arc::new(Mutex::new(Vec::new()));
    let raw = Arc::into_raw(Arc::clone(&events)) as *mut c_void;
    (raw, events)
}

fn handler(tag: *mut c_void, progress: f32, status: &str) {
    let events = unsafe { &*(tag as *const Mutex<Vec<(f32, String)>>) };
    events
        .lock()
        .unwrap()
        .push((progress, status.to_string()));
}

fn cube() -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let c = [
        [-1.0, -1.0, -1.0],
        [1.0, -1.0, -1.0],
        [1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
    ];
    let v: Vec<Vector3> = c.iter().map(|p| Vector3::new(p[0], p[1], p[2])).collect();
    let f = [
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [2, 3, 7],
        [2, 7, 6],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ];
    let t: Vec<Vec<usize>> = f.iter().map(|t| vec![t[0], t[1], t[2]]).collect();
    (v, t)
}

#[test]
fn smoke_box_remesh() {
    let (v, t) = cube();
    let mut r = AutoRemesher::new(&v, &t);
    r.set_target_triangle_count(200);
    let (tag, events) = tag_box();
    r.set_tag(tag);
    let h: AutoRemesherProgressHandler = handler;
    r.set_progress_handler(Some(h));
    assert!(r.remesh());
    assert!(!r.remeshed_vertices().is_empty());
    assert!(!r.remeshed_quads().is_empty());
    assert!(!r.phase_report().is_empty());
    let ev = events.lock().unwrap();
    assert!(!ev.is_empty());
    assert_eq!(ev.last().unwrap().1, "Done");
    assert_eq!(ev.last().unwrap().0, 1.0);
    eprintln!("quads={} verts={}", r.remeshed_quads().len(), r.remeshed_vertices().len());
    for line in r.phase_report() {
        eprintln!("PHASE {line}");
    }
    eprintln!("events={}", ev.len());
    unsafe {
        drop(Arc::from_raw(tag as *const Mutex<Vec<(f32, String)>>));
    }
}
