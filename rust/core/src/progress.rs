//! Line-by-line mirror of `core/progress.cppm` (`retopo.core.progress`).
//!
//! The C++ side is a single alias,
//! `std::function<void(float fraction, const char* name)>`, with the
//! contract that a handler must be safe to call from several TBB worker
//! threads at once. The Rust mirror keeps the same shape (one alias, same
//! argument order and widths: `f32` fraction, borrowed name) and encodes
//! the thread-safety contract as `Send + Sync` on the trait object.
//!
//! `&str` mirrors `const char*`: call sites pass short static step names,
//! so no owned `String` crosses the callback boundary on either side.

/// Progress callback: `fraction` runs 0..1 within the stage, `name`
/// labels the step that is starting. `Send + Sync` mirrors the C++
/// "safe to call from several threads at once" requirement.
pub type ProgressHandler = Box<dyn Fn(f32, &str) + Send + Sync>;
