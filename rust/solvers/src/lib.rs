// Port mandate: index loops mirror the C++ 1:1, so the lint that wants
// iterators stays off crate-wide.
#![allow(clippy::needless_range_loop)]

pub mod constrained;
pub mod mixed_integer;
