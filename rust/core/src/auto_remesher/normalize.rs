use super::AutoRemesher;
use crate::vector3::Vector3;

impl AutoRemesher {
    /// Power-of-two scale mapping the input bbox diagonal into [1, 2),
    /// or 1.0 when no normalization applies: healthy scales (diag >= 1)
    /// take the identical unscaled path, and degenerate (zero),
    /// denormal, or non-finite diagonals are left for downstream to
    /// handle as today. Scaling up also refuses inputs whose largest
    /// coordinate would overflow to infinity.
    pub(crate) fn normalization_scale(vertices: &[Vector3]) -> f64 {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        let mut max_abs = 0.0f64;
        for v in vertices {
            for axis in 0..3 {
                let c = v[axis];
                lo[axis] = lo[axis].min(c);
                hi[axis] = hi[axis].max(c);
                max_abs = max_abs.max(c.abs());
            }
        }
        // `hypot` (no intermediate square, so no underflow below 1e-154
        // or overflow off huge offsets): the band below is exact for
        // every normal diagonal.
        let diag = (hi[0] - lo[0]).hypot(hi[1] - lo[1]).hypot(hi[2] - lo[2]);
        if !(diag >= f64::MIN_POSITIVE) || diag >= 1.0 {
            return 1.0;
        }
        // Exact 2^k from the IEEE exponent (libm-free): diag in
        // [2^e, 2^(e+1)) scales by 2^-e into [1, 2).
        let exp = ((diag.to_bits() >> 52) & 0x7ff) as i32;
        debug_assert!((1..1023).contains(&exp));
        if !(1..1023).contains(&exp) {
            return 1.0;
        }
        let scale = f64::from_bits(((2046 - exp) as u64) << 52);
        if max_abs > 0.0 && scale >= f64::MAX / max_abs {
            return 1.0;
        }
        scale
    }

    pub(crate) fn scale_positions(positions: &mut [Vector3], scale: f64) {
        for p in positions.iter_mut() {
            *p *= scale;
        }
    }

    /// Exact round-trip restore of the scaled inputs (plus the symmetry
    /// offset, which detection wrote in scaled space), so a failed run
    /// — or a second `remesh()` call — sees pristine inputs.
    pub(crate) fn restore_normalized_inputs(&mut self, normalize_scale: f64) {
        if normalize_scale == 1.0 {
            return;
        }
        let inv = 1.0 / normalize_scale;
        Self::scale_positions(&mut self.vertices, inv);
        for line in self
            .guide_polylines
            .iter_mut()
            .chain(self.sharp_polylines.iter_mut())
        {
            Self::scale_positions(line, inv);
        }
        self.symmetry_plane.offset *= inv;
    }
}
