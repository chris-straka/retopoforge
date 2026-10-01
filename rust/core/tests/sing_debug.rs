// TEMPORARY diagnostic (deleted before push): reports per-case max FO
// divergence instead of failing fast.
use retopo_core::singularity_simplifier::SingularitySimplifier;
use retopo_core::surface_mesh::SurfaceMesh;
use retopo_core::vector3::Vector3;

const FIXTURE: &str = include_str!("../../../tests/fixtures/singularitysimplifier_diff.txt");

struct Cur {
    toks: Vec<String>,
    pos: usize,
}

impl Cur {
    fn new(s: &str) -> Self {
        Self {
            toks: s.split_ascii_whitespace().map(str::to_string).collect(),
            pos: 0,
        }
    }
    fn word(&mut self) -> String {
        let w = self.toks[self.pos].clone();
        self.pos += 1;
        w
    }
    fn expect(&mut self, tag: &str) {
        assert_eq!(self.word(), tag);
    }
    fn uint(&mut self) -> usize {
        self.word().parse().expect("bad uint")
    }
    fn num(&mut self) -> f64 {
        self.word().parse().expect("bad f64")
    }
    fn int(&mut self) -> i32 {
        self.word().parse().expect("bad i32")
    }
}

fn ulp_diff(a: f64, b: f64) -> u64 {
    if a.to_bits() == b.to_bits() {
        return 0;
    }
    if a.is_nan() || b.is_nan() || a.signum() != b.signum() {
        return u64::MAX;
    }
    a.to_bits().abs_diff(b.to_bits())
}

#[test]
fn diagnose_divergence() {
    let mut c = Cur::new(FIXTURE);
    c.expect("SINGSIMP1");
    let mut ndiv = 0;
    while c.pos < c.toks.len() {
        let tag = c.word();
        if tag == "T" {
            for _ in 0..7 {
                c.word();
            }
            continue;
        }
        assert_eq!(tag, "CASE");
        let id = c.uint();
        let nv = c.uint();
        let nt = c.uint();
        let max_pair = c.uint();
        let sharp_deg: f64 = c.num();
        let class = c.word();
        let mut positions = Vec::with_capacity(nv);
        for _ in 0..nv {
            c.expect("v");
            positions.push(Vector3::new(c.num(), c.num(), c.num()));
        }
        let mut triangles = Vec::with_capacity(nt);
        for _ in 0..nt {
            c.expect("t");
            let k = c.uint();
            let mut face = Vec::with_capacity(k);
            for _ in 0..k {
                face.push(c.uint());
            }
            triangles.push(face);
        }
        let mesh = SurfaceMesh::new(&positions, &triangles);
        c.expect("F");
        let nf = c.uint();
        let mut field = Vec::with_capacity(nf);
        for _ in 0..nf {
            field.push(Vector3::new(c.num(), c.num(), c.num()));
        }
        let (charges_before, before, after, cancelled, charges_after) = {
            let mut simp = SingularitySimplifier::new(&mesh, &mut field);
            simp.set_maximum_pair_distance(max_pair);
            simp.set_sharp_edge_degrees(sharp_deg);
            let cb = simp.vertex_charges();
            simp.simplify();
            (
                cb,
                simp.singularity_count_before(),
                simp.singularity_count_after(),
                simp.cancelled_pair_count(),
                simp.vertex_charges(),
            )
        };
        c.expect("R");
        let (e_before, e_after, e_cancelled) = (c.uint(), c.uint(), c.uint());
        c.expect("CHI");
        assert_eq!(c.uint(), nv);
        let mut chi_bad = 0;
        for &charge in &charges_before {
            if charge != c.int() {
                chi_bad += 1;
            }
        }
        let counts_ok = before == e_before
            && (class == "S" || (after == e_after && cancelled == e_cancelled));
        c.expect("CH");
        assert_eq!(c.uint(), nv);
        let mut ch_bad = 0;
        for &charge in &charges_after {
            if class == "E" && charge != c.int() {
                ch_bad += 1;
            } else if class == "S" {
                c.int();
            }
        }
        c.expect("FO");
        assert_eq!(c.uint(), nf);
        let mut max_ulp = 0u64;
        let mut wrong = Vec::new();
        for (f, vec) in field.iter().enumerate() {
            for (axis, got) in [vec.x(), vec.y(), vec.z()].iter().enumerate() {
                let expect = c.num();
                let d = ulp_diff(*got, expect);
                if d > 0 && wrong.len() < 8 {
                    wrong.push((f, axis, got.to_bits(), expect.to_bits()));
                }
                max_ulp = max_ulp.max(d);
            }
        }
        let n_wrong = wrong.len();
        if class == "E" && (max_ulp > 0 || ch_bad > 0 || chi_bad > 0 || !counts_ok) {
            ndiv += 1;
            eprintln!("case {id}: R=({before},{after},{cancelled}) wrong={wrong:?}");
        }
    }
    eprintln!("divergent exact cases: {ndiv}");
}
