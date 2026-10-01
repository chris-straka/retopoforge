// TEMPORARY (deleted before push): dump Rust internals for case 11.
use retopo_core::singularity_simplifier::SingularitySimplifier;
use retopo_core::surface_mesh::SurfaceMesh;
use retopo_core::vector3::Vector3;

const FIXTURE: &str = include_str!("../../../tests/fixtures/singularitysimplifier_diff.txt");

#[test]
fn dump_case11() {
    let toks: Vec<String> = FIXTURE.split_ascii_whitespace().map(str::to_string).collect();
    let mut pos = 0;
    // find CASE 11
    while !(toks[pos] == "CASE" && toks[pos + 1] == "11") {
        pos += 1;
    }
    pos += 2; // id
    let nv: usize = toks[pos + 1].parse().unwrap();
    let nt: usize = toks[pos + 2].parse().unwrap();
    let max_pair: usize = toks[pos + 3].parse().unwrap();
    let sharp: f64 = toks[pos + 4].parse().unwrap();
    pos += 6;
    let mut positions = Vec::new();
    for _ in 0..nv {
        assert_eq!(toks[pos], "v");
        positions.push(Vector3::new(
            toks[pos + 1].parse().unwrap(),
            toks[pos + 2].parse().unwrap(),
            toks[pos + 3].parse().unwrap(),
        ));
        pos += 4;
    }
    let mut triangles = Vec::new();
    for _ in 0..nt {
        assert_eq!(toks[pos], "t");
        let k: usize = toks[pos + 1].parse().unwrap();
        let mut face = Vec::new();
        for j in 0..k {
            face.push(toks[pos + 2 + j].parse().unwrap());
        }
        triangles.push(face);
        pos += 2 + k;
    }
    let mesh = SurfaceMesh::new(&positions, &triangles);
    assert_eq!(toks[pos], "F");
    let nf: usize = toks[pos + 1].parse().unwrap();
    pos += 2;
    let mut field = Vec::new();
    for _ in 0..nf {
        field.push(Vector3::new(
            toks[pos].parse().unwrap(),
            toks[pos + 1].parse().unwrap(),
            toks[pos + 2].parse().unwrap(),
        ));
        pos += 3;
    }
    let mut simp = SingularitySimplifier::new(&mesh, &mut field);
    simp.set_maximum_pair_distance(max_pair);
    simp.set_sharp_edge_degrees(sharp);
    simp.simplify();
    eprintln!("R {} {} {}", simp.singularity_count_before(), simp.singularity_count_after(), simp.cancelled_pair_count());
    eprintln!("iterations: {:?}", simp.debug_iterations);
    eprintln!("init_angles[15]: {:#018x}", simp.debug_init_angles[15].to_bits());
    eprintln!("angles[15]: {:#018x}", simp.debug_angles()[15].to_bits());
    eprintln!("conn[45]: {:#018x}", simp.debug_connection()[45].to_bits());
    eprintln!("conn[46]: {:#018x}", simp.debug_connection()[46].to_bits());
    eprintln!("conn[47]: {:#018x}", simp.debug_connection()[47].to_bits());
    let (fu, fv) = simp.debug_frames();
    eprintln!("U15: {:#018x} {:#018x} {:#018x}", fu[15].x().to_bits(), fu[15].y().to_bits(), fu[15].z().to_bits());
    eprintln!("V15: {:#018x} {:#018x} {:#018x}", fv[15].x().to_bits(), fv[15].y().to_bits(), fv[15].z().to_bits());
    // all final angles vs... just print all
    for (f, a) in simp.debug_angles().iter().enumerate() {
        eprintln!("angle[{f}]: {a:#018x}");
    }
}
