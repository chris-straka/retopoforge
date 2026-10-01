// Build script for the rs-autoremesher lane: compiles the vendored
// meshoptimizer sources that `auto_remesher::decimate_if_too_dense` calls
// through FFI (`meshopt_generateVertexRemap`, `meshopt_remapIndexBuffer`,
// `meshopt_remapVertexBuffer`, `meshopt_simplifyWithAttributes`).
//
// Optimization is pinned to the CMake Release level on every cargo
// profile: the C++ oracle side builds Release, and cargo debug + release
// must both replay the identical decimation.
//
// Compiler choice mirrors cmake/macos-llvm.cmake: when `$CXX` is unset
// and the Homebrew LLVM exists, it compiles the FFI objects so both sides
// share codegen; otherwise cc's default detection applies.
fn main() {
    let core_dir = std::path::PathBuf::from(
        std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is always set by cargo"),
    );
    let meshopt_dir = core_dir.join("../../thirdparty/meshoptimizer/src");
    let simplifier = meshopt_dir.join("simplifier.cpp");
    let index_generator = meshopt_dir.join("indexgenerator.cpp");

    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("c++17")
        .opt_level(3)
        .warnings(false)
        .define("NDEBUG", None)
        .include(&meshopt_dir)
        .file(&simplifier)
        .file(&index_generator);
    if std::env::var("CXX").is_err() {
        let prefix = std::env::var("RETOPO_LLVM_PREFIX")
            .unwrap_or_else(|_| "/opt/homebrew/opt/llvm".to_string());
        let clang = std::path::PathBuf::from(&prefix).join("bin/clang++");
        if clang.exists() {
            build.compiler(clang);
        }
    }
    build.compile("meshopt");

    println!("cargo::rerun-if-changed=build.rs");
    println!("cargo::rerun-if-changed={}", simplifier.display());
    println!("cargo::rerun-if-changed={}", index_generator.display());
    println!(
        "cargo::rerun-if-changed={}",
        meshopt_dir.join("meshoptimizer.h").display()
    );
}
