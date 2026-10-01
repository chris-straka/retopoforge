# Unit tests for the pure/std-only core units.
# Plain assert-style mains, no third-party framework.
# NOTE: each binary uses a CHECK macro instead of <cassert> assert(),
# which compiles to a no-op in Release (NDEBUG) builds.

function(retopo_add_test name source)
    add_executable(${name} ${source})
    # Test sources import retopo.core modules, so they need module
    # dependency scanning (BMI flags + build ordering after retopo_core).
    set_target_properties(${name} PROPERTIES CXX_SCAN_FOR_MODULES ON)
    target_link_libraries(${name} PRIVATE retopo_core)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

retopo_add_test(test_double test_double.cpp)
retopo_add_test(test_vector2 test_vector2.cpp)
retopo_add_test(test_vector3 test_vector3.cpp)
retopo_add_test(test_positionkey test_positionkey.cpp)
retopo_add_test(test_objreader test_objreader.cpp)
retopo_add_test(test_surfacemesh test_surfacemesh.cpp)
retopo_add_test(test_weld test_weld.cpp)
retopo_add_test(test_constrainedleastsquares test_constrainedleastsquares.cpp)
retopo_add_test(test_mixedintegerleastsquares test_mixedintegerleastsquares.cpp)
retopo_add_test(test_symmetry test_symmetry.cpp)
set_tests_properties(test_symmetry PROPERTIES TIMEOUT 600)
retopo_add_test(test_guides test_guides.cpp)
set_tests_properties(test_guides PROPERTIES TIMEOUT 600)
retopo_add_test(test_density test_density.cpp)
set_tests_properties(test_density PROPERTIES TIMEOUT 600)
retopo_add_test(test_sharp test_sharp.cpp)
set_tests_properties(test_sharp PROPERTIES TIMEOUT 600)

# CLI round-trip: runs the built retopo binary on bench/models/armadillo.obj.
# Needs the model file (bench/fetch_models.sh, gitignored) and the retopo
# target built first. Timeout matches bench/run.py TIMEOUT_SECONDS.
retopo_add_test(test_cli_roundtrip test_cli_roundtrip.cpp)
target_compile_definitions(test_cli_roundtrip PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
    "RETOPO_MODEL_ARMADDILLO=\"${CMAKE_SOURCE_DIR}/bench/models/armadillo.obj\""
)
add_dependencies(test_cli_roundtrip retopo)
set_tests_properties(test_cli_roundtrip PROPERTIES TIMEOUT 600)

# CLI multi-mode: --lods chains + folder batch mode on bench/models/fandisk.obj.
retopo_add_test(test_cli_multimode test_cli_multimode.cpp)
target_compile_definitions(test_cli_multimode PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
    "RETOPO_MODEL_FANDISK=\"${CMAKE_SOURCE_DIR}/bench/models/fandisk.obj\""
)
add_dependencies(test_cli_multimode retopo)
set_tests_properties(test_cli_multimode PROPERTIES TIMEOUT 600)

# CLI --quiet: same binary + model, asserts the silence contract.
retopo_add_test(test_cli_quiet test_cli_quiet.cpp)
target_compile_definitions(test_cli_quiet PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
    "RETOPO_MODEL_ARMADDILLO=\"${CMAKE_SOURCE_DIR}/bench/models/armadillo.obj\""
)
add_dependencies(test_cli_quiet retopo)
set_tests_properties(test_cli_quiet PROPERTIES TIMEOUT 600)

# CLI GLB: round-trips, batch/LOD emission, truncated-input failure.
retopo_add_test(test_cli_glb test_cli_glb.cpp)
target_compile_definitions(test_cli_glb PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
    "RETOPO_MODEL_TETRA=\"${CMAKE_SOURCE_DIR}/tests/fixtures/tetra.glb\""
    "RETOPO_MODEL_FANDISK=\"${CMAKE_SOURCE_DIR}/bench/models/fandisk.obj\""
)
add_dependencies(test_cli_glb retopo)
set_tests_properties(test_cli_glb PROPERTIES TIMEOUT 600)

# CLI --symmetry: flag wiring end to end (help text, bogus value, fallback
# on an asymmetric input, mirror-symmetric output on a cube).
retopo_add_test(test_cli_symmetry test_cli_symmetry.cpp)
target_compile_definitions(test_cli_symmetry PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
    "RETOPO_MODEL_FANDISK=\"${CMAKE_SOURCE_DIR}/bench/models/fandisk.obj\""
)
add_dependencies(test_cli_symmetry retopo)
set_tests_properties(test_cli_symmetry PROPERTIES TIMEOUT 600)

# CLI --guides: guide-file wiring end to end (help text, missing/bad
# files, batch rejection, guided output differs from unguided).
retopo_add_test(test_cli_guides test_cli_guides.cpp)
target_compile_definitions(test_cli_guides PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
)
add_dependencies(test_cli_guides retopo)
set_tests_properties(test_cli_guides PROPERTIES TIMEOUT 600)

# CLI --uvs: on/off contract for remeshed UV emission (OBJ vt + v/vt
# corners, GLB TEXCOORD_0, geometry identical with the flag on or off).
retopo_add_test(test_cli_uvs test_cli_uvs.cpp)
target_compile_definitions(test_cli_uvs PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
    "RETOPO_MODEL_FANDISK=\"${CMAKE_SOURCE_DIR}/bench/models/fandisk.obj\""
)
add_dependencies(test_cli_uvs retopo)
set_tests_properties(test_cli_uvs PROPERTIES TIMEOUT 600)

# CLI --density: mask-file wiring end to end (help text, missing/bad
# files, count mismatch, batch rejection, masked output differs).
retopo_add_test(test_cli_density test_cli_density.cpp)
target_compile_definitions(test_cli_density PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
)
add_dependencies(test_cli_density retopo)
set_tests_properties(test_cli_density PROPERTIES TIMEOUT 600)

# Input validation: loader index rejection, weld NaN drops + stats, engine
# remesh() entry validation. No binary, no models, no fixtures needed.
retopo_add_test(test_input_validation test_input_validation.cpp)

# CLI nasty-corpus: every tests/fixtures/nasty-*.obj class plus two
# procedurally generated large inputs through the built binary (exit
# codes, output validity, drop accounting). Hermetic: committed fixtures
# + temp-dir inputs only, no bench/models download required.
retopo_add_test(test_cli_corpus test_cli_corpus.cpp)
target_compile_definitions(test_cli_corpus PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
    "RETOPO_FIXTURES=\"${CMAKE_SOURCE_DIR}/tests/fixtures\""
)
add_dependencies(test_cli_corpus retopo)
set_tests_properties(test_cli_corpus PROPERTIES TIMEOUT 600)

# CLI --features: sharp-file wiring end to end (help text, missing/bad
# files, batch rejection, marked output differs, --lods wiring).
retopo_add_test(test_cli_features test_cli_features.cpp)
target_compile_definitions(test_cli_features PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
)
add_dependencies(test_cli_features retopo)
set_tests_properties(test_cli_features PROPERTIES TIMEOUT 600)

# CLI UV atlas: multi-island --uvs on packs island UV ranges into one shared
# 0..1 atlas (disjoint boxes, geometry identical to --uvs off); single-island
# output keeps the exact per-island 0..1 normalization.
retopo_add_test(test_cli_uv_atlas test_cli_uv_atlas.cpp)
target_compile_definitions(test_cli_uv_atlas PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
    "RETOPO_MODEL_FANDISK=\"${CMAKE_SOURCE_DIR}/bench/models/fandisk.obj\""
)
add_dependencies(test_cli_uv_atlas retopo)
set_tests_properties(test_cli_uv_atlas PROPERTIES TIMEOUT 600)

# Differential oracle dump for the vector2/vector3 port (wave 2
# foundation). Build-only helper: run it and redirect stdout to
# tests/fixtures/vector_diff.txt for rust/core/tests/vector_diff.rs.
add_executable(vector_diff_dump vector_diff_dump.cpp)
set_target_properties(vector_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(vector_diff_dump PRIVATE retopo_core)

# Differential oracle dump for the Rust solvers calibration experiment.
# Build-only helper (not a ctest test): run it and redirect stdout to
# tests/fixtures/solver_diff.txt for rust/solvers/tests/differential.rs.
add_executable(solver_diff_dump solver_diff_dump.cpp)
set_target_properties(solver_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(solver_diff_dump PRIVATE retopo_core)

# Differential oracle dump for the double_utils + progress port (wave 1).
# Build-only helper (not a ctest test): run it and redirect stdout to
# tests/fixtures/double_progress_diff.txt for
# rust/core/tests/double_progress_diff.rs.
add_executable(double_progress_diff_dump double_progress_diff_dump.cpp)
set_target_properties(double_progress_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(double_progress_diff_dump PRIVATE retopo_core)
# Differential oracle dump for the meshseparator Rust port (wave 1).
# Build-only helper (not a ctest test): run it and redirect stdout to
# tests/fixtures/meshsep_diff.txt for rust/core/tests/mesh_separator_diff.rs.
add_executable(meshsep_diff_dump meshsep_diff_dump.cpp)
set_target_properties(meshsep_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(meshsep_diff_dump PRIVATE retopo_core)
# Differential oracle dump for the obj_reader Rust port (rs-objreader lane).
# Build-only helper (not a ctest test): run it with the fixtures dir and
# redirect stdout to tests/fixtures/objreader_diff.txt for
# rust/core/tests/obj_reader_diff.rs.
add_executable(objreader_diff_dump objreader_diff_dump.cpp)
set_target_properties(objreader_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(objreader_diff_dump PRIVATE retopo_core)
# Differential oracle dump for the position_key Rust port (rs-positionkey lane).
# Build-only helper (not a ctest test): run it and redirect stdout to
# tests/fixtures/positionkey_diff.txt for
# rust/core/tests/position_key_diff.rs.
add_executable(positionkey_diff_dump positionkey_diff_dump.cpp)
set_target_properties(positionkey_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(positionkey_diff_dump PRIVATE retopo_core)
# Differential oracle dump for the surface_mesh Rust port (rs-surfacemesh lane).
# Build-only helper (not a ctest test): run it and redirect stdout to
# tests/fixtures/surfacemesh_diff.txt for rust/core/tests/surface_mesh_diff.rs.
add_executable(surfacemesh_diff_dump surfacemesh_diff_dump.cpp)
set_target_properties(surfacemesh_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(surfacemesh_diff_dump PRIVATE retopo_core)
# Differential oracle dump for the symmetry Rust port (rs-symmetry lane).
# Build-only helper (not a ctest test): build Release and redirect stdout
# to tests/fixtures/symmetry_diff.txt for rust/core/tests/symmetry_diff.rs.
add_executable(symmetry_diff_dump symmetry_diff_dump.cpp)
set_target_properties(symmetry_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(symmetry_diff_dump PRIVATE retopo_core)
# Differential oracle dump for the quad_parameterizer Rust port
# (rs-quadparameterizer lane). Build-only helper (not a ctest test): run it
# and redirect stdout to tests/fixtures/quadparam_diff.txt for
# rust/core/tests/quad_parameterizer_diff.rs.
add_executable(quadparam_diff_dump quadparam_diff_dump.cpp)
set_target_properties(quadparam_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(quadparam_diff_dump PRIVATE retopo_core)

# Differential oracle dump for the isotropic_remesher Rust port
# (rs-isotropicremesher lane). Build-only helper (not a ctest test): run it
# and redirect stdout to tests/fixtures/isoremesh_diff.txt for
# rust/core/tests/isotropic_remesher_diff.rs.
add_executable(isoremesh_diff_dump isoremesh_diff_dump.cpp)
set_target_properties(isoremesh_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(isoremesh_diff_dump PRIVATE retopo_core)

# Engine follow-ups: per-island output accounting (engine contract + CLI
# island reporting over a two-box mesh).
retopo_add_test(test_island_stats test_island_stats.cpp)
target_compile_definitions(test_island_stats PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
)
add_dependencies(test_island_stats retopo)
set_tests_properties(test_island_stats PROPERTIES TIMEOUT 600)

# Pole-cap manifold: remeshed UV sphere must be edge-manifold (zero
# boundary edges, zero non-manifold edges, zero duplicate faces).
# Hermetic: committed fixture only, no bench/models download required.
retopo_add_test(test_pole_manifold test_pole_manifold.cpp)
target_compile_definitions(test_pole_manifold PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
    "RETOPO_FIXTURE_SPHERE=\"${CMAKE_SOURCE_DIR}/tests/fixtures/sphere-pole.obj\""
)
add_dependencies(test_pole_manifold retopo)
set_tests_properties(test_pole_manifold PROPERTIES TIMEOUT 600)

# Differential oracle dump for the density Rust port (rs-density lane).
# Build-only helper (not a ctest test): run it and redirect stdout to
# tests/fixtures/density_diff.txt for rust/core/tests/density_diff.rs.
add_executable(density_diff_dump density_diff_dump.cpp)
set_target_properties(density_diff_dump PROPERTIES CXX_SCAN_FOR_MODULES ON)
target_link_libraries(density_diff_dump PRIVATE retopo_core)

# Engine follow-ups: --quiet silences engine-owned stderr (progress and
# phase-report lines) on both streams while errors still print.
retopo_add_test(test_cli_engine_quiet test_cli_engine_quiet.cpp)
target_compile_definitions(test_cli_engine_quiet PRIVATE
    "RETOPO_BINARY=\"$<TARGET_FILE:retopo>\""
    "RETOPO_MODEL_ARMADDILLO=\"${CMAKE_SOURCE_DIR}/bench/models/armadillo.obj\""
    "RETOPO_MODEL_FANDISK=\"${CMAKE_SOURCE_DIR}/bench/models/fandisk.obj\""
)
add_dependencies(test_cli_engine_quiet retopo)
set_tests_properties(test_cli_engine_quiet PROPERTIES TIMEOUT 600)

