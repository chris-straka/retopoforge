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
