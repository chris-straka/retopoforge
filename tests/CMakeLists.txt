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
