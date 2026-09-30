add_executable(retopo main.cpp glb.cpp)

# OBJ input goes through AutoRemesher::loadObjPositionsAndTriangles
# (core/objreader.cpp, part of retopo_core).
# GLB input goes through cgltf (thirdparty/cgltf, header-only, MIT);
# CGLTF_IMPLEMENTATION lives in glb.cpp, the only TU including cgltf.h.
target_include_directories(retopo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/cgltf)
target_link_libraries(retopo PRIVATE retopo_core)
