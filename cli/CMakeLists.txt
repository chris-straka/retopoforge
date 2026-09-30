add_executable(retopo main.cpp)

# OBJ input goes through AutoRemesher::loadObjPositionsAndTriangles
# (core/objreader.cpp, part of retopo_core).
target_link_libraries(retopo PRIVATE retopo_core)
