add_executable(retopo main.cpp)

# tiny_obj_loader.h lives in thirdparty/; it is itself a
# header-only, dependency-free OBJ loader, so the CLI reuses it directly.
target_include_directories(retopo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/tinyobjloader)

target_link_libraries(retopo PRIVATE retopo_core)
