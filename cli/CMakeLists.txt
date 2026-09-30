add_executable(retopo main.cpp)

# tiny_obj_loader.h lives in the (Qt) app sources; it is itself a
# header-only, dependency-free OBJ loader, so the CLI reuses it directly.
target_include_directories(retopo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)

target_link_libraries(retopo PRIVATE retopo_core)
