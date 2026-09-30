# macOS LLVM toolchain: build with Homebrew LLVM instead of AppleClang.
# AppleClang has no C++ named-modules support, so the LLVM compiler is
# required for the modules conversion (and for newer C++23 library
# features). TBB/Eigen interop was verified bench-neutral.
#
# Usage:
#   brew install llvm
#   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/macos-llvm.cmake \
#     -DCMAKE_BUILD_TYPE=Release
# Overridable for packagers (Homebrew on Intel uses /usr/local):
# cmake -DRETOPO_LLVM_PREFIX=<llvm-prefix> ...
if(NOT DEFINED RETOPO_LLVM_PREFIX)
    set(RETOPO_LLVM_PREFIX /opt/homebrew/opt/llvm)
endif()
set(LLVM_PREFIX ${RETOPO_LLVM_PREFIX})
if(NOT EXISTS "${LLVM_PREFIX}/bin/clang++")
    message(FATAL_ERROR
        "Homebrew LLVM not found at ${LLVM_PREFIX} (run: brew install llvm)")
endif()
set(CMAKE_C_COMPILER "${LLVM_PREFIX}/bin/clang")
set(CMAKE_CXX_COMPILER "${LLVM_PREFIX}/bin/clang++")
# Brew clang does not inherit an SDK sysroot from CMake the way AppleClang
# does; without it the libc++ headers pick up the wrong C library headers.
execute_process(
    COMMAND xcrun --sdk macosx --show-sdk-path
    OUTPUT_VARIABLE CMAKE_OSX_SYSROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE)
