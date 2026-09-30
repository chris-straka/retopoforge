class Retopoforge < Formula
  desc "Quad remesher: headless C++ engine + CLI + Blender extension"
  homepage "https://github.com/chris-straka/retopoforge"
  url "https://github.com/chris-straka/retopoforge/archive/refs/heads/main.tar.gz"
  version "0.2.0"
  license "MIT"

  depends_on "cmake" => :build
  depends_on "ninja" => :build
  depends_on "llvm" => :build
  depends_on "tbb"

  def install
    args = %W[
      -DCMAKE_TOOLCHAIN_FILE=#{buildpath}/cmake/macos-llvm.cmake
      -DRETOPO_LLVM_PREFIX=#{Formula["llvm"].opt_prefix}
      -DCMAKE_BUILD_TYPE=Release
    ]
    system "cmake", "-S", ".", "-B", "build", "-G", "Ninja", *args
    system "cmake", "--build", "build", "--target", "retopo"
    bin.install "build/cli/retopo"
  end

  test do
    # Cube smoke: remesh 12 triangles, expect a quad mesh out.
    (testpath/"cube.obj").write <<~EOS
      v -1 -1 -1
      v 1 -1 -1
      v 1 1 -1
      v -1 1 -1
      v -1 -1 1
      v 1 -1 1
      v 1 1 1
      v -1 1 1
      f 1 4 3
      f 1 3 2
      f 5 6 7
      f 5 7 8
      f 1 2 6
      f 1 6 5
      f 4 8 7
      f 4 7 3
      f 1 5 8
      f 1 8 4
      f 2 3 7
      f 2 7 6
    EOS
    system bin/"retopo", "--input", testpath/"cube.obj",
           "--output", testpath/"out.obj", "--target-quads", "200"
    assert_predicate testpath/"out.obj", :exist?
    assert_match(/^f /, (testpath/"out.obj").read)
  end
end
