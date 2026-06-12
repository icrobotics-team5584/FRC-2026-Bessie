#include <hal/HAL.h>

#include <filesystem>

#include "gtest/gtest.h"

namespace {

namespace fs = std::filesystem;

// Robot code loads files relative to the project root on desktop (e.g.
// SubVision reads src/main/deploy/2026-rebuilt.json and throws if missing),
// but gradle runs the test executable from a build output directory. Anchor
// the working directory at the project root by walking up to the directory
// containing build.gradle.
bool ChdirToProjectRoot(fs::path dir) {
  while (true) {
    if (fs::exists(dir / "build.gradle") && fs::exists(dir / "src/main/deploy")) {
      fs::current_path(dir);
      return true;
    }
    fs::path parent = dir.parent_path();
    if (parent.empty() || parent == dir) {
      return false;
    }
    dir = parent;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (!ChdirToProjectRoot(fs::current_path()) && argc > 0) {
    ChdirToProjectRoot(fs::absolute(fs::path(argv[0])).parent_path());
  }

  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
