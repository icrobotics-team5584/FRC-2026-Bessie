# 01 — Re-enable the GoogleTest suite

**Phase:** MVP | **Size:** S | **Depends on:** nothing | **Track:** A (harness)

> Status: implemented in the bootstrap PR (branch `AI-Tester-Env`) together with these
> docs. Kept as a spec for the record and for the outstanding CI verification step.

## Goal

`./gradlew runFrcUserProgramTestWindowsx86-64DebugGoogleTestExe` compiles and runs a
trivial GoogleTest suite on the desktop, proving the toolchain end-to-end. This is the
foundation every harness task builds on.

## Context

- The gtest entry point already exists: `src/test/cpp/main.cpp`
  (calls `HAL_Initialize(500, 0)` then `RUN_ALL_TESTS()`).
- The standard GradleRIO `GoogleTestTestSuiteSpec` block existed in `build.gradle`
  (formerly commented out at lines 92–110). The `google-test-test-suite` plugin and
  desktop support were already enabled.
- `Robot.cpp` guards its `main()` with `#ifndef RUNNING_FRC_TESTS` — the test binary must
  define `RUNNING_FRC_TESTS` (the test suite recompiles all of `src/main/cpp` with the
  test suite's compiler flags, which is also why guarded production code works).
- `SubVision` loads `src/main/deploy/2026-rebuilt.json` relative to the working directory
  on desktop and **throws** if missing (`src/main/include/subsystems/SubVision.h:64-65`),
  so test executables must run from the project root.

## Steps

1. Uncomment the `testSuites` block in `build.gradle`; inside the spec add
   `it.binaries.all { cppCompiler.define 'RUNNING_FRC_TESTS' }`.
2. Guard `wpi::WebServer::GetInstance().Start(5800, ...)` in `Robot.cpp` with
   `#ifndef RUNNING_FRC_TESTS` (port binds don't belong in headless test runs).
3. Anchor the working directory in the test entry point (`src/test/cpp/main.cpp`):
   before `HAL_Initialize`, walk up from the current/executable directory to the first
   directory containing `build.gradle` + `src/main/deploy` and `chdir` there.
   (Gradle's `RunTestExecutable` task runs the exe from its `outputDir` and exposes no
   usable `workingDir` property — configuring the task type does **not** work. Anchoring
   in `main()` also makes direct exe invocation work from any directory.)
4. Add `src/test/cpp/smoke/BuildSanityTest.cpp` with three tests: a trivial
   `EXPECT_TRUE(true)`, a HAL sim time-step check (`frc::sim::PauseTiming()` /
   `StepTiming(20_ms)` moves `frc::Timer::GetFPGATimestamp()` by exactly 0.020 s), and a
   cwd assertion (`std::filesystem::exists("src/main/deploy/2026-rebuilt.json")`),
   which guards the `main.cpp` anchoring from step 3.
5. Discover task names with `./gradlew tasks --all`. Discovered (2026.2.1):
   - **Iteration command (use this):** `runFrcUserProgramTestWindowsx86-64DebugGoogleTestExe`
   - Filter a single suite/test: run the built exe directly (it anchors its own cwd, so
     this works from any directory — verified):
     `build\install\frcUserProgramTest\windowsx86-64\debug\lib\frcUserProgramTest.exe --gtest_filter=...`
   - Full verification: `./gradlew check` (all variants — slow; note it also includes
     `linuxathena` test binaries which cannot run on the host; if `check` fails on
     athena test tasks, mark those binaries unbuildable in the test suite spec:
     `if (it.targetPlatform.name == nativeUtils.wpi.platforms.roborio) { it.buildable = false }`).
6. Open a PR and observe CI. Note: `.github/workflows/main.yml` triggers only on
   push/PR **to main**, so the test execution in CI (`./gradlew build` → `check` in the
   `wpilib/roborio-cross-ubuntu:2025-24.04` container on a self-hosted runner) first
   happens on the PR. Record the outcome here; task 11 makes CI behavior deliberate.

## Acceptance criteria

- [ ] `./gradlew runFrcUserProgramTestWindowsx86-64DebugGoogleTestExe` exits 0 with all
      smoke tests passing, run from a clean checkout.
- [ ] The cwd test fails with a clear message if `workingDir` is removed (verified once
      manually).
- [ ] One CI run on a branch documents whether the container builds + runs tests
      (paste result into this file).
