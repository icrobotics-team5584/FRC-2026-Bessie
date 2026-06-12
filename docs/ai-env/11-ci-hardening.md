# 11 — CI hardening for desktop tests

**Phase:** Fast-follow | **Size:** S | **Depends on:** [01](01-enable-tests.md)

## Goal

Make CI test execution deliberate instead of incidental. Since task 01,
`./gradlew build` (run by `.github/workflows/main.yml` in the
`wpilib/roborio-cross-ubuntu` container) depends on `check`, which now includes test
tasks — whatever happened on the first post-01 CI run was an accident of the container's
toolchain, not a decision.

## Context

- `.github/workflows/main.yml` — current build workflow (container build).
- Task 01's spec records what actually happened on its branch CI run — read it first.
- The test suite also declares `linuxathena` (roborio) test binaries which can never run
  on the host; if they break `check` in CI, mark them unbuildable in the test-suite spec:
  `if (it.targetPlatform.name == nativeUtils.wpi.platforms.roborio) { it.buildable = false }`.

## Steps

1. Decide and implement the CI test path:
   - Preferred: a dedicated job/step running the Linux desktop equivalent task
     (`runFrcUserProgramTestLinuxx86-64DebugGoogleTestExe` — confirm exact name with
     `./gradlew tasks --all` on Linux) so test results are first-class and visibly
     separated from the roborio cross-build.
   - The roborio build job should not silently depend on desktop tests passing twice.
2. Upload `build/test-logs/*.wpilog` as a CI artifact on test failure (and optionally on
   success) so an agent (or human) can run `tools/logcli.py` on CI failures locally.
3. Decide `-Werror` for test code (`src/test/cpp`) — currently CI `-Werror` covers only
   `frcUserProgram` binaries. Recommendation: yes, same flag block extended to the test
   suite binaries under `CI=true`, so agent-written tests meet the same bar.
4. Decide whether clang-format CI should also cover `src/test` (recommendation: yes —
   agents generate this code; cheap to keep formatted). clang-tidy can stay
   `src/main`-only for now (test code triggers many fixture-pattern false positives).
5. Document the final CI behavior in `docs/ai-env/README.md`.

## Acceptance criteria

- [ ] CI runs desktop tests as an explicit, named job/step; a failing test fails the PR.
- [ ] `.wpilog` artifacts downloadable from a failed run (demonstrated once).
- [ ] `-Werror`/format decisions implemented and recorded.
- [ ] Total CI wall-time increase measured and noted (<5 min target).
