# 04 — All-autos regression suite + results table

**Phase:** MVP | **Size:** M | **Depends on:** [03](03-runauto-first-scenario.md) | **Track:** A (harness)

## Goal

One parameterized scenario runs **every** registered autonomous routine with per-auto
mechanism-level criteria and prints a per-auto results table. This is the regression
surface the agent re-runs after every change, and the core of the MVP acceptance demo.

## Context — read first

- `docs/ai-env/README.md` → "Auto selection" for the five registered auto names
  (re-read `src/main/cpp/RobotContainer.cpp:38-46` in case they changed).
- `src/main/cpp/commands/AutonCommands.cpp` — waypoints and start poses live here
  (`DriveToPose` targets around lines 264-410; tolerances 15–50 cm are already encoded
  in the autos themselves).
- Task 03's `RunAuto` and observer machinery.

## Steps

1. `src/test/cpp/scenarios/AutoRegressionTest.cpp` using `TEST_P` over the auto names
   (`::testing::ValuesIn` on a list that should be read from the chooser if practical,
   so new autos are picked up automatically — otherwise a static list with a guard test
   asserting the chooser contains exactly the listed names).
2. Per-auto criteria, sampled via per-step observers:
   - **Pose vs. final waypoint:** robot pose at auto completion within tolerance of the
     auto's final `DriveToPose` target (use the tolerances from the auto definitions;
     start poses from the comments in `AutonCommands.cpp`).
   - **Shooter:** flywheel reached commanded setpoint during each shooting phase.
   - **Sequencing:** the auto command completed before the 15 s deadline (or document
     expected timeout for autos designed to run out the clock).
   - **Current draw:** peak and integrated total simulated current under bounds (start
     loose, e.g. 2× observed baseline; values are visible in the existing
     `Logger::Log` current topics).
3. Structure the criteria record as a struct with one row per auto so a future
   "pieces scored" column can be added without reshaping (task 10).
4. Print a markdown results table to stdout at suite teardown — gtest stdout is the
   agent's primary results surface. Columns: auto name, each criterion (pass/fail +
   measured value), wall time.
5. Known-failing autos: assert with `EXPECT_NONFATAL_FAILURE`-style documentation or an
   explicit expected-failure list in the test file with a comment per entry — never
   silently skip.

## Acceptance criteria

- [ ] All five autos run in one suite invocation; table printed with measured values.
- [ ] `--gtest_filter='*LeftTrench*'` runs a single auto (per-process isolation escape
      hatch works).
- [ ] Suite is deterministic across 3 consecutive full runs (same pass/fail and values
      within noise).
- [ ] A criteria failure message contains the measured value, the bound, and the
      `.wpilog` path for follow-up queries.

## Notes

- Expect total suite wall time ≈ (5 autos × 15 s sim) ÷ acceleration factor from
  task 02; keep it under ~2 min so the agent loop stays tight.
- If trench autos fail for path-following reasons in stepped time, investigate before
  loosening tolerances — that's exactly the class of bug this suite exists to surface.
