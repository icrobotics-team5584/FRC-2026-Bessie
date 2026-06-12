# 07 — Skills: sim-testing + log-analysis

**Phase:** MVP | **Size:** S | **Depends on:** [02](02-simscenario-fixture.md), [05](05-wpilog-query-cli.md) | **Track:** C (knowledge)

## Goal

Two procedure skills that teach an agent the closed-loop mechanics: how to write/run a
sim scenario, and how to interrogate the resulting logs. Every example must be
copy-paste-runnable against the repo as it exists when this task lands.

## `.claude/skills/sim-testing/SKILL.md`

Frontmatter description: use when writing, running, or debugging simulation test
scenarios for the robot.

Must cover:
- **The iteration command** (fast, single-variant):
  `./gradlew runFrcUserProgramTestWindowsx86-64DebugGoogleTestExe`
  (≈15 s incremental after a test-only change; minutes after touching `src/main`).
- Running a single test: invoke the installed exe directly with
  `--gtest_filter` (it chdirs to the project root itself):
  `build\install\frcUserProgramTest\windowsx86-64\debug\lib\frcUserProgramTest.exe --gtest_filter='AutoRegression*'`
  (works from any directory — verified even from outside the repo)
- Full verification: `./gradlew check` (slow, all variants) — final step only.
- How to write a scenario: `SimScenario` fixture walkthrough (SetUp/reset semantics,
  `SetStartPose` before enabling, `RunAuto` names must match
  `RobotContainer.cpp` registration exactly, direct `Schedule()` of commands preferred
  over joystick emulation, `AddPerStepObserver` for sampling).
- The measured sim acceleration limit from task 02 (`kMaxSimAcceleration`) and why it
  exists (Phoenix6 status signals refresh on a wall-clock thread).
- Where logs land (`build/test-logs/`), the `Test/<name>` marker convention, and the
  singleton/state-reset rules (one robot per process; suspect state bleed → re-run with
  `--gtest_filter` to isolate).
- Skeleton example of a complete new scenario file that compiles.

## `.claude/skills/log-analysis/SKILL.md`

Frontmatter description: use when inspecting robot/sim `.wpilog` files to diagnose
behavior — match logs in `logs/`, test logs in `build/test-logs/`.

Must cover:
- The survey-then-drill workflow: always `--summary` first, then narrow topic globs +
  time windows; never dump a whole log.
- Copy-paste examples for each CLI mode (`--summary`, topic+window CSV, `--test`
  marker window, `--changes`), run against a real checked-in or freshly generated log.
- Topic-name conventions (NT capture prefix `NT:/SmartDashboard/...`; per-subsystem
  prefixes — cross-link to the robot-description skill for the full topic map).
- Diagnostic recipes: "did the shooter reach setpoint" (compare target vs measured
  velocity topics), "what command was running at time T" (`--changes` on scheduler
  topics), "did the robot follow its path" (pose topics vs expectation).

## Acceptance criteria

- [ ] Every command in both skills executed verbatim during review, outputs sane.
- [ ] sim-testing documents the measured acceleration factor (or links the constant).
- [ ] Cross-links: sim-testing ↔ log-analysis ↔ robot-description.
- [ ] `docs/ai-env/README.md` links both skills.
