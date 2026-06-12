# 08 — Root CLAUDE.md + closed-loop workflow

**Phase:** MVP | **Size:** S | **Depends on:** [06](06-skills-robot-game.md), [07](07-skills-sim-log.md) | **Track:** C (knowledge)

## Goal

A root `CLAUDE.md` that makes any fresh agent session in this repo immediately
productive: build/test commands, where knowledge lives, and the closed-loop procedure
as the default way of working on robot behavior.

## Content (keep it under ~80 lines — CLAUDE.md is loaded into every session)

1. **What this repo is:** FRC 2026 C++ robot (GradleRIO), team conventions
   (`.clang-format` enforced on `src/main` in CI; subsystems are singletons
   `SubX::GetInstance()`; command-based).
2. **Commands:**
   - Iteration tests: `./gradlew runFrcUserProgramTestWindowsx86-64DebugGoogleTestExe`
   - Single test: run the installed exe with `--gtest_filter` (see sim-testing skill)
   - Full check: `./gradlew check` | Sim GUI: `./gradlew simulateNative`
   - Log queries: `python tools/logcli.py ...` (see log-analysis skill)
3. **The closed loop** (the default procedure for any behavior change or bug fix):
   1. Run the relevant scenario(s) — or write one (sim-testing skill).
   2. Inspect the `.wpilog` in `build/test-logs/` (log-analysis skill).
   3. Diagnose; state the root cause before editing.
   4. Fix; re-run the scenario AND the auto regression suite.
   5. Report quantitative results against the criteria — never claim a fix without
      re-run evidence.
4. **Pointers:** `.claude/skills/` (robot-description, game-info, sim-testing,
   log-analysis), `docs/ai-env/` (architecture + task specs), `254-ai-env.md`
   (background concept).
5. **Hard rules:** don't bypass the harness with wall-clock sleeps; don't re-enable the
   WebServer in tests; auto names must match `RobotContainer.cpp` registration exactly;
   `.wpilog` files are never read raw — always via `tools/logcli.py`.

## Acceptance criteria

- [ ] A cold agent session prompted only with "run the auto regression suite and
      summarize results" succeeds using just the repo's guidance.
- [ ] CLAUDE.md stays under ~80 lines and defers detail to the skills.
