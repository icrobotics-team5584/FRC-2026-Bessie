# 12 — Algorithm-level replay runner + replay skill

**Phase:** Phase 2 | **Size:** L | **Depends on:** [05](05-wpilog-query-cli.md)

## Goal

254's "threshold tuning across every match log" capability: re-run a specific
*algorithm* (not the whole robot) against logged real-match data and produce a results
table. Example: tune a tip-detection threshold against all logs from an event and report
true/false positives per match.

**Scope guard (decided):** algorithm-level only. No whole-robot deterministic replay, no
IO-layer refactor of subsystems, no AdvantageKit clone. If a use case seems to need
interacting subsystems replayed together, bring it back to the team — don't grow this
task into one.

## Design

Two halves:

1. **Pure-function extraction (C++):** key decision logic must be callable with plain
   inputs (no hardware, no singleton reads inside). Candidates already close to pure:
   - `ShotPlanner::CalculateShotTarget(pose)` (`src/main/include/utilities/ShotPlanner.h`)
     — already takes a pose, used in `Robot.cpp:36-37`.
   - `ShiftHandler` shift/timing math (`src/main/cpp/utilities/ShiftHandler.cpp`) —
     extract the pure parts.
   - `SubVision::IsEstimateUsable` / vision-acceptance heuristics — extract the
     predicate from the subsystem.
   Refactors must keep robot behavior identical (the regression suite from task 04 is
   the guard). Pattern: free functions or static methods in `utilities/`, with the
   subsystem calling them.
2. **Replay runner:** reads a `.wpilog`, maps topics → function inputs, pumps every
   sample (or a time-window) through the function, writes a results CSV.
   - Default implementation: **Python**, reusing `tools/loglib/` from task 05, invoking
     the algorithm via one of:
     a. re-implementation in Python only when trivially small (document drift risk), or
     b. a tiny C++ CLI (`replaycli`) added as a second desktop binary that links the
        pure functions and reads CSV from stdin → preferred for nontrivial algorithms,
        zero drift.
   - Config per replay: YAML/JSON or argparse flags naming the wpilog(s), topic→input
     mapping, function/variant, parameter sweep ranges (e.g. threshold 5°..20° step 1°).
   - Output: one CSV row per (log, parameter set) with metrics; a summary table to
     stdout.

## Steps

1. Extract 1–2 pure functions (start with the vision-acceptance predicate or whatever
   the team currently wants to tune — ask) with regression-suite verification.
2. Build the runner (variant b recommended) + one real tuning study over the logs in
   `logs/` as the worked example.
3. `.claude/skills/replay/SKILL.md`: when to use replay vs. sim re-run (replay = tuning
   thresholds/algorithms against historical data; sim = behavior/sequencing changes),
   how to add a new algorithm to the runner, copy-paste example of the worked study.
4. Cross-link from CLAUDE.md and the log-analysis skill.

## Acceptance criteria

- [ ] One completed parameter-sweep study over ≥2 real logs with a results table,
      committed as the skill's example.
- [ ] Regression suite green before/after the purity refactors.
- [ ] Adding a second algorithm to the runner requires no changes to loglib/logcli.
- [ ] Replay skill explains the algorithm-only scope and when NOT to reach for replay.
