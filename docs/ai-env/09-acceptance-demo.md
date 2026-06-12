# 09 — MVP acceptance demo

**Phase:** MVP gate | **Size:** M | **Depends on:** [04](04-auto-regression-suite.md), [05](05-wpilog-query-cli.md), [07](07-skills-sim-log.md), [08](08-claude-md.md)

## Goal

Prove the whole stack closes the loop, 254-style: an agent — given a single prompt —
runs every registered auto, reports a per-auto criteria table, then is handed a broken
robot and must diagnose from logs, fix, and verify, all without human help.

## Procedure

### Part 1 — Regression run
Fresh agent session, prompt: *"Run the autonomous regression suite and report results."*
Expected: agent uses the iteration gradle task, captures the per-auto markdown table,
relays it with commentary grounded in measured values.

### Part 2 — Bug injection + closed-loop fix
1. A human (or scripted patch) injects ONE bug on a scratch branch. Candidates, in
   order of preference:
   - **Organic:** make `SubDeploy::Zero()`'s missing sim short-circuit actually bite
     (e.g. remove the `WithDeadline` guard in one auto) — a real gap, found during
     environment design (`src/main/cpp/subsystems/SubDeploy.cpp:68-92`).
   - **Injected:** flip the hood angle offset sign, or break the shooter setpoint
     mapping in one auto — symptoms visible in logs as flywheel/hood never reaching
     target while the commands run.
2. Fresh agent session, prompt: *"The <auto name> auto is failing its regression test.
   Diagnose and fix it."* The agent must NOT be told what was injected.
3. Expected agent behavior (per CLAUDE.md closed-loop procedure): run the failing
   scenario → query the run's `.wpilog` with `tools/logcli.py` → identify the
   discrepancy (e.g. commanded vs measured) → locate root cause in code → fix → re-run
   the scenario AND the full regression suite → report before/after values.

### Part 3 — Record
Write `docs/ai-env/acceptance-demo.md` (replacing this section's TODO) containing: the
prompts used, the injected bug diff, the agent's diagnosis transcript highlights, actual
command outputs (tables before/after), and total wall time / iteration count.

## Acceptance criteria

- [ ] Part 1 and Part 2 each completed by the agent without mid-task human input.
- [ ] The fix is correct (reviewer-verified), not a test-weakening (tolerance loosening
      or assertion deletion fails the demo).
- [ ] The demo doc exists with real outputs pasted, reproducible by re-running.
- [ ] Lessons learned recorded: anything the agent fumbled becomes a follow-up issue
      (skill wording, missing helper, unclear failure message).

## Notes

This task is intentionally last in the MVP: it is the test of the *environment*, not of
the robot code. Failures here are environment bugs — fix the skills/harness/CLI and
re-run the demo.
