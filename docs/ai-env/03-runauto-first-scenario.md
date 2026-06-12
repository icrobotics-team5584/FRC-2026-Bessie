# 03 — RunAuto + first auto scenario (ShootAndStay)

**Phase:** MVP | **Size:** M | **Depends on:** [02](02-simscenario-fixture.md) | **Track:** A (harness)

## Goal

`SimScenario::RunAuto("ShootAndStay", 15_s)` selects and runs a real autonomous routine
end-to-end in the headless harness, with mechanism-level assertions proving it did what
it should. This is the first complete "agent runs an auto and judges it" capability.

## Context — read first

- `docs/ai-env/README.md` → "Auto selection" (chooser topic, exact auto names,
  weak_ptr.lock() null-check requirement).
- `src/main/cpp/RobotContainer.cpp:38-48` — auto registration + chooser publication.
- `src/main/cpp/utilities/AutonHelper.cpp` — `GetChosenAuton()` semantics.
- `src/main/cpp/Robot.cpp:59-69` — what `AutonomousInit` does (the behavior `RunAuto`
  triggers via the fixture's mode dispatch — do NOT bypass it by scheduling the command
  yourself unless the fixture's mode dispatch doesn't already call `AutonomousInit`).
- `src/main/cpp/commands/AutonCommands.cpp` — auto definitions. Start poses are
  documented in comments (e.g. left trench start: X 3.58 m, Y 7.47 m, heading 270°,
  around line 267). `ShootAndStay` is the simplest: it shoots from its starting spot.

## Steps

1. Implement `RunAuto(name, maxTime, until)` on the fixture:
   - Write `name` to the NT string topic `/SmartDashboard/CHOSEN AUTON/selected`
     (`nt::NetworkTableInstance::GetDefault()`), then run 1–2 fixture loop iterations so
     `frc::SmartDashboard::UpdateValues()` lets the chooser listener fire.
   - Verify selection took: `RobotContainer::GetAutonomousCommand()` (or the equivalent
     path through the fixture's `AutonomousInit` dispatch) must be non-null —
     `ASSERT`/`FAIL` with the auto name in the message if not (typos in names with
     spaces/parens are the expected failure mode).
   - Set DS to autonomous + enabled, `NotifyNewData()`, then `AdvanceTime` in 20 ms
     steps until `until()` returns true, the scheduled auto command finishes
     (`frc2::CommandScheduler::GetInstance().IsScheduled(...)` goes false), or `maxTime`
     elapses. Return or record which exit condition fired.
2. Write `src/test/cpp/scenarios/AutoShootAndStayTest.cpp`:
   - `SetStartPose` to the auto's documented start pose **before** enabling.
   - `RunAuto("ShootAndStay", 15_s)`.
   - Assert mechanism truths: flywheel reached its commanded setpoint during the
     shooting window (track via a per-step observer sampling
     `SubShooter::GetInstance()` velocity vs target), the hood completed zeroing
     (sim-aware short-circuit exists, `SubHood.cpp:98`), and the auto command completed
     before the deadline.
3. Confirm the run's `.wpilog` in `build/test-logs/` contains a queryable shooter
   velocity rise (topic names come from `Logger::Log` call sites in
   `src/main/cpp/subsystems/SubShooter.cpp`).

## Acceptance criteria

- [ ] `AutoShootAndStayTest` passes and is deterministic across 3 consecutive runs.
- [ ] `RunAuto` with a misspelled name fails with a message that includes the bad name
      and the list of registered autos (read them from the chooser if practical).
- [ ] The produced `.wpilog` shows the shooter velocity rise (verified with the task-05
      CLI if available, else AdvantageScope).

## Notes

- Keep `RunAuto` generic — task 04 parameterizes it over all five autos.
- If the trench autos' path-following needs more than vision-converged odometry to
  behave (e.g. PathPlanner quirks in stepped time), note findings here for task 04;
  ShootAndStay was chosen first because it barely moves.
