# 02 — SimScenario fixture core + sim calibration tests

**Phase:** MVP | **Size:** L | **Depends on:** [01](01-enable-tests.md) | **Track:** A (harness)

## Goal

A GoogleTest fixture, `SimScenario`, that constructs the **real `Robot`** and drives the
robot loop deterministically with a stepped HAL clock — proving the full headless robot
(all subsystems, vendor sims, vision) runs in-process, and measuring how fast the clock
can be stepped before Phoenix6 sim breaks down.

## Context — read first

- `docs/ai-env/README.md` → "Driving the robot loop in a test" and "Vendor sim behavior".
- `src/main/cpp/Robot.cpp` — the loop being replicated: `RobotPeriodic` runs
  `CommandScheduler::Run()`, `ShiftHandler::Periodic()`, ShotPlanner display logging,
  `SetTOFOffset`. `AutonomousInit` schedules the chooser command.
- All subsystems are Meyers singletons (`SubX::GetInstance()`), constructed on first use,
  alive for the whole process. `Robot`/`RobotContainer` must likewise be constructed once
  and never destroyed (function-local static in the harness).
- Mode dispatch to replicate (WPILib `IterativeRobotBase::LoopFunc`): on each iteration,
  read DS state (`frc::DriverStation::IsEnabled/IsAutonomous/IsTeleop`); when the mode
  changed since last iteration call the old mode's `*Exit()` then the new mode's
  `*Init()`; then call the current mode's `*Periodic()`, then `RobotPeriodic()`, then
  `Robot::SimulationPeriodic()`.
- `SubHood::SimulationPeriodic` (`src/main/cpp/subsystems/Hood/SubHood.cpp:59-63`) shows
  the canonical subsystem sim loop; `SubShooter::SimulationPeriodic`
  (`src/main/cpp/subsystems/SubShooter.cpp:104-124`) shows the Phoenix6 `GetSimState()`
  pattern; `KrakenIO::UpdateSim` (`src/main/cpp/utilities/KrakenIO.cpp:148-162`) covers
  swerve.

## Deliverables

```
src/test/cpp/harness/SimScenario.h / .cpp     // the fixture
src/test/cpp/harness/TestLogging.h / .cpp     // DataLogManager startup + per-test markers
src/test/cpp/scenarios/SmokeTest.cpp          // calibration + smoke tests below
```

### Fixture API (target)

```cpp
class SimScenario : public ::testing::Test {
 protected:
  void SetUp() override;     // cancel all commands, DS disabled+NotifyNewData,
                             // reset pose, write "Test/<name>/start" log marker
  void TearDown() override;  // disable, cancel all, write end marker

  void SetStartPose(frc::Pose2d pose);   // PoseHandler::GetInstance().SetPose(...)
                                          // (resets BOTH estimators, PoseHandler.cpp:21-24)
  void EnableAutonomous();
  void EnableTeleop();
  void Disable();
  void AdvanceTime(units::second_t t);    // N x 20ms LoopFunc-style steps (see below)
  void Schedule(frc2::CommandPtr&& cmd);  // direct command scheduling
  void AddPerStepObserver(std::function<void(units::second_t)> fn);  // game-piece seam (task 10)

  // Assertion helpers (::testing::AssertionResult so failures carry values):
  //   PoseNear(expected, tolMeters, tolDegrees)
  //   ReachedSpeed(actualFn, target, tol)  or subsystem-specific equivalents
  frc::sim::XboxControllerSim& DriverSim();    // port 0
  frc::sim::XboxControllerSim& OperatorSim();  // port 1
};
```

### One-time environment (gtest `::testing::Environment` or first-use static)

Order matters:
1. `frc::sim::PauseTiming();`
2. `frc::DriverStation::SilenceJoystickConnectionWarning(true);`
3. `frc::DataLogManager::Start("build/test-logs");`  // BEFORE Robot's ctor, whose own
   Start() call then becomes a no-op — logs land in build/test-logs/
4. Construct the `Robot` (function-local static; never call `StartCompetition()`).
5. Environment teardown: `frc::DataLogManager::Stop();`  // flush guard for short runs

### AdvanceTime inner loop (per 20 ms step)

```cpp
frc::sim::StepTiming(20_ms);
frc::sim::DriverStationSim::NotifyNewData();
// mode dispatch: *Exit/*Init on change, then XPeriodic()
robot.RobotPeriodic();          // -> CommandScheduler::Run() -> subsystem Periodic
                                //    AND SimulationPeriodic (verified, see README)
robot.SimulationPeriodic();
for (auto& obs : observers_) obs(frc::Timer::GetFPGATimestamp());
```

Phoenix6 actuators only produce output when the DS is attached + enabled — `Enable*()`
helpers must do `DriverStationSim::SetDsAttached(true)`, `SetEnabled(true)`,
`SetAutonomous(...)`, `NotifyNewData()`. Also `DriverStationSim::SetMatchTime(...)`.

## Calibration & smoke tests (in SmokeTest.cpp)

1. **Shooter (Phoenix6 path):** schedule the shooter setpoint command (see
   `SubShooter::GetInstance()` API), `AdvanceTime(3_s)`, assert flywheel velocity within
   tolerance of target. Then rerun the same test with different per-step real-time
   sleeps (0 ms, 1 ms) and compare convergence — this measures CTRE status-signal
   staleness under accelerated time. **Document the supported acceleration factor as a
   named constant in SimScenario.h** (e.g. `kMaxSimAcceleration`) with a comment
   explaining the measurement. If motor voltage stays at 0 V, add
   `ctre::phoenix6::unmanaged::FeedEnable()` to the step loop and note it.
2. **Hood (REV/ICSpark path):** schedule a hood position target, advance, assert
   convergence (`SubHood` zeroing short-circuits in sim, `SubHood.cpp:98`).
3. **Vision smoke:** `SetStartPose` to a non-origin pose, advance ~2 s, assert the pose
   estimate stays near the sim pose (proves `VisionSystemSim` + `AddVisionMeasurement`
   run headless without throwing or binding ports).
4. **Determinism:** run one scenario twice in the same process; assert key end-state
   values match between runs (catches cross-test state bleed early).

## Acceptance criteria

- [ ] `./gradlew runFrcUserProgramTestWindowsx86-64DebugGoogleTestExe` runs ≥4 new tests
      green, **twice consecutively** (second run catches singleton state bleed).
- [ ] A `.wpilog` containing the per-test markers appears in `build/test-logs/` after a
      run and is readable (verify with the task-05 CLI if available, else AdvantageScope).
- [ ] `kMaxSimAcceleration` (or equivalent) is documented with measured evidence.
- [ ] No test depends on test execution order (verified with `--gtest_shuffle`).

## Risks / notes

- If Phoenix6 staleness makes accelerated stepping unreliable, fall back to
  near-real-time stepping (1 ms sleep per 20 ms step ≈ 20× real time) — a 15 s auto
  still takes <1 s of extra wall time. Record whatever is measured.
- `ctre_sim/` at the repo root persists device state between runs (absolute encoder
  positions). If tests see "impossible" initial mechanism positions, look there.
- Keep the fixture free of scenario-specific logic — `RunAuto` arrives in task 03 and
  the game-piece model hooks in via `AddPerStepObserver` (task 10).
