# AI-Agent Testing Environment

Infrastructure that lets an AI coding agent run, observe, and iterate on this robot's code
autonomously — the closed loop demonstrated by Team 254 (background: [`254-ai-env.md`](../../254-ai-env.md)):

1. **Run** a headless simulation scenario (`./gradlew` test task)
2. **Inspect** the resulting `.wpilog` (Python query CLI)
3. **Diagnose** the failure from logs + code
4. **Fix** and re-run
5. **Verify** against explicit success criteria and report quantitative results

254 builds on Java + AdvantageKit. This repo is C++ (GradleRIO), so the equivalent
capability is built from the pieces below. Each numbered file in this directory is a
**self-contained task spec**: an agent (or human) should be able to execute it cold,
as a single PR to `main`, without any other context.

## Architecture decisions

These were resolved deliberately (June 2026). Do not silently re-litigate them in task PRs.

| Topic | Decision |
|---|---|
| MVP | Closed-loop sim harness. Replay is phase 2, and only **algorithm-level** replay (pure functions fed logged values) — no AdvantageKit-style whole-robot IO replay. |
| Run model | **In-process GoogleTest.** Tests construct the real `Robot` object and drive its mode/periodic methods directly with a stepped HAL clock (faster than real time, deterministic, single-threaded). Never `StartCompetition()` in tests — its notifier thread races with stepped time. |
| Singletons | Subsystems stay Meyers singletons. The fixture resets state between tests; `--gtest_filter` gives per-process isolation when a test can't share state. No dependency-injection refactor. |
| Log artifact | `frc::DataLogManager` + NetworkTables capture → `.wpilog`. Already live on the real robot (`Robot.cpp` constructor); the test harness starts it too, into `build/test-logs/`. The custom `utilities/Logger.h` needs no call-site changes — its NT writes are captured automatically. |
| Log tooling | Python, stdlib-only query CLI over `.wpilog` (`tools/logcli.py`): topic glob + time window + downsample → CSV, plus `--summary` stats. Never dump whole logs into an LLM context. |
| Scenarios | C++ GoogleTest cases on the `SimScenario` fixture. Prefer scheduling commands directly over emulating joystick input; `XboxControllerSim` helpers exist for binding-level tests. |
| Success criteria | Mechanism-level for MVP: setpoints reached, command sequencing, pose-vs-path error, current draw. Game-piece scoring ("N balls scored") comes from a **season-generic** game-piece framework (task 10) hooked to the fixture's per-step observer seam — piece types / possession / ordering / scoring rules are pluggable per year. |
| Project mgmt | Task specs live here, numbered. One task = one PR to `main`. |
| Agent knowledge | `.claude/skills/<name>/SKILL.md` in-repo: `robot-description`, `game-info`, `sim-testing`, `log-analysis` (+ `replay` in phase 2). Version-controlled, reusable across seasons. |
| MVP acceptance | Task 09: agent runs every registered auto, emits a per-auto criteria table, then demonstrates one inject-bug → diagnose-from-logs → fix → verify cycle unassisted. |

## Task index

Track A (harness) is the critical path. Track B and C run in parallel with it.

| Spec | Task | Size | Depends on | Phase |
|---|---|---|---|---|
| [01](01-enable-tests.md) | Re-enable the GoogleTest suite | S | — | MVP |
| [02](02-simscenario-fixture.md) | `SimScenario` fixture core + sim calibration tests | L | 01 | MVP |
| [03](03-runauto-first-scenario.md) | `RunAuto` + first auto scenario (ShootAndStay) | M | 02 | MVP |
| [04](04-auto-regression-suite.md) | All-autos regression suite + results table | M | 03 | MVP |
| [05](05-wpilog-query-cli.md) | Python wpilog query CLI | M | — | MVP |
| [06](06-skills-robot-game.md) | Skills: robot-description + game-info | M | — | MVP |
| [07](07-skills-sim-log.md) | Skills: sim-testing + log-analysis | S | 02, 05 | MVP |
| [08](08-claude-md.md) | Root CLAUDE.md + closed-loop workflow | S | 06, 07 | MVP |
| [09](09-acceptance-demo.md) | MVP acceptance demo | M | 04, 05, 07, 08 | MVP gate |
| [10](10-game-piece-model.md) | Season-generic game-piece model framework | L | 04 | Fast-follow |
| [11](11-ci-hardening.md) | CI hardening for desktop tests | S | 01 | Fast-follow |
| [12](12-algorithm-replay.md) | Algorithm-level replay runner + skill | L | 05 | Phase 2 |

## How to execute a task (agent workflow)

1. Read the spec fully. Read every file it references before editing anything.
2. Create a branch named `ai-env/<NN>-<slug>` off `main`.
3. Implement. Match existing code style (`.clang-format` is enforced in CI for `src/main`).
4. Verify per the spec's acceptance criteria — actually run the commands; do not claim
   success without output.
5. Open a PR to `main` titled `[AI-Env NN] <task title>`, with acceptance-criteria
   results in the description.
6. If the spec turns out to be wrong about the codebase (things drift), fix the spec in
   the same PR and call it out in the description.

## Verified technical findings

Hard-won facts the task specs rely on. File references are correct as of June 2026.

### Driving the robot loop in a test
- Construct the **real `Robot`** once per process (it is process-lifetime, like the
  subsystem singletons). Never call `StartCompetition()`.
- Per 20 ms step: `frc::sim::StepTiming(20_ms)` → `DriverStationSim::NotifyNewData()` →
  mode-transition dispatch (replicating WPILib `IterativeRobotBase::LoopFunc`: on DS mode
  change call `DisabledInit/AutonomousInit/TeleopInit` + matching `*Exit`) → the mode's
  `*Periodic()` → `RobotPeriodic()` → `Robot::SimulationPeriodic()`.
- **Verified:** `CommandScheduler::Run()` (called by `RobotPeriodic`) runs each registered
  subsystem's `Periodic()` **and** `SimulationPeriodic()` in desktop builds
  (`if constexpr (frc::RobotBase::IsSimulation())`, CommandScheduler.cpp:183-189 in the
  WPILib 2026.2.1 sources). The existing physics sims run with no extra wiring.
- `DataLogManager::Start("build/test-logs")` must run **before** `Robot`'s constructor
  (which calls `Start()` with defaults — subsequent calls are no-ops), so test logs land
  in the test directory.
- `RUNNING_FRC_TESTS` is defined for the test binary (build.gradle test suite). It
  removes `main()` in `Robot.cpp` and the WebServer startup (port 5800).

### Auto selection
- Chooser: `SendableChooser<std::shared_ptr<frc2::CommandPtr>>` published at NT
  `/SmartDashboard/CHOSEN AUTON` (`src/main/cpp/RobotContainer.cpp:48`, via
  `utilities/AutonHelper.h`).
- Registered autos (`RobotContainer.cpp:38-46`): `ShootAndStay` (default), `LeftTrench`,
  `RightTrench`, `(Short first pass) LeftTrench`, `(Short first pass) RightTrench`.
  Names contain spaces/parens — match exactly.
- Programmatic selection: write the string topic
  `/SmartDashboard/CHOSEN AUTON/selected`, run 1–2 loop iterations including
  `frc::SmartDashboard::UpdateValues()`, then schedule
  `RobotContainer::GetAutonomousCommand()` (this mirrors `Robot::AutonomousInit`,
  `Robot.cpp:59-69`). `AutonManager::GetChosenAuton()` returns `weak_ptr.lock()` —
  **assert non-null** with the auto name in the failure message
  (`src/main/cpp/utilities/AutonHelper.cpp:27-29`).

### Build system
- Test suite block in `build.gradle` (`frcUserProgramTest`, GoogleTestTestSuiteSpec);
  gtest entry point `src/test/cpp/main.cpp` (calls `HAL_Initialize(500, 0)`).
- Test executables anchor their working directory to the project root at startup
  (`src/test/cpp/main.cpp` walks up to `build.gradle` and chdirs): `SubVision` loads
  `src/main/deploy/2026-rebuilt.json` relative to cwd on desktop and **throws** if
  missing (`src/main/include/subsystems/SubVision.h:64-65`). Gradle's
  `RunTestExecutable` task runs the exe from its `outputDir` and cannot be given a
  working directory — don't try.
- Iteration command: the single-variant debug test task (see `.claude/skills/sim-testing`
  once task 07 lands; discovered in task 01). Full `./gradlew check` builds every variant
  and is reserved for final verification/CI. `./gradlew build` also runs tests.
- `-Werror` (CI) applies only to `frcUserProgram` binaries, not the test binary.
  clang-format/clang-tidy CI only match `src/main/`.

### Vendor sim behavior
- **Phoenix6 (TalonFX, Pigeon2, CANcoder):** subsystems use the standard `GetSimState()`
  pattern. Actuators produce output only when the DS is attached + enabled:
  `DriverStationSim::SetDsAttached(true)`, `SetEnabled(true)`, `NotifyNewData()`.
  Fallback if outputs stay at 0 V: call `ctre::phoenix6::unmanaged::FeedEnable()` each
  loop. **Open risk:** CTRE status signals refresh on a wall-clock background thread;
  under aggressive `StepTiming` acceleration they can go stale. Task 02 calibrates the
  safe acceleration factor empirically.
- **REV:** `utilities/ICSpark*.h` wraps SparkMax/Flex with built-in sim
  (`CalcSimVoltage()` / `IterateSim()`).
- **Vision stays enabled in tests:** `SubVision::SimulationPeriodic` feeds
  `VisionSystemSim::Update(GetSimPose())`, which converges the pose estimator onto the
  sim pose. `PoseHandler::SetPose()` resets both estimators
  (`src/main/cpp/utilities/PoseHandler.cpp:21-24`).
- `ctre_sim/` at the repo root holds persistent per-device sim state keyed to cwd
  (e.g. CANcoder absolute positions). Mostly desirable (mimics absolute encoders), but
  it is cross-run state — be aware when debugging "impossible" initial conditions.
- `BotVars` reads `/sys/class/net/eth0/address`; on Windows/desktop it falls through to
  the COMP default. No issue in tests.

### Known sim gaps (good demo targets)
- `SubDeploy::Zero()` completes only on a sustained current spike
  (`src/main/cpp/subsystems/SubDeploy.cpp:68-92`) and has **no** sim short-circuit,
  unlike `SubHood::ZeroHood` (`src/main/cpp/subsystems/Hood/SubHood.cpp:98`,
  `|| frc::RobotBase::IsSimulation()`). Autos survive only because the zero sits inside
  a `WithDeadline` group.
- No game-piece sensors exist (no beam breaks); intake/feeder commands are time/velocity
  based. "Balls scored" is not measurable until task 10.

### Misc
- Silence joystick spam in tests: `frc::DriverStation::SilenceJoystickConnectionWarning(true)`.
- Set `DriverStationSim::SetMatchTime(...)`; otherwise `GetMatchTime()` is -1 and
  `ShiftHandler::GetTimeLeft` goes negative (harmless today — rumble no-ops in sim).
- NT capture already proven: existing `logs/*.wpilog` contain `NT:/SmartDashboard/...`
  topics from the custom Logger. Sessions run ~31 MB, so CLI downsampling is mandatory.
- `DataLogManager` flushes ~every 0.25 s wall time; call `DataLogManager::Stop()` in
  gtest Environment teardown so short runs aren't truncated.
