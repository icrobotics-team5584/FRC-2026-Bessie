# 06 — Skills: robot-description + game-info

**Phase:** MVP | **Size:** M | **Depends on:** nothing (content benefits from 01–04 landing) | **Track:** C (knowledge)

## Goal

Two Claude Code knowledge skills so agents "already know" the robot and the game instead
of having them re-explained per prompt (254's recommendation). These are static
reference docs, auto-discovered from the repo.

## Format

`.claude/skills/<name>/SKILL.md` with YAML frontmatter:

```markdown
---
name: robot-description
description: Physical and software description of the 2026 robot Bessie — subsystems,
  motors, sensors, sim models, coordinate conventions, log topic names, known sim gaps.
  Use when reasoning about robot capabilities, hardware, or interpreting logs.
---
<content>
```

Keep each under ~300 lines; link to source files rather than duplicating code.

## `.claude/skills/robot-description/SKILL.md` must cover

- Subsystem inventory (from `src/main/include/subsystems/`): SubDrivebase (swerve, 4×
  TalonFX drive + 4× TalonFX steer, Pigeon2, CANcoders), SubShooter (2× Kraken X60
  flywheels), SubHood (REV via `HoodMotorIO` abstraction — note this IO pattern at
  `src/main/include/subsystems/Hood/` is the team's exemplar), SubTurret (REV +
  CANcoder + throughbore), SubIntake, SubIndexer, SubFeeder, SubDeploy, SubVision
  (PhotonVision, 3 cameras). All are Meyers singletons: `SubX::GetInstance()`.
- Per-subsystem sim model (FlywheelSim / SingleJointedArmSim / DCMotorSim / swerve
  module sim / `photon::VisionSystemSim`) and where `SimulationPeriodic` lives.
- Key log/NT topic names, harvested from `Logger::Log` call sites (grep
  `Logger::Log("` across `src/main/cpp`) — group by subsystem prefix. Note the
  NT-capture prefix in wpilog files: `NT:/SmartDashboard/...`.
- Coordinate conventions and physical dimensions/constraints (check `Constants.h`,
  drivebase geometry, hood angle range, turret soft limits — pull real numbers from
  `Constants.h` and subsystem configs).
- Known sim gaps: `SubDeploy::Zero()` needs a current spike that sim may not produce
  (`src/main/cpp/subsystems/SubDeploy.cpp:68-92` — no sim short-circuit, unlike
  `SubHood.cpp:98`); no game-piece sensors exist, intake/feeder commands are
  time/velocity based, so "balls scored" is not directly measurable (until game-piece
  model task lands).
- `BotVars` COMP/PRACTICE selection (MAC-based on the rio, COMP default on desktop).

## `.claude/skills/game-info/SKILL.md` must cover

- 2026 game ("REBUILT") essentials for testing: field dimensions, hub/target position,
  trench/zone geometry — extract from `src/main/include/utilities/FieldConstants.h` and
  related geometry utilities (verify the actual file names; explore
  `src/main/include/utilities/` for field/geometry headers).
- Match structure and shift timing as implemented in
  `src/main/cpp/utilities/ShiftHandler.cpp` (this is team-specific strategy logic —
  describe what it does, with the caveat that match-time is -1 in tests unless
  `DriverStationSim::SetMatchTime` is set).
- Auto start poses (documented in `src/main/cpp/commands/AutonCommands.cpp` comments)
  and what each registered auto is supposed to accomplish.
- Alliance flipping conventions if present (search for flipping/mirroring utilities).

## Acceptance criteria

- [ ] Both SKILL.md files exist with valid frontmatter and trigger-worthy descriptions.
- [ ] Spot-check: a fresh agent session given only the skills can correctly answer
      "what motors drive the turret and which log topics show its state?" and "where
      does the robot start for the LeftTrench auto?"
- [ ] Every factual claim carries a source-file reference so future seasons can audit
      staleness.
- [ ] `docs/ai-env/README.md` links to the skills.
