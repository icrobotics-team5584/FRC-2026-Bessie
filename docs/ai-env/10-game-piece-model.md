# 10 — Season-generic game-piece model framework

**Phase:** Fast-follow | **Size:** L | **Depends on:** [04](04-auto-regression-suite.md)

## Goal

Make "pieces scored" a measurable criterion in sim. The framework must be
**season-generic** — FRC games vary: different piece types in one game, multiple
possession slots, pickup-order rules, different scoring actions and zones. The 2026
implementation is just the first plug-in.

## Design constraints (decided up front — do not bake 2026 into the core)

- **Core abstractions** (`src/test/cpp/harness/GamePieceModel.h/.cpp`):
  - `PieceType` — name + properties (there may be several per game).
  - `FieldStock` — where pieces start on the field (positions/zones, counts,
    respawn/feeding rules).
  - `AcquisitionRule` — predicate over robot state + field state → robot gains a piece
    (e.g. "intake running AND robot pose within zone X AND stock available").
    Rules may enforce **ordering/eligibility** constraints.
  - `PossessionState` — slots/capacity, which piece is where in the robot (intake →
    indexer → feeder), advancing on transport conditions.
  - `ScoringRule` — predicate over robot + possession state → piece leaves robot with
    an outcome (scored / missed / dropped) and a point value. For shooting games the
    outcome check can delegate to ballistics (see below).
  - `GamePieceModel` — owns the above, advances every fixture step via
    `SimScenario::AddPerStepObserver`, exposes counters
    (`PiecesScored(type)`, `PiecesAcquired(type)`, event log with timestamps), and
    writes its state to NT/DataLog topics (`GamePieceModel/...`) so the log CLI and
    AdvantageScope can show it.
- **2026 plug-in** (`GamePieceModel2026.cpp` or similar): fuel-ball stock in trench
  zones; acquired when intake runs near stock; advances through indexer/feeder on those
  motors running; "shot" when feeder runs with flywheel at speed; scored when, at shot
  time, the physics shot model says the ball enters the hub — reuse the team's existing
  shot-calculation work (`ShotPlanner`, see `utilities/ShotPlanner.h`) rather than
  writing new ballistics. A simple "shooter at setpoint + turret/hood on target ⇒
  scored" approximation is an acceptable first cut; document which fidelity level is
  implemented.

## Steps

1. Implement core + 2026 plug-in as above.
2. Wire into the regression suite (task 04): add the "pieces scored" column to the
   per-auto results table; set per-auto expected minimums with the team (e.g.
   "ShootAndStay scores ≥3").
3. Add `GamePieceModel/...` topics to the robot-description skill's topic map.
4. Unit-test the model itself with synthetic robot states (no full sim needed).

## Acceptance criteria

- [ ] Regression table gains a real "pieces scored" column for all five autos.
- [ ] The 2026 rules live entirely in the plug-in; the core compiles with no
      season-specific identifiers (review check).
- [ ] Model state visible in test `.wpilog`s and queryable via `tools/logcli.py`.
- [ ] A deliberate auto-breaking change (e.g. feeder never runs) drops the scored count
      to 0 — demonstrated once.

## Notes

- This unlocks 254-style headline metrics ("non-zero balls scored across all autos").
- Keep rule predicates as composable lambdas/functors so next season is config, not
  rewrite. Resist physics perfectionism: the model exists to catch *behavioral* bugs.
