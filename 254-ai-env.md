# Context: Closed-Loop AI Testing System for FRC Robot Code (based on Team 254's approach)

## Core Concept
Build infrastructure that lets an AI coding agent run, observe, and iterate on robot code autonomously — without human intervention for each test cycle.

## The Closed-Loop Cycle
1. **Observe behavior** — Robot/sim logs everything: motor voltages, currents, command states, pose, all relevant metadata, at high frequency.
2. **Inspect logs** — Tooling converts raw logs (e.g. WPILib `.wpilog`) into a format an LLM can read (e.g. CSV).
3. **Agent diagnoses** — Agent reads logs, identifies what's wrong, proposes a fix.
4. **Agent applies fix and re-runs** — Agent makes the code change, re-executes the test.
5. **Compare results** — Agent checks new logs against success criteria; repeats until correct.

## Required Skills/Tools (as "skill files" — text docs describing capabilities, e.g. in a `skills/` directory)

### 1. Simulation agent / harness
- Must run teleop and autonomous simulation with **zero user intervention**.
- Programmable inputs: scripted gamepad/joystick inputs, pre-set autonomous mode selection.
- Agent triggers a full run, captures logs, and inspects the result in a loop.

### 2. Robot description skill
- Static reference doc: robot dimensions, degrees of freedom, mechanism capabilities (e.g. "can shoot in these directions/ranges"), constraints.

### 3. Replay testing
- Re-run a specific logged scenario through new code *without* a full resimulation — apply a code change to existing log data and see what the new code would have done.
- AdvantageScope/AdvantageKit has built-in support for this pattern.
- Useful for tuning algorithms/thresholds against many historical matches at once (their tip-detection example ran threshold tuning across every match log from an event).

### 4. Game info skill
- Field layout, obstacle positions, scoring zones, match timing/structure — so the agent doesn't need this re-explained per prompt.

### 5. Log reading / conversion skill
- Pipeline/tool to convert WPILib logs into LLM-readable formats (e.g. CSV export).

## Success Criteria & Verification
- Define explicit, checkable success criteria up front (e.g. "score > 50 balls," "non-zero balls scored across all autos," "no false positives in tip detection").
- Agent should self-verify by re-running tests after a fix and reporting results against criteria — not just claim it's fixed.

## Workflow Recommendation
- Use **plan mode**: have the agent draft a technical plan first (what it will inspect, change, and how it'll verify) before writing code. Iterate on the plan more than on the code/tests.
- Prompts should reference skill-provided context (game rules, field geometry, robot capabilities) rather than re-explaining each time — the agent should already "know" this from the skill files.

## Example Use Cases to Replicate
- **Threshold tuning via replay**: run an algorithm (e.g. tip detection based on pitch) across all historical match logs, find true/false positives, recommend a tuned threshold, and produce a results table per match.
- **Bug-fix-and-verify loop**: agent runs sim, finds broken behavior (e.g. auto scores 0), inspects code, finds root cause (e.g. typo/incorrect action mapping), fixes it, reruns multiple autonomous routines unprompted, and reports quantitative results (balls scored) confirming the fix.
- **Energy/current analysis**: post-match analytics pipeline (via the log-reading skill) generating per-match energy/current draw graphs to inform gear ratio and current-limit tuning.

## Architecture Notes
- Treat skill files/agent configs as part of the codebase — version-controlled and reusable across seasons.
- Consider open-sourcing skill definitions (they referenced 254 planning to release theirs post-championship).