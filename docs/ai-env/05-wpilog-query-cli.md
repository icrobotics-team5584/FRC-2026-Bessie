# 05 — Python wpilog query CLI

**Phase:** MVP | **Size:** M | **Depends on:** nothing | **Track:** B (log tooling)

## Goal

A stdlib-only Python CLI that turns `.wpilog` files into LLM-consumable extracts. Whole
logs are ~31 MB per session — the agent must **never** read one raw. The CLI answers
"what topics exist and what did they do" (`--summary`) and "give me these signals in
this window" (CSV extraction).

## Context

- `.wpilog` is WPILib's DataLog binary format. WPILib ships a reference pure-Python
  parser: `datalog.py` in allwpilib (`wpiutil/examples/printlog/datalog.py`). Vendor it
  (keep the license header) — do not depend on robotpy or pip packages.
- Real fixture data for development: `logs/*.wpilog` at the repo root (e.g.
  `logs/FRC_20260221_111024.wpilog`, ~31 MB). Topics include NT-captured entries named
  like `NT:/SmartDashboard/Shooter/Motor1/...`, `NT:/SmartDashboard/Drivebase/...`, plus
  DS joystick/match entries. Test-run logs land in `build/test-logs/` and contain
  `Test/<name>` start/end markers (written by the harness, task 02).
- Entry types to handle: double, float, int64, string, boolean, plus arrays of each.
  Struct-typed topics (`struct:Pose2d` etc.): decode if trivial, otherwise pass through
  as hex with a warning — do not crash.

## Deliverables

```
tools/loglib/datalog.py    # vendored parser, license header retained
tools/logcli.py            # the CLI
tools/README.md            # usage examples (kept in sync with the log-analysis skill)
```

## CLI design

```
python tools/logcli.py <file.wpilog> --summary
    # topic list with: name, type, record count, t_start..t_end, min/max/mean (numeric),
    # number of value changes (useful for booleans/strings/enums)
python tools/logcli.py <file.wpilog> --topics "NT:/SmartDashboard/Shooter/*" \
    --start 5.0 --end 12.5 --downsample 50ms [--csv out.csv]
    # wide CSV: timestamp column + one column per matched topic, forward-filled,
    # downsampled; default to stdout
python tools/logcli.py <file.wpilog> --test "AutoShootAndStay" --topics "..." ...
    # resolve --start/--end from the harness's Test/<name> markers
python tools/logcli.py <file.wpilog> --changes "NT:/SmartDashboard/Commands/*"
    # event view: timestamped value-change list for sparse topics (command starts/ends)
```

Rules:
- Stream the file (single pass per invocation); must handle 31 MB in seconds.
- Default output cap ≈ 1000 rows unless `--downsample`/`--no-limit` overrides; print a
  loud note when capping so the agent knows data was elided.
- Topic glob matching via `fnmatch`; `--topics` may repeat.
- Exit non-zero with a clear message on: missing file, no matching topics, unknown
  `--test` marker (list available markers).
- Windows-first: must run via `py -3` with no pip installs; CRLF-safe output.

## Acceptance criteria

- [ ] `--summary` on `logs/FRC_20260221_111024.wpilog` completes in <30 s and lists
      topics with stats.
- [ ] A topic+window CSV extraction matches values hand-checked in AdvantageScope for
      at least one topic.
- [ ] `--test` window resolution works against a harness-produced log from task 02 (if
      not yet available, leave a TODO + a unit test with a synthetic marker log).
- [ ] Handles a struct-typed topic and an array topic without crashing.
- [ ] `python tools/logcli.py --help` documents every mode with an example.

## Notes

- Keep parsing (loglib) separate from presentation (logcli) — task 12's replay runner
  reuses loglib.
- Summary stats per topic are the agent's survey tool; resist adding plotting or pandas.
