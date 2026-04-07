# C++ Build Time Guidelines

Rules for keeping build times fast as the codebase grows.

## 1. Every header starts with `#pragma once`

```cpp
#pragma once    // ALWAYS line 1, before any includes

#include <frc/geometry/Pose2d.h>
// ...
```

Without this, the same header can be parsed multiple times per translation unit.

## 2. Headers include only what they declare

If a type is only used inside a `.cpp` file, include it in the `.cpp`, not the `.h`.

**Bad** (SubFeeder.h):
```cpp
#include "utilities/Logger.h"  // Only used in SubFeeder.cpp
```

**Good** (SubFeeder.h has no Logger include; SubFeeder.cpp does):
```cpp
// SubFeeder.cpp
#include "subsystems/SubFeeder.h"
#include "utilities/Logger.h"
```

## 3. Forward declare when possible

If a header only uses a type as a pointer, reference, or return type (not as a member variable or base class), use a forward declaration instead of `#include`.

```cpp
// Instead of: #include <frc2/command/button/CommandXboxController.h>
namespace frc2 { class CommandXboxController; }

class MySubsystem {
  frc2::CommandPtr DoSomething(frc2::CommandXboxController& controller);  // Reference = OK
};
```

**You MUST include** when:
- Inheriting from a class (`class Sub : public frc2::SubsystemBase`)
- Using a type as a member variable (`frc::PIDController _pid;`)
- Using `sizeof()` or accessing members

## 4. Never include a subsystem header from a utility header

Utilities are lower-level than subsystems. A utility that needs a subsystem creates a dependency cycle that forces everything to rebuild when any subsystem changes.

**Bad**: `utilities/PoseHandler.h` includes `subsystems/SubDrivebase.h`
**Good**: `PoseHandler.h` only includes what it directly needs (`DrivebaseConfig.h`, `frc/estimator/...`)

## 5. Separate stable constants from tuning values

Constants that never change (CAN IDs, physical dimensions, DIO ports) belong in headers as `constexpr`:
```cpp
constexpr int SHOOTER_MOTOR = 20;
constexpr frc::Translation2d FL_POSITION{+0.281_m, +0.281_m};
```

Values that change during tuning (PID gains, offsets, speeds) should use `extern const` in headers with definitions in `.cpp` files:
```cpp
// DrivebaseConfig.h
extern const frc::PIDController P2P_TRANSLATION_PID;

// DrivebaseConfig.cpp
const frc::PIDController P2P_TRANSLATION_PID{5, 0, 0};
```

This way, changing a PID gain only recompiles `DrivebaseConfig.cpp` + relinks, instead of rebuilding every file that includes the header.

## 6. Keep command headers lightweight

Command headers should only declare function signatures. Include subsystem headers in the `.cpp` file, not the `.h`:

```cpp
// FuelCommands.h - lightweight
#pragma once
#include <frc2/command/CommandPtr.h>
namespace cmd {
  frc2::CommandPtr IntakeSequence();
}

// FuelCommands.cpp - includes everything it needs
#include "commands/FuelCommands.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubIndexer.h"
```

## 7. Split heavy utility headers

If a utility header pulls in expensive vendor includes (like CTRE TalonFX) but most users don't need them, split it:

- `Logger.h` - lightweight, no CTRE dependency
- `LoggerCTRE.h` - CTRE-specific overloads, only included by files that log CTRE signals

## 8. Use the precompiled header for vendor libraries

`pch.h` contains commonly-used WPILib/vendor headers that are parsed once and reused. Rules:
- **DO** add stable external library headers used across many files
- **DON'T** add project headers (they change too often and invalidate the PCH)
- **DON'T** add headers only used by 1-2 files

## Quick checklist for code review

- [ ] New `.h` file has `#pragma once` on line 1?
- [ ] Header doesn't include anything only used in the `.cpp`?
- [ ] No subsystem headers included from utility headers?
- [ ] Tuning values in `.cpp` files, not headers?
- [ ] No unnecessary vendor includes in headers?
