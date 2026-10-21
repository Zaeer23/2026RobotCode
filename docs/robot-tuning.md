# Robot tuning

## Shooter power

The shooter uses the existing fixed setting, `Constants.ShooterConstants.REFERENCE_RPM = 3200`.
In the **Current Controls** profile, hold **operator right bumper** to spin without feeding.
Hold **operator right trigger** to shoot; feeding waits for wheel speed and pauses during recovery.
Watch **Cmd RPM** and **Shooter RPM** on the dashboard to compare the target and measured speed.

If actual RPM remains below commanded RPM, check battery voltage, shooter power delivery,
and mechanical drag.

## Turret alignment

With the turret tracking a stationary tag, look along the shooting direction:

- For a miss to the right, hold **operator BACK** and tap **left bumper** to move aim left by **0.5°**.
- For a miss to the left, hold **operator BACK** and tap **right bumper** to move aim right by **0.5°**.
- Trim saves across restarts and is limited to **±10°**. The dashboard shows **Aim trim**.
  SmartDashboard also exposes `Turret/AimTrimDegrees` and the `Turret/ResetAimTrim` command.

The geometry now accounts for camera pitch and its position beside the turret. A constant angular
bias can be trimmed, but a miss that varies with distance calls for checking camera height, pitch,
yaw, lateral position, and the turret encoder zero against physical measurements. Robot coordinates
use **+X forward and +Y robot-left**; judge robot-left while facing the robot's forward direction.
