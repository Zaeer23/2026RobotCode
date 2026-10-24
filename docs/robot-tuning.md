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

## CyberPsycho

On **either** controller's D-pad, press **UP UP DOWN DOWN LEFT RIGHT LEFT RIGHT**. A gap of
more than 1.5 s between presses starts the attempt over, and a direction you press twice has to
come back to centre in between. It works enabled or disabled. Both controllers rumble for a
second when it takes.

From then on:

- **Shooter, turret, intake, hopper and kicker are off.** Their buttons do nothing, and no
  autonomous routine runs, since every one needs a mechanism.
- **The drive gets everything.** Every limit on the drive NEOs comes off, and the dashboard
  **Drive Speed** setting is ignored, so full stick is full power:
  - Speed cap: from 3.05 m/s to 4.02 m/s, the NEO's free speed (473 RPM per volt, 5676 RPM at
    12 V) on a full 13 V battery through the 8.14:1 gearing in the swerve config. Under load the
    battery sags below that, so full stick always asks for more than the motors have and they
    run flat out.
  - Voltage compensation off, so a fresh battery's extra volt reaches the wheels instead of
    being scaled down to 12 V.
  - Current limit: 40 A to 80 A per motor (`CyberPsycho.DRIVE_CURRENT_LIMIT_AMPS`). That is
    REV's factory setting for the SPARK MAX, which is rated for 60 A continuous and 100 A for
    2 s; the NEO itself stalls at 105 A. Above about 44 A the wheels break traction at launch,
    so the top of that range shows up as wheel spin, and four motors at 80 A can pull the
    battery down to a brownout.
  - Ramp rate: 0.25 s to none.
- **The Limelight becomes a plain camera.** The robot stops reading tags and switches the camera
  to pipeline **9** (`LimeLightConstants.DRIVER_CAMERA_PIPELINE`). Set pipeline 9 up once in the
  Limelight web UI as a **Viewfinder** pipeline with the exposure turned up for driving. If it is
  left unset the tag data is still ignored, but the stream keeps looking like a tag tracker.
- **The dashboard goes haywire**, and the header mode reads **CYBERPSYCHO**.

**There is no way out.** No button, dashboard setting or disable ends it. Only restarting the
robot program does (power cycle, or Restart Robot Code in the Driver Station), which also puts
the drive motors' current limit, voltage compensation and ramp back.
