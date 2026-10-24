package frc.robot.cyberpsycho;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DashboardTelemetry;
import frc.robot.controls.RemappableController;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.vision.Limelight;

/**
 * CyberPsycho: every amp to the wheels, nothing else fires, and there is no way back.
 *
 * <p>Entered with UP UP DOWN DOWN LEFT RIGHT LEFT RIGHT on either controller's D-pad (see
 * {@link KonamiCode}), in any mode, enabled or not. From that loop on:
 *
 * <ul>
 *   <li><b>Every mechanism is cut off.</b> One command takes the shooter, turret, intake, hopper
 *       and kicker, writes zero to all of them every loop, and refuses anything else that asks
 *       for them. Buttons, default commands and autonomous routines that need a mechanism are
 *       simply not scheduled, rather than each binding having to know about this mode.</li>
 *   <li><b>The drive gets everything.</b> The speed cap rises past what the NEOs can reach on a
 *       full battery, teleop stops applying the dashboard's speed setting, voltage compensation
 *       and the ramp rate come off, and each drive motor's current limit goes up to
 *       {@link #DRIVE_CURRENT_LIMIT_AMPS}. See {@link SwerveSubsystem#unleash}.</li>
 *   <li><b>The Limelight becomes a camera.</b> See {@link Limelight#setCameraOnly}.</li>
 *   <li><b>Everyone is told.</b> Both controllers rumble, the console says so, and
 *       {@code Dashboard/CyberPsycho/Active} goes true, which is what sends the driver station
 *       dashboard haywire.</li>
 * </ul>
 *
 * <p><b>It cannot be exited.</b> Nothing clears the latch: no button, no dashboard key, not
 * disabling. Only restarting the robot program does, because none of it is persisted -- the
 * latch is a field and the motor settings are written without saving to flash. That is
 * deliberate as well as unavoidable: a mode that survived a reboot would need its own way out,
 * and a brownout mid-match should not need one.
 *
 * <p>Call {@link #periodic} from {@code robotPeriodic} BEFORE the command scheduler runs. That
 * way the lockout is in place before the scheduler polls buttons, so a button pressed on the
 * same loop the code lands never gets one loop of a mechanism moving.
 */
public final class CyberPsycho {

  /**
   * Drive motor current limit once active, per motor, in amps. The JSON's normal limit is 40.
   *
   * <p>As much as the hardware is built for. A NEO v1.1 stalls at 105 A, but the SPARK MAX
   * driving it is rated for 60 A continuous and 100 A for a two-second surge, and 80 A is the
   * limit REV ships it with. More than this and a pushing match holds the controller past its
   * surge rating for as long as the push lasts.
   *
   * <p>What it costs: four NEOs at 80 A launching from a stop ask the battery for 320 A, which
   * can sag it to the roboRIO's brownout threshold, and a browned-out robot drops every output
   * at once. And past about 44 A per motor at this gearing and weight the wheels break traction
   * at launch (COF 1.19), so the top of this range mostly becomes wheel spin rather than
   * acceleration. Every mechanism is at zero, so the drive is the only thing drawing.
   */
  public static final int DRIVE_CURRENT_LIMIT_AMPS = 80;

  /**
   * The battery voltage the top speed is worked out at: a fully charged battery at rest.
   *
   * <p>A NEO turns 473 RPM per volt, so its rated 5676 RPM is a 12 V figure, and a fresh battery
   * is worth about 8% more. Setting the cap here means full stick always asks for more than the
   * motors can give once the battery sags under load, so they run flat out rather than being
   * held to a target below what they could reach.
   */
  public static final double FULL_BATTERY_VOLTS = 13.0;

  /** How long both controllers rumble to say the code took. */
  private static final double RUMBLE_SECONDS = 1.0;

  private final SwerveSubsystem drivebase;
  private final Limelight camera;
  private final RemappableController[] pads;
  private final KonamiCode[] codes;
  private final Command lockout;
  private final NetworkTable table;
  private final Timer rumble = new Timer();

  private boolean active = false;

  public CyberPsycho(
      SwerveSubsystem drivebase,
      Limelight camera,
      ShooterSubsystem shooter,
      TurretSubsystem turret,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      KickerSubsystem kicker,
      RemappableController... pads) {
    this.drivebase = drivebase;
    this.camera = camera;
    this.pads = pads;
    this.codes = new KonamiCode[pads.length];
    for (int i = 0; i < pads.length; i++) {
      codes[i] = new KonamiCode();
    }

    // kCancelIncoming is what makes this a lockout rather than one more command: anything else
    // that wants one of these subsystems is refused while this holds it. Runs while disabled so
    // it is still holding them when the robot is enabled again.
    this.lockout = Commands.run(() -> {
      shooter.cutPower();
      turret.setOpenLoop(0.0);
      intake.cutPower();
      hopper.setFeedEnabled(false);
      kicker.setFeedEnabled(false);
    }, shooter, turret, intake, hopper, kicker)
        .ignoringDisable(true)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("CyberPsycho.Lockout");

    this.table = NetworkTableInstance.getDefault().getTable(DashboardTelemetry.TABLE);
    table.getEntry("CyberPsycho/Active").setBoolean(false);
  }

  /** True from the loop the code lands until the robot program restarts. */
  public boolean isActive() {
    return active;
  }

  /** Watches for the code, and once active keeps the lockout in place. Every loop. */
  public void periodic() {
    if (!active) {
      double now = Timer.getFPGATimestamp();
      for (int i = 0; i < pads.length; i++) {
        if (codes[i].update(pads[i].pov(), now)) {
          activate(pads[i].label());
          break;
        }
      }
      return;
    }

    // Put back whenever something removed it. teleopInit cancels every command when no auto
    // ran, and so does entering test mode; without this the mechanisms would come back to life
    // on the next button press.
    if (!lockout.isScheduled()) {
      CommandScheduler.getInstance().schedule(lockout);
    }

    if (rumble.isRunning() && rumble.hasElapsed(RUMBLE_SECONDS)) {
      rumble.stop();
      for (RemappableController pad : pads) {
        pad.setRumble(0.0);
      }
    }
  }

  private void activate(String who) {
    active = true;

    CommandScheduler.getInstance().schedule(lockout);
    double topSpeed = drivebase.unleash(DRIVE_CURRENT_LIMIT_AMPS, FULL_BATTERY_VOLTS);
    camera.setCameraOnly();

    for (RemappableController pad : pads) {
      pad.setRumble(1.0);
    }
    rumble.restart();

    table.getEntry("CyberPsycho/TopSpeedMetersPerSecond").setDouble(topSpeed);
    table.getEntry("CyberPsycho/DriveCurrentLimitAmps").setDouble(DRIVE_CURRENT_LIMIT_AMPS);
    table.getEntry("CyberPsycho/Active").setBoolean(true);

    DriverStation.reportWarning(String.format(
        "[CYBERPSYCHO] ACTIVE, entered on the %s controller. Mechanisms locked out. Drive"
            + " unleashed: speed cap %.2f m/s, %d A per motor, no voltage compensation, no ramp."
            + " Limelight is camera only. There is no way out short of restarting the robot"
            + " program.",
        who, topSpeed, DRIVE_CURRENT_LIMIT_AMPS), false);
  }
}
