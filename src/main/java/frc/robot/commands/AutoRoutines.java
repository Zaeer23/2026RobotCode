package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Autonomous routines for 2026 REBUILT.
 *
 * <h2>Why these are vision-relative rather than coordinate-based</h2>
 *
 * <p>Every routine here aims, ranges, and repositions off the HUB AprilTags rather than off
 * odometry coordinates. That choice is deliberate: a coordinate-based auto is only as good as the
 * starting pose, and a robot placed a few inches off the line — or bumped in the first second —
 * carries that error all the way through. Closing the loop on the tags means the robot corrects
 * whatever the real starting position turns out to be, which matters far more at an event than at
 * home.
 *
 * <p>It also means these routines do not care which alliance station the robot starts in.
 * {@link FieldConstants#hubTagIds()} resolves the correct hub once the driver station reports an
 * alliance, so the same routine works on either side of the field with no mirrored copy.
 *
 * <h2>Tuning at the event</h2>
 *
 * <p>The timings below are the knobs. {@link #COLLECT_DRIVE_SECONDS} and
 * {@link #COLLECT_DRIVE_SPEED_METERS_PER_SEC} decide how far the robot ranges out to sweep up
 * FUEL; everything else is closed-loop and should not need touching.
 */
public final class AutoRoutines {

  private AutoRoutines() {
  }

  /** How long to let the turret and flywheel converge before firing anyway. */
  private static final double READY_TIMEOUT_SECONDS = 2.0;

  /** How long to run the feed once the shot is ready. Long enough to clear the hopper. */
  private static final double FEED_SECONDS = 2.0;

  /** Budget for the whole first shot, including spin-up. */
  private static final double PRELOAD_SHOT_BUDGET_SECONDS = 4.5;

  /** Budget for the second volley. */
  private static final double SECOND_SHOT_BUDGET_SECONDS = 4.0;

  /** How long the robot drives while sweeping up FUEL. */
  private static final double COLLECT_DRIVE_SECONDS = 2.0;

  /** Collection drive speed. Slow enough that the intake actually picks balls up. */
  private static final double COLLECT_DRIVE_SPEED_METERS_PER_SEC = 1.2;

  /** How long to let the lineup command reacquire before giving up and shooting anyway. */
  private static final double LINEUP_TIMEOUT_SECONDS = 3.0;

  /**
   * Fires whatever is loaded, from wherever the robot currently stands.
   *
   * <p>Holds the turret on the hub and the flywheel at the vision-computed speed for the whole
   * window, and only starts the feed once both report ready. Feeding into a flywheel that has not
   * spun up is the single most common way to put a ball nowhere near the hub.
   */
  public static Command shootFromHere(
      Superstructure superstructure,
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      LimeLight limelight,
      SwerveSubsystem drivebase,
      double budgetSeconds) {
    Command aimAndSpin = Commands.parallel(
        new TurretTrackCommand(turret, limelight, drivebase),
        shooter.setSpeedFromLimelight(limelight, drivebase));

    Command fireWhenReady = Commands.sequence(
        Commands.waitUntil(() -> turret.isOnTarget() && shooter.isAtShotSpeed())
            .withTimeout(READY_TIMEOUT_SECONDS),
        Commands.runOnce(() -> Logger.recordOutput("Auto/Firing", true)),
        superstructure.feedAllCommand().withTimeout(FEED_SECONDS),
        Commands.runOnce(() -> Logger.recordOutput("Auto/Firing", false)));

    // Deadline: the aim/spin half runs only as long as the firing half needs it.
    return Commands.deadline(fireWhenReady, aimAndSpin)
        .withTimeout(budgetSeconds)
        .withName("Auto.shootFromHere");
  }

  /**
   * Drives the chassis for a fixed time while the intake sweeps.
   *
   * <p>Open-loop on purpose. There is nothing on the field to close a loop against while facing
   * away from the hub, and a timed drive at a known speed is predictable and easy to re-tune at an
   * event. The turret keeps tracking throughout so it is already aimed when the robot turns back.
   *
   * @param forwardMetersPerSecond positive drives toward the robot's front; negative backs up
   */
  public static Command collectFuel(
      Superstructure superstructure,
      TurretSubsystem turret,
      LimeLight limelight,
      SwerveSubsystem drivebase,
      double forwardMetersPerSecond,
      double seconds) {
    Command drive = Commands.runEnd(
        () -> drivebase.drive(new Translation2d(forwardMetersPerSecond, 0.0), 0.0, false),
        () -> drivebase.drive(Translation2d.kZero, 0.0, false),
        drivebase);

    // With the intake disabled there is nothing to collect, so the drive-out is skipped entirely
    // rather than trundling around scooping air and eating seconds of the autonomous period.
    if (!frc.robot.Constants.IntakeConstants.ENABLED) {
      return Commands.runOnce(
          () -> System.out.println("AUTO_SKIP,step=collectFuel,reason=intake_disabled"))
          .withName("Auto.collectFuel(skipped)");
    }

    return Commands.deadline(
        Commands.waitSeconds(seconds),
        drive,
        superstructure.setIntakeDeployAndRoll().asProxy(),
        new TurretTrackCommand(turret, limelight, drivebase))
        .withName("Auto.collectFuel");
  }

  /**
   * Shoots the preload and stops. The safe default when there is no time to test anything else.
   */
  public static Command shootPreloadOnly(
      Superstructure superstructure,
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      LimeLight limelight,
      SwerveSubsystem drivebase) {
    return Commands.sequence(
        Commands.runOnce(() -> logAutoStart("shootPreloadOnly")),
        shootFromHere(
            superstructure, turret, shooter, limelight, drivebase, PRELOAD_SHOT_BUDGET_SECONDS),
        stopEverything(superstructure, drivebase))
        .withName("Auto.shootPreloadOnly");
  }

  /**
   * The full routine: shoot the preload, drive out and sweep up FUEL, line back up on the hub, and
   * shoot again.
   *
   * <p>Fits inside a 15 second autonomous with margin: roughly 4.5 s of shooting, 2 s of
   * collecting, up to 3 s of reacquiring, then 4 s of shooting.
   */
  public static Command shootCollectShoot(
      Superstructure superstructure,
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      LimeLight limelight,
      SwerveSubsystem drivebase) {
    return Commands.sequence(
        Commands.runOnce(() -> logAutoStart("shootCollectShoot")),

        // 1. Preload out of the way first, while the robot is still exactly where it was placed.
        shootFromHere(
            superstructure, turret, shooter, limelight, drivebase, PRELOAD_SHOT_BUDGET_SECONDS),

        // 2. Back away from the hub with the intake down. Reversing keeps the camera pointed at
        //    the hub the whole time, so the turret never loses its target and the lineup in step 3
        //    starts from a tag it can already see.
        collectFuel(
            superstructure,
            turret,
            limelight,
            drivebase,
            -COLLECT_DRIVE_SPEED_METERS_PER_SEC,
            COLLECT_DRIVE_SECONDS),

        // 3. Close the loop back onto the calibrated standoff range.
        new HubLineupCommand(drivebase, limelight).withTimeout(LINEUP_TIMEOUT_SECONDS),

        // 4. Everything we picked up.
        shootFromHere(
            superstructure, turret, shooter, limelight, drivebase, SECOND_SHOT_BUDGET_SECONDS),

        stopEverything(superstructure, drivebase))
        .withName("Auto.shootCollectShoot");
  }

  /**
   * Shoot the preload, then drive clear of the starting line without collecting.
   *
   * <p>Useful when an alliance partner wants that space, or when the intake is not trusted.
   */
  public static Command shootAndLeave(
      Superstructure superstructure,
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      LimeLight limelight,
      SwerveSubsystem drivebase) {
    Command leave = Commands.runEnd(
        () -> drivebase.drive(
            new Translation2d(-COLLECT_DRIVE_SPEED_METERS_PER_SEC, 0.0), 0.0, false),
        () -> drivebase.drive(Translation2d.kZero, 0.0, false),
        drivebase)
        .withTimeout(COLLECT_DRIVE_SECONDS);

    return Commands.sequence(
        Commands.runOnce(() -> logAutoStart("shootAndLeave")),
        shootFromHere(
            superstructure, turret, shooter, limelight, drivebase, PRELOAD_SHOT_BUDGET_SECONDS),
        leave,
        stopEverything(superstructure, drivebase))
        .withName("Auto.shootAndLeave");
  }

  /** Parks everything at the end of a routine so nothing coasts into teleop still running. */
  private static Command stopEverything(Superstructure superstructure, SwerveSubsystem drivebase) {
    return Commands.parallel(
        superstructure.stopShootingCommand().asProxy(),
        superstructure.stopFeedingAllCommand().asProxy(),
        Commands.runOnce(() -> drivebase.drive(Translation2d.kZero, 0.0, false), drivebase))
        .withName("Auto.stopEverything");
  }

  private static void logAutoStart(String routineName) {
    System.out.printf(
        "AUTO_START,routine=%s,alliance=%s,hub_tags=%s%n",
        routineName,
        FieldConstants.alliance(),
        FieldConstants.hubTagIds());
    Logger.recordOutput("Auto/Routine", routineName);
    Logger.recordOutput("Auto/Alliance", FieldConstants.alliance().toString());
  }
}
