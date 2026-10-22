package frc.robot.subsystems;

import java.util.function.Supplier;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class TurretSubsystem extends SubsystemBase {

  private static final double MAX_ONE_DIR_FOV = Constants.TurretConstants.MAX_ONE_DIR_FOV_DEGREES;
  private static final double SLOW_ZONE_DEGREES = 20.0;
  private static final double TRACK_HEALTH_MAX_TX_DEGREES = 3.0;
  private static final double TRACK_HEALTH_MAX_AGE_SECONDS = 0.20;

  /** Minimum gap between tracking debug lines, so the console is not written 50 times a second. */
  private static final double TRACK_DEBUG_PERIOD_SECONDS = 0.5;
  private double lastTrackDebugTimestampSeconds = -1.0;
  private double lastDriveReportSeconds = -1.0;

  private static final String AIM_TRIM_KEY = "Turret/AimTrimDegrees";
  /**
   * How far the saved trim may pull the shot either way.
   *
   * <p>Was 10, which put a measured 10 degree bias exactly on the clamp: the trim could just
   * reach it with nothing left over, and any fine adjustment afterwards did nothing silently.
   * A limit still exists because trim is meant to correct a small residual bias, and a trim large
   * enough to swing the turret across the field is hiding a wrong constant rather than tuning one.
   */
  private static final double MAX_AIM_TRIM_DEGREES = 20.0;
  private double aimTrimDegrees = 0.0;

  public final Translation3d turretTranslation = Constants.TurretConstants.PIVOT_TO_ROBOT_CENTER;

  private final SparkMax spark = new SparkMax(Constants.TurretConstants.kMotorId, MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(15.0, 0, 0, DegreesPerSecond.of(2440), DegreesPerSecondPerSecond.of(2440))
      .withFeedforward(new SimpleMotorFeedforward(0, 7.5, 0))
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 10)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
      .withStatorCurrentLimit(Amps.of(10))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private final SmartMotorController smc = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final PivotConfig turretConfig = new PivotConfig(smc)
      .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV + 5))
      .withStartingPosition(Degrees.of(0))
      .withMOI(0.05)
      .withTelemetry("Turret", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(
          new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(turretTranslation));

  private final Pivot turret = new Pivot(turretConfig);

  private double lastTrackingOutput = 0.0;
  private double filteredTrackingTxDegrees = 0.0;
  private int lastTrackingTagId = -1;
  private double lastTrackingTimestampSeconds = -1.0;
  private double lastCommandedAngleDegrees = 0.0;

  /** Live aim error in degrees, published by the tracking command. +Inf when not tracking. */
  private double trackingErrorDegrees = Double.POSITIVE_INFINITY;

  public TurretSubsystem() {
    Preferences.initDouble(AIM_TRIM_KEY, 0.0);
    setAimTrimDegrees(Preferences.getDouble(AIM_TRIM_KEY, 0.0));
    SmartDashboard.putString("Turret/AimTrimHelp",
        "Positive = left (CCW) viewed along the shot; negative = right (CW). Limit +/-20 deg. "
            + "Type a value here directly - you do not have to press the bumpers 20 times.");
    SmartDashboard.putData("Turret/ResetAimTrim", Commands.runOnce(() -> setAimTrimDegrees(0.0))
        .ignoringDisable(true).withName("Turret.resetAimTrim"));
  }

  /** Saved fine adjustment; positive rotates the shot left (CCW viewed from above). */
  public double getAimTrimDegrees() {
    return aimTrimDegrees;
  }

  private void setAimTrimDegrees(double requestedDegrees) {
    if (Double.isFinite(requestedDegrees)) {
      aimTrimDegrees = Math.max(-MAX_AIM_TRIM_DEGREES,
          Math.min(MAX_AIM_TRIM_DEGREES, requestedDegrees));
    }
    Preferences.setDouble(AIM_TRIM_KEY, aimTrimDegrees);
    SmartDashboard.putNumber(AIM_TRIM_KEY, aimTrimDegrees);
  }

  /** Changes calibration without interrupting the active aiming/shooting command. */
  public Command adjustAimTrimCommand(double deltaDegrees) {
    return Commands.runOnce(() -> setAimTrimDegrees(aimTrimDegrees + deltaDegrees))
        .ignoringDisable(true).withName("Turret.adjustAimTrim");
  }

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return turret.setAngle(turretAngleSupplier);
  }

  public Command center() {
    return turret.setAngle(Degrees.of(0));
  }

  public Angle getRobotAdjustedAngle() {
    return turret.getAngle().plus(Degrees.of(Constants.TurretConstants.ZERO_ROBOT_DIRECTION_DEGREES
        - Constants.TurretConstants.ENCODER_OFFSET_DEGREES));
  }

  public Angle getRawAngle() {
    return turret.getAngle();
  }

  public double getRawEncoderRotations() {
    return spark.getEncoder().getPosition();
  }

  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public void setOpenLoop(double dutyCycle) {
    smc.setDutyCycle(dutyCycle);
    lastTrackingOutput = dutyCycle;
  }

  // ---- Open-loop angle control --------------------------------------------
  // The turret is positioned by proportional duty cycle rather than by handing an angle to the
  // motor controller's position loop. setOpenLoop() is what the manual stick uses and what the
  // team's original tracking used, and both move this turret; setPosition() has never been
  // observed to move it on this robot.

  /** Duty cycle per degree of angle error. */
  private static final double GOTO_KP = 0.020;

  /** Output ceiling, so a large error cannot slam the turret. */
  private static final double GOTO_MAX_OUTPUT = 0.25;

  /** Smallest output that breaks static friction, applied only outside the deadband. */
  private static final double GOTO_STICTION_OUTPUT = 0.045;

  /** Error below which the turret counts as arrived and output drops to zero. */
  private static final double GOTO_DEADBAND_DEGREES = 0.75;

  /** Stop driving toward a soft limit once this close to it. */
  private static final double GOTO_LIMIT_GUARD_DEGREES = 1.0;

  /**
   * Drives the turret toward an absolute angle, open loop.
   *
   * <p>Proportional on error, with a friction floor applied only once the error is genuinely
   * outside the deadband. The floor is a floor on a real demand, not a clamp minimum: clamping
   * made the turret jump to minimum output the instant it left the deadband, overshoot back
   * through, and hunt forever.
   *
   * @param targetDegrees absolute turret angle wanted
   * @return true once the turret is inside the deadband
   */
  public boolean driveTowardAngle(double targetDegrees) {
    double currentDegrees = turret.getAngle().in(Degrees);
    double clampedTarget = Math.max(-MAX_ONE_DIR_FOV, Math.min(MAX_ONE_DIR_FOV, targetDegrees));
    double error = clampedTarget - currentDegrees;

    lastCommandedAngleDegrees = clampedTarget;
    frc.robot.util.RobotLog.put("Turret/CommandedAngleDegrees", clampedTarget);
    frc.robot.util.RobotLog.put("Turret/GotoErrorDegrees", error);

    if (Math.abs(error) < GOTO_DEADBAND_DEGREES) {
      setOpenLoop(0.0);
      return true;
    }

    double output = error * GOTO_KP;
    if (Math.abs(output) < GOTO_STICTION_OUTPUT) {
      output = Math.copySign(GOTO_STICTION_OUTPUT, output);
    }
    output = Math.max(-GOTO_MAX_OUTPUT, Math.min(GOTO_MAX_OUTPUT, output));

    double limit = MAX_ONE_DIR_FOV - GOTO_LIMIT_GUARD_DEGREES;
    boolean blockedByLimit =
        (currentDegrees >= limit && output > 0.0) || (currentDegrees <= -limit && output < 0.0);
    if (blockedByLimit) {
      output = 0.0;
    }

    setOpenLoop(output);
    reportDrive(currentDegrees, clampedTarget, error, output, blockedByLimit);
    return false;
  }

  /**
   * Prints exactly why the turret is or is not moving, rate limited to twice a second.
   *
   * <p>Deliberately readable in the plain driver station console, no AdvantageScope needed. When
   * the turret "refuses to move" this line says which of the possible causes it is: a commanded
   * output of zero, a non-zero output that the motor is ignoring, an angle reading that is outside
   * the soft limits, or the limit guard holding it back.
   */
  private void reportDrive(
      double currentDegrees,
      double targetDegrees,
      double errorDegrees,
      double output,
      boolean blockedByLimit) {
    double nowSeconds = Timer.getFPGATimestamp();
    if (lastDriveReportSeconds >= 0.0 && nowSeconds - lastDriveReportSeconds < 0.5) {
      return;
    }
    lastDriveReportSeconds = nowSeconds;

    boolean angleOutsideSoftLimits = Math.abs(currentDegrees) > MAX_ONE_DIR_FOV;
    System.out.printf(
        "TURRET_DRIVE,angle=%.2f,target=%.2f,error=%.2f,output=%.3f,raw_rot=%.2f,%s%s%n",
        currentDegrees,
        targetDegrees,
        errorDegrees,
        output,
        spark.getEncoder().getPosition(),
        blockedByLimit ? "BLOCKED_BY_LIMIT_GUARD" : "driving",
        angleOutsideSoftLimits
            ? "  *** ANGLE IS OUTSIDE +/-" + (int) MAX_ONE_DIR_FOV
                + " DEG - the Spark soft limits will refuse to move. Run Turret.Rezero. ***"
            : "");
  }

  /**
   * Drives to a fixed angle with NO camera involved.
   *
   * <p>This is the test that separates the two failures that look identical from the driver seat.
   * If the turret reaches the angle, the mechanism and the control path are fine and any remaining
   * aiming problem is vision. If it does not move, vision is irrelevant — the problem is here.
   */
  /**
   * Commands a fixed duty cycle straight at the motor, bypassing every bit of our own logic.
   *
   * <p>No angle solve, no P loop, no deadband, no limit guard — just "turn". This is the test that
   * separates "our code is computing zero output" from "the motor is not responding". If the
   * turret does not move under this, nothing in the vision or aiming code is relevant: it is CAN
   * wiring, the motor, a mechanical bind, or the Spark's own soft limits refusing the direction.
   *
   * <p>Note the Spark's configured soft limits still apply, which is deliberate — this is a
   * diagnostic, not a way to drive the turret into its hard stop.
   */
  public Command rawDutyCycleCommand(double dutyCycle) {
    return run(() -> {
      spark.set(dutyCycle);
      frc.robot.util.RobotLog.put("Turret/RawTestOutput", dutyCycle);
      frc.robot.util.RobotLog.put("Turret/RawTestAngleDegrees", turret.getAngle().in(Degrees));
    }).finallyDo(() -> {
      spark.set(0.0);
      frc.robot.util.RobotLog.put("Turret/RawTestOutput", 0.0);
    }).withName("Turret.rawTest");
  }

  public Command gotoAngleCommand(double targetDegrees) {
    return run(() -> driveTowardAngle(targetDegrees))
        .finallyDo(() -> setOpenLoop(0.0))
        .withName("Turret.goto" + (int) targetDegrees);
  }

  public void setClosedLoopAngle(Angle angle) {
    Angle clampedAngle = Degrees.of(Math.max(-MAX_ONE_DIR_FOV, Math.min(MAX_ONE_DIR_FOV, angle.in(Degrees))));
    smc.setPosition(clampedAngle);
    lastCommandedAngleDegrees = clampedAngle.in(Degrees);
    frc.robot.util.RobotLog.put("Turret/CommandedAngleDegrees", lastCommandedAngleDegrees);
  }

  public Command manualSet(Supplier<Double> dutyCycleSupplier) {
    return run(() -> {
      double rawInput = dutyCycleSupplier.get();
      double currentAngleDegrees = turret.getAngle().in(Degrees);
      double scaledOutput = applySoftLimitScaling(rawInput, currentAngleDegrees);
      setOpenLoop(scaledOutput);
    }).finallyDo(() -> setOpenLoop(0.0)).withName("Turret.manualSet");
  }

  public Command rezero() {
    return Commands.runOnce(() -> spark.getEncoder().setPosition(0), this).withName("Turret.Rezero");
  }

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  public void clearTrackingTelemetry() {
    trackingErrorDegrees = Double.POSITIVE_INFINITY;
    lastTrackingOutput = 0.0;
    filteredTrackingTxDegrees = 0.0;
    lastTrackingTagId = -1;
    lastTrackingTimestampSeconds = -1.0;
    frc.robot.util.RobotLog.put("Turret/TrackingState", "NO_TARGET");
    frc.robot.util.RobotLog.put("Turret/TrackingTagID", -1);
    frc.robot.util.RobotLog.put("Turret/FilteredTX", 0.0);
    frc.robot.util.RobotLog.put("Turret/TrackingDutyCycle", 0.0);
    frc.robot.util.RobotLog.put("Turret/TrackingLastValidAgeSec", -1.0);
    frc.robot.util.RobotLog.put("Turret/CommandedAngleDegrees", lastCommandedAngleDegrees);
  }

  public void updateTrackingTelemetry(int tagId, double filteredTxDegrees, double dutyCycle) {
    lastTrackingTagId = tagId;
    filteredTrackingTxDegrees = filteredTxDegrees;
    lastTrackingOutput = dutyCycle;
    lastTrackingTimestampSeconds = Timer.getFPGATimestamp();
    frc.robot.util.RobotLog.put("Turret/TrackingState", "TRACKING");
    frc.robot.util.RobotLog.put("Turret/TrackingTagID", tagId);
    frc.robot.util.RobotLog.put("Turret/FilteredTX", filteredTxDegrees);
    frc.robot.util.RobotLog.put("Turret/TrackingDutyCycle", dutyCycle);
    frc.robot.util.RobotLog.put("Turret/TrackingLastValidAgeSec", 0.0);
    frc.robot.util.RobotLog.put("Turret/CommandedAngleDegrees", lastCommandedAngleDegrees);
  }

  public double getLastTrackingOutput() {
    return lastTrackingOutput;
  }

  public double getFilteredTrackingTxDegrees() {
    return filteredTrackingTxDegrees;
  }

  public int getLastTrackingTagId() {
    return lastTrackingTagId;
  }

  public double getLastValidTrackingAgeSeconds() {
    if (lastTrackingTimestampSeconds < 0.0) {
      return -1.0;
    }
    return Timer.getFPGATimestamp() - lastTrackingTimestampSeconds;
  }

  public boolean isTrackingHealthy() {
    double ageSeconds = getLastValidTrackingAgeSeconds();
    return lastTrackingTagId != -1
        && ageSeconds >= 0.0
        && ageSeconds <= TRACK_HEALTH_MAX_AGE_SECONDS
        && Math.abs(filteredTrackingTxDegrees) <= TRACK_HEALTH_MAX_TX_DEGREES;
  }

  /** Published by the tracking command each loop. Pass +Inf when there is nothing to track. */
  public void setTrackingError(double errorDegrees) {
    trackingErrorDegrees = errorDegrees;
  }

  /** Current aim error in degrees, or +Inf when not tracking. */
  public double getTrackingErrorDegrees() {
    return trackingErrorDegrees;
  }

  /**
   * True when the turret is aimed within firing tolerance of a live target.
   *
   * <p>Gates the feed in autonomous and in the one-button shot, so balls are not launched at a
   * turret that has not converged yet.
   */
  public boolean isOnTarget() {
    return Double.isFinite(trackingErrorDegrees)
        && Math.abs(trackingErrorDegrees) <= Constants.TurretConstants.ON_TARGET_TOLERANCE_DEGREES;
  }

  @Override
  public void periodic() {
    double requestedTrim = SmartDashboard.getNumber(AIM_TRIM_KEY, aimTrimDegrees);
    if (Double.compare(requestedTrim, aimTrimDegrees) != 0) {
      setAimTrimDegrees(requestedTrim);
    }
    turret.updateTelemetry();
    frc.robot.util.RobotLog.put("Turret/AngleDegrees", turret.getAngle().in(Degrees));
    frc.robot.util.RobotLog.put("Turret/AngleErrorDegrees", lastCommandedAngleDegrees - turret.getAngle().in(Degrees));
    frc.robot.util.RobotLog.put("Turret/RawEncoderRotations", spark.getEncoder().getPosition());
    frc.robot.util.RobotLog.put("Turret/LastTrackingOutput", lastTrackingOutput);
    frc.robot.util.RobotLog.put("ASCalibration/FinalComponentPoses", new Pose3d[] {
        new Pose3d(
            turretTranslation,
            new Rotation3d(0, 0, turret.getAngle().in(Radians)))
    });
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }

  private double applySoftLimitScaling(double input, double currentAngleDeg) {
    double distanceToPositiveLimit = MAX_ONE_DIR_FOV - currentAngleDeg;
    double distanceToNegativeLimit = currentAngleDeg - (-MAX_ONE_DIR_FOV);

    double scale = 1.0;
    if (input > 0.0 && distanceToPositiveLimit < SLOW_ZONE_DEGREES) {
      scale = Math.max(0.0, distanceToPositiveLimit / SLOW_ZONE_DEGREES);
    } else if (input < 0.0 && distanceToNegativeLimit < SLOW_ZONE_DEGREES) {
      scale = Math.max(0.0, distanceToNegativeLimit / SLOW_ZONE_DEGREES);
    }

    return input * scale;
  }

  /**
   * Debug dump of one tracking step.
   *
   * <p>Rate limited to {@link #TRACK_DEBUG_PERIOD_SECONDS}, and it no longer calls
   * {@link DriverStation#reportWarning}. It used to do both on every loop at 50 Hz, which floods
   * the driver station log, bloats the match log, and burns radio bandwidth during a match — the
   * console line alone is plenty for diagnosing tracking.
   */
  public void reportTrackingCommand(
      int tagId,
      double rawTxDegrees,
      double correctedTxDegrees,
      double filteredTxDegrees,
      double currentAngleDegrees,
      double requestedTargetDegrees,
      double steppedTargetDegrees) {
    double nowSeconds = Timer.getFPGATimestamp();
    if (lastTrackDebugTimestampSeconds >= 0.0
        && nowSeconds - lastTrackDebugTimestampSeconds < TRACK_DEBUG_PERIOD_SECONDS) {
      return;
    }
    lastTrackDebugTimestampSeconds = nowSeconds;

    System.out.printf(
        "TURRET_TRACK_DEBUG,tag=%d,raw_tx=%.2f,corrected_tx=%.2f,filtered_tx=%.2f,current_deg=%.2f,requested_deg=%.2f,commanded_deg=%.2f,error_deg=%.2f%n",
        tagId,
        rawTxDegrees,
        correctedTxDegrees,
        filteredTxDegrees,
        currentAngleDegrees,
        requestedTargetDegrees,
        steppedTargetDegrees,
        steppedTargetDegrees - currentAngleDegrees);
  }
}
