package frc.robot.subsystems;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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

  private static final Set<Integer> TRACKING_TAG_IDS =
      Set.of(9, 10, 11, 2, 4, 3, 8, 5, 19, 20, 18, 27, 26, 25, 21, 24);

  private final double MAX_ONE_DIR_FOV = 90;
  public final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375);

  private SparkMax spark = new SparkMax(Constants.TurretConstants.kMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(15.0, 0, 0, DegreesPerSecond.of(244), DegreesPerSecondPerSecond.of(2440))
      .withFeedforward(new SimpleMotorFeedforward(0, 7.5, 0))
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 10)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
      .withStatorCurrentLimit(Amps.of(10))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final PivotConfig turretConfig = new PivotConfig(smc)
      .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV + 5))
      .withStartingPosition(Degrees.of(0))
      .withMOI(0.05)
      .withTelemetry("Turret", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(
          new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(turretTranslation));

  private Pivot turret = new Pivot(turretConfig);

  // Limelight physical offset from turret pivot center
  private static final double LIMELIGHT_HORIZONTAL_OFFSET_INCHES = 9.0;  // tune between 8-10

  // Slow zone: ramp down output near soft limits
  private static final double SLOW_ZONE_DEGREES = 20.0;
  
  // Tracking constants — tune kP if still oscillating/sluggish
  private static final double TRACK_DEADBAND_DEGREES = 0.25;
  private static final double TRACK_TX_FILTER_ALPHA = 0.18;
  private static final double TRACK_DISTANCE_FILTER_ALPHA = 0.12;
  private static final double TRACK_MAX_TARGET_STEP_DEGREES = 2.0;
  private static final double TRACK_MAX_TX_JUMP_DEGREES = 10.0;
  private static final double TRACK_MAX_DISTANCE_JUMP_INCHES = 36.0;
  private static final int TRACK_JUMP_HOLD_CYCLES = 5;
  private static final int TRACK_MIN_ACQUIRE_SAMPLES = 1;
  private static final double TRACK_LATENCY_FUDGE_MS = 20.0;

  private double filteredTrackingTxDegrees = 0.0;
  private double filteredTrackingDistanceInches = 120.0;
  private double lastTrackingTargetAngleDegrees = 0.0;
  private double lastRawTrackingTxDegrees = 0.0;
  private int lastTrackingTagId = -1;
  private int consecutiveTrackingSamples = 0;
  private boolean hasTrackingSample = false;
  private int trackingJumpHoldCyclesRemaining = 0;
  private double lastNoTargetWarningTimestamp = -1.0;
  private double lastBadDataWarningTimestamp = -1.0;
  private double lastOutOfRangeWarningTimestamp = -1.0;
  private double lastTurretConsoleLogTimestamp = -1.0;
  private int activeTurretAttemptId = -1;
  private boolean printedTurretHeader = false;



  public TurretSubsystem() {
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
    return turret.getAngle().plus(Degrees.of(180));
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

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  /**
   * Manual turret control with soft limit slow zone.
   * Uses spark.set() directly to avoid YAMS command-wrapping issues.
   */
  public Command manualSet(Supplier<Double> dutyCycleSupplier) {
    return run(() -> {
      double rawInput = dutyCycleSupplier.get();
      double currentAngle = turret.getAngle().in(Degrees);
      double scaledOutput = applySoftLimitScaling(rawInput, currentAngle);
      spark.set(scaledOutput);
    }).withName("Turret.manualSet");
  }

  /**
   * Track an AprilTag using limelight TX.
   *
   * The limelight is fixed to the robot body, NOT the turret. TX is the
   * horizontal angle from the limelight center to the target. We correct
   * for parallax (physical offset of limelight from turret pivot) based
   * on target distance, then P-control the turret toward corrected TX = 0.
   */
  public Command trackTarget(LimeLight limelight) {
    return trackTarget(limelight, null, TRACKING_TAG_IDS);
  }

  public Command trackTarget(LimeLight limelight, SwerveSubsystem drivebase) {
    return trackTarget(limelight, drivebase, TRACKING_TAG_IDS);
  }

  public Command trackTarget(LimeLight limelight, SwerveSubsystem drivebase, Set<Integer> allowedTagIds) {
    return turret.setAngle(() -> {
      LimeLight.AprilTagScan scan = limelight.scan(allowedTagIds);
      double currentAngleDegrees = turret.getAngle().in(Degrees);
      double nowSeconds = Timer.getFPGATimestamp();

      if (!scan.isValid()) {
        filteredTrackingTxDegrees = 0.0;
        filteredTrackingDistanceInches = 120.0;
        lastTrackingTargetAngleDegrees = currentAngleDegrees;
        lastTrackingTagId = -1;
        consecutiveTrackingSamples = 0;
        hasTrackingSample = false;
        trackingJumpHoldCyclesRemaining = 0;
        Logger.recordOutput("Turret/TrackingState", "NO_TARGET");
        Logger.recordOutput("Turret/TrackingTagID", -1);
        Logger.recordOutput("Turret/FilteredTX", filteredTrackingTxDegrees);
        Logger.recordOutput("Turret/TrackingTargetAngleDegrees", currentAngleDegrees);
        reportNoTargetWarning(scan);
        maybePrintTurretTelemetry(nowSeconds, scan, currentAngleDegrees, currentAngleDegrees, "NO_TARGET");
        return turret.getAngle();
      }

      double robotOmegaDegPerSecond = 0.0;
      if (drivebase != null) {
        robotOmegaDegPerSecond = Math.toDegrees(drivebase.getRobotVelocity().omegaRadiansPerSecond);
      }
      double visionLatencySeconds = (scan.latencyMs + TRACK_LATENCY_FUDGE_MS) / 1000.0;
      double latencyCompensationDegrees = robotOmegaDegPerSecond * visionLatencySeconds;
      double rawTxDegrees = -(scan.tx + latencyCompensationDegrees);
      double rawDistanceInches = Math.max(scan.distance * 39.37, 1.0);
      if (!Double.isFinite(scan.tx) || !Double.isFinite(scan.ty) || !Double.isFinite(scan.distance)) {
        reportBadVisionDataWarning(scan);
        maybePrintTurretTelemetry(nowSeconds, scan, currentAngleDegrees, currentAngleDegrees, "BAD_DATA");
        return turret.getAngle();
      }
      boolean tagChanged = scan.tagID != lastTrackingTagId;
      if (tagChanged) {
        consecutiveTrackingSamples = 0;
        hasTrackingSample = false;
        trackingJumpHoldCyclesRemaining = 0;
        lastTrackingTagId = scan.tagID;
      }

      if (!hasTrackingSample) {
        filteredTrackingTxDegrees = rawTxDegrees;
        filteredTrackingDistanceInches = rawDistanceInches;
        lastRawTrackingTxDegrees = rawTxDegrees;
        consecutiveTrackingSamples++;
        Logger.recordOutput("Turret/TrackingState", "ACQUIRING");
        Logger.recordOutput("Turret/TrackingAcquireSamples", consecutiveTrackingSamples);
        Logger.recordOutput("Turret/TrackingTagID", scan.tagID);
        maybePrintTurretTelemetry(nowSeconds, scan, currentAngleDegrees, currentAngleDegrees, "ACQUIRING");
        if (consecutiveTrackingSamples < TRACK_MIN_ACQUIRE_SAMPLES) {
          return turret.getAngle();
        }
        hasTrackingSample = true;
      }
        
      // Freeze briefly on unrealistic one-frame jumps instead of commanding a violent reversal.
      double txDeltaDegrees = rawTxDegrees - lastRawTrackingTxDegrees;
      double distanceDeltaInches = rawDistanceInches - filteredTrackingDistanceInches;
      if (Math.abs(txDeltaDegrees) > TRACK_MAX_TX_JUMP_DEGREES
          || Math.abs(distanceDeltaInches) > TRACK_MAX_DISTANCE_JUMP_INCHES) {
        trackingJumpHoldCyclesRemaining = TRACK_JUMP_HOLD_CYCLES;
      } else if (trackingJumpHoldCyclesRemaining > 0) {
        trackingJumpHoldCyclesRemaining--;
      }
      lastRawTrackingTxDegrees = rawTxDegrees;

      if (trackingJumpHoldCyclesRemaining == 0) {
        // Smooth the measured tag angle so noisy frames do not jerk the turret.
        filteredTrackingTxDegrees +=
            (rawTxDegrees - filteredTrackingTxDegrees) * TRACK_TX_FILTER_ALPHA;
        filteredTrackingDistanceInches +=
            (Math.max(rawDistanceInches, 1.0) - filteredTrackingDistanceInches) * TRACK_DISTANCE_FILTER_ALPHA;
      }

      if (Math.abs(filteredTrackingTxDegrees) < TRACK_DEADBAND_DEGREES) {
        filteredTrackingTxDegrees = 0.0;
      }

      double parallaxDegrees = Math.toDegrees(
          Math.atan2(LIMELIGHT_HORIZONTAL_OFFSET_INCHES, filteredTrackingDistanceInches));
      double correctedTargetDegrees = filteredTrackingTxDegrees - parallaxDegrees;
      reportOutOfRangeTrackingWarning(correctedTargetDegrees, scan.tagID);

      // Rate limit the commanded turret angle so fast tag motion cannot cause
      // full-speed reversals from one loop to the next.
      double unclampedTargetAngleDegrees = MathUtil.clamp(
          correctedTargetDegrees, -MAX_ONE_DIR_FOV, MAX_ONE_DIR_FOV);
      double steppedTargetAngleDegrees = MathUtil.clamp(
          unclampedTargetAngleDegrees,
          lastTrackingTargetAngleDegrees - TRACK_MAX_TARGET_STEP_DEGREES,
          lastTrackingTargetAngleDegrees + TRACK_MAX_TARGET_STEP_DEGREES);
      double clampedTargetAngleDegrees = MathUtil.clamp(
          steppedTargetAngleDegrees, -MAX_ONE_DIR_FOV, MAX_ONE_DIR_FOV);
      lastTrackingTargetAngleDegrees = clampedTargetAngleDegrees;

      Logger.recordOutput("Turret/RawTX", scan.tx);
      Logger.recordOutput("Turret/RawDistanceInches", rawDistanceInches);
      Logger.recordOutput("Turret/VisionLatencyMs", scan.latencyMs);
      Logger.recordOutput("Turret/RobotOmegaDegPerSec", robotOmegaDegPerSecond);
      Logger.recordOutput("Turret/LatencyCompensationDegrees", latencyCompensationDegrees);
      Logger.recordOutput("Turret/ClampedTX", rawTxDegrees);
      Logger.recordOutput("Turret/FilteredTX", filteredTrackingTxDegrees);
      Logger.recordOutput("Turret/FilteredDistanceInches", filteredTrackingDistanceInches);
      Logger.recordOutput("Turret/DistanceJumpInches", distanceDeltaInches);
      Logger.recordOutput("Turret/ParallaxDegrees",   parallaxDegrees);
      Logger.recordOutput("Turret/JumpHoldCycles", trackingJumpHoldCyclesRemaining);
      Logger.recordOutput("Turret/TrackingAcquireSamples", consecutiveTrackingSamples);
      Logger.recordOutput("Turret/TrackingTagID", scan.tagID);
      Logger.recordOutput("Turret/TargetDistanceM", scan.distance);
      Logger.recordOutput("Turret/TrackingTargetAngleDegrees", clampedTargetAngleDegrees);
      Logger.recordOutput("Turret/TrackingState", "TRACKING");
      maybePrintTurretTelemetry(
          nowSeconds,
          scan,
          currentAngleDegrees,
          clampedTargetAngleDegrees,
          "TRACKING");

      return Degrees.of(clampedTargetAngleDegrees);
    }).beforeStarting(() -> {
      activeTurretAttemptId = LimelightAttemptTracker.nextAttemptId();
      lastTurretConsoleLogTimestamp = -1.0;
      System.out.printf(
          "LIMELIGHT_ATTEMPT_START,source=TURRET,attempt=%d%n",
          activeTurretAttemptId);
    }).finallyDo(() -> {
      System.out.printf(
          "LIMELIGHT_ATTEMPT_END,source=TURRET,attempt=%d%n",
          activeTurretAttemptId);
      activeTurretAttemptId = -1;
      lastTurretConsoleLogTimestamp = -1.0;
      lastNoTargetWarningTimestamp = -1.0;
      lastBadDataWarningTimestamp = -1.0;
      lastOutOfRangeWarningTimestamp = -1.0;
    }).withName("Turret.trackTarget");
  }


  private double applySoftLimitScaling(double input, double currentAngleDeg) {
    double distanceToPositiveLimit = MAX_ONE_DIR_FOV - currentAngleDeg;
    double distanceToNegativeLimit = currentAngleDeg - (-MAX_ONE_DIR_FOV);
    double scale = 1.0;

    if (input > 0 && distanceToPositiveLimit < SLOW_ZONE_DEGREES) {
      scale = Math.max(0.0, distanceToPositiveLimit / SLOW_ZONE_DEGREES);
    } else if (input < 0 && distanceToNegativeLimit < SLOW_ZONE_DEGREES) {
      scale = Math.max(0.0, distanceToNegativeLimit / SLOW_ZONE_DEGREES);
    }

    return input * scale;
  }

  @Override
    public void periodic() {
      turret.updateTelemetry();
      Logger.recordOutput("Turret/AngleDegrees", turret.getAngle().in(Degrees));
      Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
          new Pose3d(turretTranslation, new Rotation3d(0, 0, turret.getAngle().in(Radians)))});
          Logger.recordOutput("Turret/RawEncoderRotations", spark.getEncoder().getPosition());
          Logger.recordOutput("Turret/YAMSAngleDegrees", turret.getAngle().in(Degrees));
      }

    @Override
    public void simulationPeriodic() {
      turret.simIterate();
    }

    private void reportNoTargetWarning(LimeLight.AprilTagScan scan) {
      double nowSeconds = Timer.getFPGATimestamp();
      if (lastNoTargetWarningTimestamp < 0 || nowSeconds - lastNoTargetWarningTimestamp >= 1.0) {
        DriverStation.reportWarning(
            String.format(
                "[TURRET][WARN][ATTEMPT %d] Lost/invalid Limelight target while tracking (tagId=%d).",
                activeTurretAttemptId,
                scan.tagID),
            false);
        lastNoTargetWarningTimestamp = nowSeconds;
      }
    }

    private void reportBadVisionDataWarning(LimeLight.AprilTagScan scan) {
      double nowSeconds = Timer.getFPGATimestamp();
      if (lastBadDataWarningTimestamp < 0 || nowSeconds - lastBadDataWarningTimestamp >= 1.0) {
        DriverStation.reportError(
            String.format(
                "[TURRET][ERROR][ATTEMPT %d] Limelight returned non-finite data while tracking (tx=%.3f, ty=%.3f, dist=%.3f, tagId=%d).",
                activeTurretAttemptId,
                scan.tx,
                scan.ty,
                scan.distance,
                scan.tagID),
            false);
        lastBadDataWarningTimestamp = nowSeconds;
      }
    }

    private void reportOutOfRangeTrackingWarning(double correctedTargetDegrees, int tagID) {
      if (Math.abs(correctedTargetDegrees) <= MAX_ONE_DIR_FOV) {
        return;
      }

      double nowSeconds = Timer.getFPGATimestamp();
      if (lastOutOfRangeWarningTimestamp < 0 || nowSeconds - lastOutOfRangeWarningTimestamp >= 1.0) {
        DriverStation.reportWarning(
            String.format(
                "[TURRET][WARN][ATTEMPT %d] Requested target angle %.2f deg exceeds turret range +/-%.1f deg (tagId=%d).",
                activeTurretAttemptId,
                correctedTargetDegrees,
                MAX_ONE_DIR_FOV,
                tagID),
            false);
        lastOutOfRangeWarningTimestamp = nowSeconds;
      }
    }

    private void maybePrintTurretTelemetry(
        double nowSeconds,
        LimeLight.AprilTagScan scan,
        double currentAngleDegrees,
        double targetAngleDegrees,
        String state) {
      if (!printedTurretHeader) {
        System.out.println(
            "TURRET_TABLE_HEADER,source,attempt,time_s,state,tag_id,raw_tx_deg,filtered_tx_deg,distance_m,current_angle_deg,target_angle_deg");
        printedTurretHeader = true;
      }

      if (lastTurretConsoleLogTimestamp >= 0
          && nowSeconds - lastTurretConsoleLogTimestamp < 0.25) {
        return;
      }

      lastTurretConsoleLogTimestamp = nowSeconds;
      System.out.printf(
          "TURRET_TABLE_ROW,TURRET,%d,%.3f,%s,%d,%.2f,%.2f,%.3f,%.2f,%.2f%n",
          activeTurretAttemptId,
          nowSeconds,
          state,
          scan.tagID,
          scan.tx,
          filteredTrackingTxDegrees,
          scan.distance,
          currentAngleDegrees,
          targetAngleDegrees);
    }
  }
