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

  private static final double SLOW_ZONE_DEGREES = 20.0;

  private static final double TRACK_DEADBAND_DEGREES = 0.25;
  private static final double TRACK_TX_FILTER_ALPHA = 0.45;
  private static final double TRACK_OUTPUT_SIGN = -1.0;
  private static final double TRACK_OUTPUT_KP = 0.022;
  private static final double TRACK_OUTPUT_MAX = 0.45;
  private static final double TRACK_OUTPUT_MIN = 0.05;
  private static final double TRACK_OUTPUT_MAX_STEP = 0.08;
  private static final int TRACK_MIN_ACQUIRE_SAMPLES = 1;
  private static final double TRACK_HOLD_LAST_VALID_SECONDS = 0.20;

  private double filteredTrackingTxDegrees = 0.0;
  private double lastTrackingTargetAngleDegrees = 0.0;
  private double lastTrackingOutput = 0.0;
  private int lastTrackingTagId = -1;
  private int consecutiveTrackingSamples = 0;
  private boolean hasTrackingSample = false;
  private double lastNoTargetWarningTimestamp = -1.0;
  private double lastBadDataWarningTimestamp = -1.0;
  private double lastTurretConsoleLogTimestamp = -1.0;
  private int activeTurretAttemptId = -1;
  private boolean printedTurretHeader = false;
  private LimeLight.AprilTagScan lastValidTrackingScan =
      new LimeLight.AprilTagScan(false, -1, 0.0, 0.0, 0.0, 0.0, new Pose3d());
  private double lastValidTrackingTimestampSeconds = -1.0;

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

  public double getLastTrackingOutput() {
    return lastTrackingOutput;
  }

  public double getFilteredTrackingTxDegrees() {
    return filteredTrackingTxDegrees;
  }

  public int getLastTrackingTagId() {
    return lastTrackingTagId;
  }

  public boolean hasTrackingSample() {
    return hasTrackingSample;
  }

  public double getLastValidTrackingAgeSeconds() {
    if (lastValidTrackingTimestampSeconds < 0.0) {
      return -1.0;
    }
    return Timer.getFPGATimestamp() - lastValidTrackingTimestampSeconds;
  }

  public boolean isTrackingHealthy() {
    return hasTrackingSample
        && lastTrackingTagId != -1
        && getLastValidTrackingAgeSeconds() >= 0.0
        && getLastValidTrackingAgeSeconds() <= TRACK_HOLD_LAST_VALID_SECONDS
        && Math.abs(filteredTrackingTxDegrees) <= 4.0;
  }

  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  public Command manualSet(Supplier<Double> dutyCycleSupplier) {
    return run(() -> {
      double rawInput = dutyCycleSupplier.get();
      double currentAngle = turret.getAngle().in(Degrees);
      double scaledOutput = applySoftLimitScaling(rawInput, currentAngle);
      spark.set(scaledOutput);
    }).withName("Turret.manualSet");
  }

  public Command trackTarget(LimeLight limelight) {
    return trackTarget(limelight, null, TRACKING_TAG_IDS);
  }

  public Command trackTarget(LimeLight limelight, SwerveSubsystem drivebase) {
    return trackTarget(limelight, drivebase, TRACKING_TAG_IDS);
  }

  public Command trackTarget(LimeLight limelight, SwerveSubsystem drivebase, Set<Integer> allowedTagIds) {
    return run(() -> {
      double executeStartSeconds = Timer.getFPGATimestamp();
      LimeLight.AprilTagScan scan = limelight.scanDirect(allowedTagIds);
      double currentAngleDegrees = turret.getAngle().in(Degrees);
      double nowSeconds = Timer.getFPGATimestamp();
      boolean usingHeldScan = false;
      boolean tagAllowedOrUnknown =
          allowedTagIds == null || scan.tagID == -1 || allowedTagIds.contains(scan.tagID);
      boolean hasTrackingTarget = scan.hasTarget && tagAllowedOrUnknown;
      double lastValidAgeSeconds = lastValidTrackingTimestampSeconds > 0.0
          ? nowSeconds - lastValidTrackingTimestampSeconds
          : -1.0;

      if (!hasTrackingTarget
          && lastValidTrackingScan.hasTarget
          && lastValidTrackingTimestampSeconds > 0.0
          && (nowSeconds - lastValidTrackingTimestampSeconds) <= TRACK_HOLD_LAST_VALID_SECONDS) {
        scan = lastValidTrackingScan;
        usingHeldScan = true;
        hasTrackingTarget = true;
      }

      if (!hasTrackingTarget) {
        filteredTrackingTxDegrees = 0.0;
        lastTrackingOutput = 0.0;
        spark.set(0.0);
        lastTrackingTagId = -1;
        consecutiveTrackingSamples = 0;
        hasTrackingSample = false;

        Logger.recordOutput("Turret/TrackingState", "NO_TARGET");
        Logger.recordOutput("Turret/TrackingTagID", -1);
        Logger.recordOutput("Turret/FilteredTX", 0.0);
        Logger.recordOutput("Turret/TrackingTargetAngleDegrees", currentAngleDegrees);
        Logger.recordOutput("Turret/TrackingDutyCycle", 0.0);
        Logger.recordOutput("Turret/TrackingLastValidAgeSec", lastValidAgeSeconds);
        Logger.recordOutput("Turret/TrackingHoldWindowSec", TRACK_HOLD_LAST_VALID_SECONDS);
        Logger.recordOutput("Turret/TrackingScanHasTarget", scan.hasTarget);
        Logger.recordOutput("Turret/TrackingTagAllowed", tagAllowedOrUnknown);
        Logger.recordOutput("Turret/TrackingUsingHeldSample", usingHeldScan);
        Logger.recordOutput("Turret/TrackingAllowedTagCount", allowedTagIds == null ? -1 : allowedTagIds.size());
        Logger.recordOutput("Turret/TrackingScanTX", scan.tx);
        Logger.recordOutput("Turret/TrackingScanTY", scan.ty);
        Logger.recordOutput("Turret/TrackingScanDistanceM", scan.distance);
        Logger.recordOutput("Turret/TrackingScanLatencyMs", scan.latencyMs);

        reportNoTargetWarning(scan, lastValidAgeSeconds);
        maybePrintTurretTelemetry(
            nowSeconds,
            scan,
            currentAngleDegrees,
            currentAngleDegrees,
            "NO_TARGET",
            "NO_TV",
            usingHeldScan,
            lastValidAgeSeconds,
            tagAllowedOrUnknown);
        Logger.recordOutput("Turret/TrackExecuteMs", (Timer.getFPGATimestamp() - executeStartSeconds) * 1000.0);
        return;
      }

      if (!usingHeldScan) {
        lastValidTrackingScan = new LimeLight.AprilTagScan(
            true,
            scan.tagID,
            scan.tx,
            scan.ty,
            scan.distance,
            scan.latencyMs,
            scan.pose);
        lastValidTrackingTimestampSeconds = nowSeconds;
      }

      if (!Double.isFinite(scan.tx) || !Double.isFinite(scan.ty) || !Double.isFinite(scan.distance)) {
        reportBadVisionDataWarning(scan);
        spark.set(0.0);
        maybePrintTurretTelemetry(
            nowSeconds,
            scan,
            currentAngleDegrees,
            currentAngleDegrees,
            "BAD_DATA",
            "NON_FINITE_SCAN",
            usingHeldScan,
            lastValidAgeSeconds,
            tagAllowedOrUnknown);
        Logger.recordOutput("Turret/TrackExecuteMs", (Timer.getFPGATimestamp() - executeStartSeconds) * 1000.0);
        return;
      }

      boolean tagChanged = scan.tagID != lastTrackingTagId;
      if (tagChanged) {
        consecutiveTrackingSamples = 0;
        hasTrackingSample = false;
        lastTrackingTagId = scan.tagID;
      }

      double rawTxDegrees = scan.tx;
      if (!hasTrackingSample) {
        filteredTrackingTxDegrees = rawTxDegrees;
        consecutiveTrackingSamples++;
        Logger.recordOutput("Turret/TrackingState", "ACQUIRING");
        Logger.recordOutput("Turret/TrackingAcquireSamples", consecutiveTrackingSamples);
        Logger.recordOutput("Turret/TrackingTagID", scan.tagID);
        maybePrintTurretTelemetry(
            nowSeconds,
            scan,
            currentAngleDegrees,
            currentAngleDegrees,
            "ACQUIRING",
            "NONE",
            usingHeldScan,
            lastValidAgeSeconds,
            tagAllowedOrUnknown);
        if (consecutiveTrackingSamples < TRACK_MIN_ACQUIRE_SAMPLES) {
          spark.set(0.0);
          Logger.recordOutput("Turret/TrackExecuteMs", (Timer.getFPGATimestamp() - executeStartSeconds) * 1000.0);
          return;
        }
        hasTrackingSample = true;
      }

      filteredTrackingTxDegrees +=
          (rawTxDegrees - filteredTrackingTxDegrees) * TRACK_TX_FILTER_ALPHA;

      if (Math.abs(filteredTrackingTxDegrees) < TRACK_DEADBAND_DEGREES) {
        filteredTrackingTxDegrees = 0.0;
      }

      double desiredTargetAngleDegrees = MathUtil.clamp(
          TRACK_OUTPUT_SIGN * filteredTrackingTxDegrees,
          -MAX_ONE_DIR_FOV,
          MAX_ONE_DIR_FOV);
      lastTrackingTargetAngleDegrees = desiredTargetAngleDegrees;

      double dutyCycle = TRACK_OUTPUT_SIGN * filteredTrackingTxDegrees * TRACK_OUTPUT_KP;
      if (Math.abs(filteredTrackingTxDegrees) > TRACK_DEADBAND_DEGREES
          && Math.abs(dutyCycle) < TRACK_OUTPUT_MIN) {
        dutyCycle = Math.copySign(TRACK_OUTPUT_MIN, dutyCycle);
      }
      dutyCycle = MathUtil.clamp(dutyCycle, -TRACK_OUTPUT_MAX, TRACK_OUTPUT_MAX);
      dutyCycle = MathUtil.clamp(
          dutyCycle,
          lastTrackingOutput - TRACK_OUTPUT_MAX_STEP,
          lastTrackingOutput + TRACK_OUTPUT_MAX_STEP);
      dutyCycle = applySoftLimitScaling(dutyCycle, currentAngleDegrees);
      lastTrackingOutput = dutyCycle;
      spark.set(dutyCycle);

      Logger.recordOutput("Turret/RawTX", scan.tx);
      Logger.recordOutput("Turret/FilteredTX", filteredTrackingTxDegrees);
      Logger.recordOutput("Turret/TrackingAcquireSamples", consecutiveTrackingSamples);
      Logger.recordOutput("Turret/TrackingTagID", scan.tagID);
      Logger.recordOutput("Turret/TargetDistanceM", scan.distance);
      Logger.recordOutput("Turret/TrackingTargetAngleDegrees", desiredTargetAngleDegrees);
      Logger.recordOutput("Turret/TrackingDutyCycle", dutyCycle);
      Logger.recordOutput("Turret/TrackingOutputMaxStep", TRACK_OUTPUT_MAX_STEP);
      Logger.recordOutput("Turret/UsingHeldTrackingSample", usingHeldScan);
      Logger.recordOutput("Turret/TrackingLastValidAgeSec", lastValidAgeSeconds);
      Logger.recordOutput("Turret/TrackingTagAllowed", tagAllowedOrUnknown);
      Logger.recordOutput("Turret/TrackingOutputSign", TRACK_OUTPUT_SIGN);
      Logger.recordOutput("Turret/TrackingState", "TRACKING");

      maybePrintTurretTelemetry(
          nowSeconds,
          scan,
          currentAngleDegrees,
          desiredTargetAngleDegrees,
          "TRACKING",
          "NONE",
          usingHeldScan,
          lastValidAgeSeconds,
          tagAllowedOrUnknown);

      Logger.recordOutput("Turret/TrackExecuteMs", (Timer.getFPGATimestamp() - executeStartSeconds) * 1000.0);
    }).beforeStarting(() -> {
      activeTurretAttemptId = LimelightAttemptTracker.nextAttemptId();
      lastTurretConsoleLogTimestamp = -1.0;
      System.out.printf(
          "LIMELIGHT_ATTEMPT_START,source=TURRET,attempt=%d%n",
          activeTurretAttemptId);
    }).finallyDo(() -> {
      spark.set(0.0);
      System.out.printf(
          "LIMELIGHT_ATTEMPT_END,source=TURRET,attempt=%d%n",
          activeTurretAttemptId);
      activeTurretAttemptId = -1;
      lastTurretConsoleLogTimestamp = -1.0;
      lastNoTargetWarningTimestamp = -1.0;
      lastBadDataWarningTimestamp = -1.0;
      lastTrackingOutput = 0.0;
      lastValidTrackingScan = new LimeLight.AprilTagScan(false, -1, 0.0, 0.0, 0.0, 0.0, new Pose3d());
      lastValidTrackingTimestampSeconds = -1.0;
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
        new Pose3d(turretTranslation, new Rotation3d(0, 0, turret.getAngle().in(Radians))) });
    Logger.recordOutput("Turret/RawEncoderRotations", spark.getEncoder().getPosition());
    Logger.recordOutput("Turret/YAMSAngleDegrees", turret.getAngle().in(Degrees));
    Logger.recordOutput("Turret/LastTrackingOutput", lastTrackingOutput);
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }

  private void reportNoTargetWarning(LimeLight.AprilTagScan scan, double lastValidAgeSeconds) {
    double nowSeconds = Timer.getFPGATimestamp();
    if (lastNoTargetWarningTimestamp < 0 || nowSeconds - lastNoTargetWarningTimestamp >= 1.0) {
      DriverStation.reportWarning(
          String.format(
              "[TURRET][WARN][ATTEMPT %d] Lost/invalid Limelight target while tracking (tagId=%d, lastValidAgeSec=%.3f).",
              activeTurretAttemptId,
              scan.tagID,
              lastValidAgeSeconds),
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

  private void maybePrintTurretTelemetry(
      double nowSeconds,
      LimeLight.AprilTagScan scan,
      double currentAngleDegrees,
      double targetAngleDegrees,
      String state,
      String reason,
      boolean usingHeldScan,
      double lastValidAgeSeconds,
      boolean tagAllowedOrUnknown) {
    if (!printedTurretHeader) {
      System.out.println(
          "TURRET_TABLE_HEADER,source,attempt,time_s,state,reason,tag_id,raw_tx_deg,filtered_tx_deg,distance_m,current_angle_deg,target_angle_deg,using_held,last_valid_age_s,tag_allowed");
      printedTurretHeader = true;
    }

    if (lastTurretConsoleLogTimestamp >= 0
        && nowSeconds - lastTurretConsoleLogTimestamp < 0.25) {
      return;
    }

    lastTurretConsoleLogTimestamp = nowSeconds;
    System.out.printf(
        "TURRET_TABLE_ROW,TURRET,%d,%.3f,%s,%s,%d,%.2f,%.2f,%.3f,%.2f,%.2f,%b,%.3f,%b%n",
        activeTurretAttemptId,
        nowSeconds,
        state,
        reason,
        scan.tagID,
        scan.tx,
        filteredTrackingTxDegrees,
        scan.distance,
        currentAngleDegrees,
        targetAngleDegrees,
        usingHeldScan,
        lastValidAgeSeconds,
        tagAllowedOrUnknown);
  }
}
