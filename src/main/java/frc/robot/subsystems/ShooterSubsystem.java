package frc.robot.subsystems;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import frc.robot.subsystems.LimeLight;

public class ShooterSubsystem extends SubsystemBase {
  private static final double SHOOTER_MAX_RPM = 5600.0;
  private static final double SHOOTER_SPINUP_RPM = 3600.0;
  private static final double LIMELIGHT_SHOT_MIN_RPM = 3200.0;
  private static final double LIMELIGHT_SHOT_MAX_RPM = 4800.0;
  private static final double LIMELIGHT_REFERENCE_DISTANCE_METERS = 3.5052; // 11 ft 6 in
  private static final double LIMELIGHT_REFERENCE_RPM = 3300.0;
  private static final double LIMELIGHT_RPM_PER_METER = 450.0;
  private static final double LIMELIGHT_RPM_TRIM = -100.0;
  private static final double LIMELIGHT_RPM_SCALE = 0.90;
  private static final double SHOOTING_LIMELIGHT_MOUNT_ANGLE_DEGREES = 20.0; // tune to your real mount
  private static final double LIMELIGHT_MIN_VALID_DISTANCE_METERS = 0.8;
  private static final double LIMELIGHT_MAX_VALID_DISTANCE_METERS = 8.0;
  private static final double INVALID_TARGET_WARNING_PERIOD_SECONDS = 5.0;
  private static final double LIMELIGHT_FEEDBACK_STEP_RPM = 75.0;
  private static final double FEEDBACK_WINDOW_SECONDS = 10.0;
  private static final double DISTANCE_BUCKET_METERS = 0.5;
  private static final int DISTANCE_BUCKET_COUNT = 20;
  private static final double DEFAULT_RANGE_HALF_WIDTH_RPM = 175.0;
  private static final int MIN_BUCKETS_FOR_MARKDOWN_TABLE = 3;

  private final SparkMax leaderSpark = new SparkMax(Constants.ShooterConstants.kLeaderMotorId,
      MotorType.kBrushless);

  private final SparkMax followerSpark = new SparkMax(Constants.ShooterConstants.kFollowerMotorId,
      MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(followerSpark, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.00936, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.30))
      .withOpenLoopRampRate(Seconds.of(0.30));

  private final SmartMotorController smc = new SparkWrapper(leaderSpark, DCMotor.getNEO(2), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(SHOOTER_MAX_RPM))
      .withLowerSoftLimit(RPM.of(0))
      .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  private final FlyWheel shooter = new FlyWheel(shooterConfig);
  private boolean printedShotHeader = false;
  private boolean wasLimelightShotValidLastCycle = true;
  private double lastInvalidTargetWarningTimestamp = -1.0;
  private int activeShooterAttemptId = -1;
  private boolean shooterAttemptHasValidSample = false;
  private boolean warnedInvalidTargetThisAttempt = false;
  private int shooterAttemptLastDistanceBucket = -1;
  private int pendingFeedbackBucket = -1;
  private double pendingFeedbackExpirySeconds = -1.0;
  private boolean pendingFeedbackAdjusted = false;
  private final double[] limelightDistanceTrimRpm = new double[DISTANCE_BUCKET_COUNT];
  private final double[] tableRpmSum = new double[DISTANCE_BUCKET_COUNT];
  private final double[] tableRpmMin = new double[DISTANCE_BUCKET_COUNT];
  private final double[] tableRpmMax = new double[DISTANCE_BUCKET_COUNT];
  private final int[] tableSampleCount = new int[DISTANCE_BUCKET_COUNT];
  private double shooterAttemptLastTimestampSeconds = 0.0;
  private int shooterAttemptLastTagId = -1;
  private double shooterAttemptLastDistanceMeters = 0.0;
  private double shooterAttemptLastEquationRpm = 0.0;
  private double shooterAttemptLastCommandedRpm = 0.0;
  private double shooterAttemptLastActualRpm = 0.0;
  private double shooterAttemptLastTxDegrees = 0.0;
  private double shooterAttemptLastTyDegrees = 0.0;
  private double shooterAttemptLastLaunchAngleDegrees = 0.0;
  private double pendingFeedbackDistanceMeters = 0.0;
  private double pendingFeedbackCommandedRpm = 0.0;

  public ShooterSubsystem() {
    loadPersistentDistanceTrims();
    loadPersistentShotTable();
  }

  public Command setSpeed(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
    return shooter.setSpeed(speedSupplier);
  }

  public Command setPercent(Supplier<Double> percentSupplier) {
    return run(() -> leaderSpark.set(percentSupplier.get()))
        .finallyDo(() -> leaderSpark.set(0)); // stops motor on release
}
public Command waitCommand() {
    return new WaitCommand(2.0);
}


public Command setPercentAsRPM(Supplier<Double> percentSupplier) {
    return shooter.setSpeed(
        () -> RPM.of(percentSupplier.get() * SHOOTER_MAX_RPM)
    ).finallyDo(() -> leaderSpark.set(0));
}
  public Command spinUp() {
    return setSpeed(RPM.of(3200));
  }

  public Command stop() {
    return setSpeed(RPM.of(0));
  }
  

  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
  }

  public Command sysId() {
    return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
  }
  
  public Command setSpeedFromLimelight(LimeLight limelight, double fixedLaunchAngleDegrees) {
    return setSpeedFromLimelight(limelight, fixedLaunchAngleDegrees, null);
  }

  public Command setSpeedFromLimelight(
      LimeLight limelight,
      double fixedLaunchAngleDegrees,
      Set<Integer> allowedTagIds) {
    return shooter.setSpeed(() -> {
        double nowSeconds = Timer.getFPGATimestamp();
        LimeLight.AprilTagScan scan =
            allowedTagIds == null ? limelight.scan() : limelight.scan(allowedTagIds);

        if (!scan.isValid()) {
            Logger.recordOutput("Shooter/LimelightShotValid", false);
            Logger.recordOutput("Shooter/LimelightShotMessage", "No valid hub AprilTag target");
            Logger.recordOutput("Shooter/LimelightDistanceMeters", 0.0);
            Logger.recordOutput("Shooter/LimelightEquationRPM", 0.0);
            Logger.recordOutput("Shooter/LimelightCommandedRPM", 0.0);
            Logger.recordOutput("Shooter/LimelightTargetTagID", -1);
            maybeReportInvalidTarget(nowSeconds);
            wasLimelightShotValidLastCycle = false;
            return RPM.of(0);
        }

        double measuredDistanceMeters = estimateTargetDistanceMeters(scan);
        double distanceMeters = measuredDistanceMeters;
        boolean distanceOutOfRange =
            measuredDistanceMeters < LIMELIGHT_MIN_VALID_DISTANCE_METERS
                || measuredDistanceMeters > LIMELIGHT_MAX_VALID_DISTANCE_METERS;
        if (distanceOutOfRange) {
          distanceMeters = LIMELIGHT_REFERENCE_DISTANCE_METERS;
        }
        int distanceBucket = bucketForDistance(distanceMeters);
        double distanceTrimRpm = limelightDistanceTrimRpm[distanceBucket];
        double equationRPM = rpmForDistanceMeters(distanceMeters) + distanceTrimRpm;
        double scaledEquationRPM = equationRPM * LIMELIGHT_RPM_SCALE;
        TableEstimate tableEstimate = estimateFromShotTable(distanceMeters, scaledEquationRPM);
        double rangedRecommended = MathUtil.clamp(
            tableEstimate.recommendedRpm,
            tableEstimate.rangeMinRpm,
            tableEstimate.rangeMaxRpm);
        double commandedRPM = MathUtil.clamp(rangedRecommended, LIMELIGHT_SHOT_MIN_RPM, LIMELIGHT_SHOT_MAX_RPM);

        Logger.recordOutput("Shooter/LimelightShotValid", true);
        Logger.recordOutput("Shooter/LimelightShotMessage", "Scaled from 11ft 6in reference shot");
        Logger.recordOutput("Shooter/LimelightEstimatedRPM", scaledEquationRPM);
        Logger.recordOutput("Shooter/LimelightEquationRPM", scaledEquationRPM);
        Logger.recordOutput("Shooter/LimelightCommandedRPM", commandedRPM);
        Logger.recordOutput("Shooter/LimelightDistanceMeters", distanceMeters);
        Logger.recordOutput("Shooter/LimelightMeasuredDistanceMeters", measuredDistanceMeters);
        Logger.recordOutput("Shooter/LimelightDistanceOutOfRange", distanceOutOfRange);
        Logger.recordOutput("Shooter/LimelightDistanceBucket", distanceBucket);
        Logger.recordOutput("Shooter/LimelightDistanceTrimRPM", distanceTrimRpm);
        Logger.recordOutput("Shooter/LimelightTableRecommendedRPM", tableEstimate.recommendedRpm);
        Logger.recordOutput("Shooter/LimelightTableRangeMinRPM", tableEstimate.rangeMinRpm);
        Logger.recordOutput("Shooter/LimelightTableRangeMaxRPM", tableEstimate.rangeMaxRpm);
        Logger.recordOutput("Shooter/LimelightTableInterpolated", tableEstimate.interpolated);
        Logger.recordOutput("Shooter/LimelightReferenceDistanceMeters", LIMELIGHT_REFERENCE_DISTANCE_METERS);
        Logger.recordOutput("Shooter/LimelightReferenceRPM", LIMELIGHT_REFERENCE_RPM);
        Logger.recordOutput("Shooter/LimelightRpmPerMeter", LIMELIGHT_RPM_PER_METER);
        Logger.recordOutput("Shooter/LimelightRpmTrim", LIMELIGHT_RPM_TRIM);
        Logger.recordOutput("Shooter/LimelightRpmScale", LIMELIGHT_RPM_SCALE);
        Logger.recordOutput("Shooter/LimelightShotMaxRPM", LIMELIGHT_SHOT_MAX_RPM);
        Logger.recordOutput("Shooter/LimelightTargetTagID", scan.tagID);
        cacheShooterAttemptSample(
            nowSeconds,
            scan,
            distanceMeters,
            tableEstimate.recommendedRpm,
            commandedRPM,
            fixedLaunchAngleDegrees,
            distanceBucket);
        wasLimelightShotValidLastCycle = true;

        return RPM.of(commandedRPM);
    }).beforeStarting(() -> {
      activeShooterAttemptId = LimelightAttemptTracker.nextAttemptId();
      lastInvalidTargetWarningTimestamp = -1.0;
      wasLimelightShotValidLastCycle = true;
      shooterAttemptHasValidSample = false;
      warnedInvalidTargetThisAttempt = false;
      System.out.printf(
          "LIMELIGHT_ATTEMPT_START,source=SHOOTER,attempt=%d%n",
          activeShooterAttemptId);
    }).finallyDo(() -> {
      leaderSpark.set(0);
      wasLimelightShotValidLastCycle = true;
      lastInvalidTargetWarningTimestamp = -1.0;
      if (shooterAttemptHasValidSample) {
        printShotTelemetryRow();
        pendingFeedbackBucket = shooterAttemptLastDistanceBucket;
        pendingFeedbackExpirySeconds = Timer.getFPGATimestamp() + FEEDBACK_WINDOW_SECONDS;
        pendingFeedbackAdjusted = false;
        pendingFeedbackDistanceMeters = shooterAttemptLastDistanceMeters;
        pendingFeedbackCommandedRpm = shooterAttemptLastCommandedRpm;
        System.out.printf(
            "SHOT_FEEDBACK_WINDOW_OPEN,source=SHOOTER,attempt=%d,bucket=%d,expires_s=%.3f,current_trim_rpm=%.1f%n",
            activeShooterAttemptId,
            pendingFeedbackBucket,
            pendingFeedbackExpirySeconds,
            limelightDistanceTrimRpm[pendingFeedbackBucket]);
      }
      System.out.printf(
          "LIMELIGHT_ATTEMPT_END,source=SHOOTER,attempt=%d%n",
          activeShooterAttemptId);
      activeShooterAttemptId = -1;
      shooterAttemptHasValidSample = false;
      warnedInvalidTargetThisAttempt = false;
      pendingFeedbackDistanceMeters = 0.0;
      pendingFeedbackCommandedRpm = 0.0;
    });
}

  private double estimateTargetDistanceMeters(LimeLight limelight) {
    return estimateTargetDistanceMeters(limelight.scan());
  }

  private double estimateTargetDistanceMeters(LimeLight.AprilTagScan scan) {
    if (!scan.isValid()) {
      return 0.0;
    }

    double ty = scan.ty;
    double totalAngleRad = Math.toRadians(SHOOTING_LIMELIGHT_MOUNT_ANGLE_DEGREES + ty);
    if (Math.abs(totalAngleRad) < 1e-6) {
      return 0.0;
    }

    double distanceMeters =
        (ProjectileMotion.HUB_APRILTAG_HEIGHT_METERS - ProjectileMotion.LIMELIGHT_MOUNT_HEIGHT_METERS)
            / Math.tan(totalAngleRad);
    return Math.max(0.0, distanceMeters);
  }

  private double rpmForDistanceMeters(double distanceMeters) {
    if (distanceMeters <= 0.0) {
      return 0.0;
    }

    double distanceOffsetMeters = distanceMeters - LIMELIGHT_REFERENCE_DISTANCE_METERS;
    return LIMELIGHT_REFERENCE_RPM + (distanceOffsetMeters * LIMELIGHT_RPM_PER_METER) + LIMELIGHT_RPM_TRIM;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/LeaderVelocity", leaderSpark.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/FollowerVelocity", followerSpark.getEncoder().getVelocity());
    maybeFinalizeFeedbackWindow();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  private void maybeReportInvalidTarget(double nowSeconds) {
    int attemptId = activeShooterAttemptId;
    if (attemptId < 0) {
      return;
    }

    if (wasLimelightShotValidLastCycle) {
      DriverStation.reportWarning(
          String.format(
              "[SHOT][WARN][SHOOTER][ATTEMPT %d] Limelight target invalid during shoot command (no approved tag / no target).",
              attemptId),
          false);
      lastInvalidTargetWarningTimestamp = nowSeconds;
      warnedInvalidTargetThisAttempt = true;
      return;
    }

    if (warnedInvalidTargetThisAttempt
        && (lastInvalidTargetWarningTimestamp < 0
            || nowSeconds - lastInvalidTargetWarningTimestamp >= INVALID_TARGET_WARNING_PERIOD_SECONDS)) {
      DriverStation.reportWarning(
          String.format(
              "[SHOT][WARN][SHOOTER][ATTEMPT %d] Still no valid Limelight target while shooter command is active.",
              attemptId),
          false);
      lastInvalidTargetWarningTimestamp = nowSeconds;
    }
  }

  private void cacheShooterAttemptSample(
      double nowSeconds,
      LimeLight.AprilTagScan scan,
      double distanceMeters,
      double equationRPM,
      double commandedRPM,
      double fixedLaunchAngleDegrees,
      int distanceBucket) {
    shooterAttemptHasValidSample = true;
    shooterAttemptLastTimestampSeconds = nowSeconds;
    shooterAttemptLastTagId = scan.tagID;
    shooterAttemptLastDistanceMeters = distanceMeters;
    shooterAttemptLastEquationRpm = equationRPM;
    shooterAttemptLastCommandedRpm = commandedRPM;
    shooterAttemptLastActualRpm = getSpeed().in(RPM);
    shooterAttemptLastTxDegrees = scan.tx;
    shooterAttemptLastTyDegrees = scan.ty;
    shooterAttemptLastLaunchAngleDegrees = fixedLaunchAngleDegrees;
    shooterAttemptLastDistanceBucket = distanceBucket;
  }

  private void printShotTelemetryRow() {
    if (!printedShotHeader) {
      System.out.println(
          "SHOT_TABLE_HEADER,source,attempt,time_s,tag_id,distance_m,equation_rpm,commanded_rpm,actual_rpm,tx_deg,ty_deg,launch_angle_deg");
      printedShotHeader = true;
    }

    System.out.printf(
        "SHOT_TABLE_ROW,SHOOTER,%d,%.3f,%d,%.3f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f%n",
        activeShooterAttemptId,
        shooterAttemptLastTimestampSeconds,
        shooterAttemptLastTagId,
        shooterAttemptLastDistanceMeters,
        shooterAttemptLastEquationRpm,
        shooterAttemptLastCommandedRpm,
        shooterAttemptLastActualRpm,
        shooterAttemptLastTxDegrees,
        shooterAttemptLastTyDegrees,
        shooterAttemptLastLaunchAngleDegrees);
  }

  public Command increaseLimelightPowerForLastShotCommand() {
    return runOnce(() -> applyFeedbackTrim(LIMELIGHT_FEEDBACK_STEP_RPM));
  }

  public Command decreaseLimelightPowerForLastShotCommand() {
    return runOnce(() -> applyFeedbackTrim(-LIMELIGHT_FEEDBACK_STEP_RPM));
  }

  public Command confirmMadeShotCommand() {
    return runOnce(this::confirmMadeShotNow);
  }

  private void applyFeedbackTrim(double deltaRpm) {
    double nowSeconds = Timer.getFPGATimestamp();
    int bucket = getActiveFeedbackBucket(nowSeconds);
    if (bucket < 0) {
      DriverStation.reportWarning(
          String.format(
              "[SHOT][FEEDBACK] No active feedback window. Shoot first, then adjust within %.0fs.",
              FEEDBACK_WINDOW_SECONDS),
          false);
      return;
    }

    limelightDistanceTrimRpm[bucket] += deltaRpm;
    savePersistentDistanceTrim(bucket);
    pendingFeedbackAdjusted = true;
    pendingFeedbackExpirySeconds = nowSeconds + FEEDBACK_WINDOW_SECONDS;
    System.out.printf(
        "SHOT_FEEDBACK_APPLIED,source=SHOOTER,bucket=%d,delta_rpm=%.1f,new_trim_rpm=%.1f,window_expires_s=%.3f%n",
        bucket,
        deltaRpm,
        limelightDistanceTrimRpm[bucket],
        pendingFeedbackExpirySeconds);
  }

  private int getActiveFeedbackBucket(double nowSeconds) {
    if (pendingFeedbackBucket >= 0 && nowSeconds <= pendingFeedbackExpirySeconds) {
      return pendingFeedbackBucket;
    }

    if (activeShooterAttemptId >= 0 && shooterAttemptLastDistanceBucket >= 0) {
      return shooterAttemptLastDistanceBucket;
    }

    return -1;
  }

  private void maybeFinalizeFeedbackWindow() {
    if (pendingFeedbackBucket < 0) {
      return;
    }

    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds <= pendingFeedbackExpirySeconds) {
      return;
    }

    if (!pendingFeedbackAdjusted && pendingFeedbackCommandedRpm > 0.0) {
      recordConfirmedShot(pendingFeedbackBucket, pendingFeedbackDistanceMeters, pendingFeedbackCommandedRpm);
    }
    System.out.printf(
        "SHOT_FEEDBACK_WINDOW_CLOSED,source=SHOOTER,bucket=%d,accepted_trim_rpm=%.1f,adjusted=%b,confirmed_rpm=%.1f%n",
        pendingFeedbackBucket,
        limelightDistanceTrimRpm[pendingFeedbackBucket],
        pendingFeedbackAdjusted,
        pendingFeedbackCommandedRpm);
    pendingFeedbackBucket = -1;
    pendingFeedbackExpirySeconds = -1.0;
    pendingFeedbackAdjusted = false;
    pendingFeedbackDistanceMeters = 0.0;
    pendingFeedbackCommandedRpm = 0.0;
  }

  private void confirmMadeShotNow() {
    if (!isFeedbackWindowOpen()) {
      DriverStation.reportWarning(
          "[SHOT][FEEDBACK] No active feedback window to confirm.",
          false);
      return;
    }

    recordConfirmedShot(
        pendingFeedbackBucket,
        pendingFeedbackDistanceMeters,
        pendingFeedbackCommandedRpm);
    System.out.printf(
        "SHOT_FEEDBACK_CONFIRMED_NOW,source=SHOOTER,bucket=%d,confirmed_rpm=%.1f%n",
        pendingFeedbackBucket,
        pendingFeedbackCommandedRpm);
    pendingFeedbackBucket = -1;
    pendingFeedbackExpirySeconds = -1.0;
    pendingFeedbackAdjusted = false;
    pendingFeedbackDistanceMeters = 0.0;
    pendingFeedbackCommandedRpm = 0.0;
  }

  private int bucketForDistance(double distanceMeters) {
    int bucket = (int) Math.round(distanceMeters / DISTANCE_BUCKET_METERS);
    return MathUtil.clamp(bucket, 0, DISTANCE_BUCKET_COUNT - 1);
  }

  private void loadPersistentDistanceTrims() {
    for (int i = 0; i < DISTANCE_BUCKET_COUNT; i++) {
      limelightDistanceTrimRpm[i] = Preferences.getDouble(distanceTrimPreferenceKey(i), 0.0);
    }
  }

  private void savePersistentDistanceTrim(int bucket) {
    Preferences.setDouble(distanceTrimPreferenceKey(bucket), limelightDistanceTrimRpm[bucket]);
  }

  private String distanceTrimPreferenceKey(int bucket) {
    return "Shooter.LimelightDistanceTrimRpm." + bucket;
  }

  private TableEstimate estimateFromShotTable(double distanceMeters, double fallbackRpm) {
    int bucket = bucketForDistance(distanceMeters);
    if (tableSampleCount[bucket] > 0) {
      double avg = tableRpmSum[bucket] / tableSampleCount[bucket];
      return new TableEstimate(avg, tableRpmMin[bucket], tableRpmMax[bucket], false);
    }

    int lower = findLowerBucketWithData(bucket);
    int upper = findUpperBucketWithData(bucket);

    if (lower >= 0 && upper >= 0 && lower != upper) {
      double lowerDist = bucketCenterDistanceMeters(lower);
      double upperDist = bucketCenterDistanceMeters(upper);
      double t = MathUtil.clamp(
          (distanceMeters - lowerDist) / Math.max(upperDist - lowerDist, 1e-6),
          0.0,
          1.0);
      double lowerAvg = tableRpmSum[lower] / tableSampleCount[lower];
      double upperAvg = tableRpmSum[upper] / tableSampleCount[upper];
      double recommended = lerp(lowerAvg, upperAvg, t);
      double rangeMin = lerp(tableRpmMin[lower], tableRpmMin[upper], t);
      double rangeMax = lerp(tableRpmMax[lower], tableRpmMax[upper], t);
      return new TableEstimate(recommended, rangeMin, rangeMax, true);
    }

    if (lower >= 0) {
      double avg = tableRpmSum[lower] / tableSampleCount[lower];
      return new TableEstimate(avg, tableRpmMin[lower], tableRpmMax[lower], true);
    }

    if (upper >= 0) {
      double avg = tableRpmSum[upper] / tableSampleCount[upper];
      return new TableEstimate(avg, tableRpmMin[upper], tableRpmMax[upper], true);
    }

    return new TableEstimate(
        fallbackRpm,
        fallbackRpm - DEFAULT_RANGE_HALF_WIDTH_RPM,
        fallbackRpm + DEFAULT_RANGE_HALF_WIDTH_RPM,
        false);
  }

  private int findLowerBucketWithData(int bucket) {
    for (int i = bucket - 1; i >= 0; i--) {
      if (tableSampleCount[i] > 0) {
        return i;
      }
    }
    return -1;
  }

  private int findUpperBucketWithData(int bucket) {
    for (int i = bucket + 1; i < DISTANCE_BUCKET_COUNT; i++) {
      if (tableSampleCount[i] > 0) {
        return i;
      }
    }
    return -1;
  }

  private void recordConfirmedShot(int bucket, double distanceMeters, double commandedRpm) {
    double rpm = MathUtil.clamp(commandedRpm, LIMELIGHT_SHOT_MIN_RPM, LIMELIGHT_SHOT_MAX_RPM);
    tableRpmSum[bucket] += rpm;
    tableSampleCount[bucket]++;
    tableRpmMin[bucket] = tableSampleCount[bucket] == 1 ? rpm : Math.min(tableRpmMin[bucket], rpm);
    tableRpmMax[bucket] = tableSampleCount[bucket] == 1 ? rpm : Math.max(tableRpmMax[bucket], rpm);
    savePersistentShotTableBucket(bucket);

    System.out.printf(
        "SHOT_TABLE_CONFIRMED,source=SHOOTER,bucket=%d,distance_m=%.3f,confirmed_rpm=%.1f,samples=%d,range_min=%.1f,range_max=%.1f%n",
        bucket,
        distanceMeters,
        rpm,
        tableSampleCount[bucket],
        tableRpmMin[bucket],
        tableRpmMax[bucket]);
    printShotTableSummary();
  }

  private void loadPersistentShotTable() {
    for (int i = 0; i < DISTANCE_BUCKET_COUNT; i++) {
      tableRpmSum[i] = Preferences.getDouble(shotTableSumPreferenceKey(i), 0.0);
      tableSampleCount[i] = (int) Math.round(Preferences.getDouble(shotTableCountPreferenceKey(i), 0.0));
      tableRpmMin[i] = Preferences.getDouble(shotTableMinPreferenceKey(i), 0.0);
      tableRpmMax[i] = Preferences.getDouble(shotTableMaxPreferenceKey(i), 0.0);
    }
  }

  private void savePersistentShotTableBucket(int bucket) {
    Preferences.setDouble(shotTableSumPreferenceKey(bucket), tableRpmSum[bucket]);
    Preferences.setDouble(shotTableCountPreferenceKey(bucket), tableSampleCount[bucket]);
    Preferences.setDouble(shotTableMinPreferenceKey(bucket), tableRpmMin[bucket]);
    Preferences.setDouble(shotTableMaxPreferenceKey(bucket), tableRpmMax[bucket]);
  }

  private void printShotTableSummary() {
    int populatedBuckets = 0;
    for (int i = 0; i < DISTANCE_BUCKET_COUNT; i++) {
      if (tableSampleCount[i] > 0) {
        populatedBuckets++;
      }
    }

    if (populatedBuckets < MIN_BUCKETS_FOR_MARKDOWN_TABLE) {
      System.out.printf(
          "SHOT_TABLE_STATUS,source=SHOOTER,buckets_with_data=%d,need=%d_for_markdown%n",
          populatedBuckets,
          MIN_BUCKETS_FOR_MARKDOWN_TABLE);
      return;
    }

    System.out.println("SHOT_TABLE_MARKDOWN_BEGIN");
    System.out.println("| Distance (m) | RPM Min | RPM Avg | RPM Max | Samples |");
    System.out.println("| --- | --- | --- | --- | --- |");
    for (int i = 0; i < DISTANCE_BUCKET_COUNT; i++) {
      if (tableSampleCount[i] <= 0) {
        continue;
      }
      double avg = tableRpmSum[i] / tableSampleCount[i];
      System.out.printf(
          "| %.2f | %.1f | %.1f | %.1f | %d |%n",
          bucketCenterDistanceMeters(i),
          tableRpmMin[i],
          avg,
          tableRpmMax[i],
          tableSampleCount[i]);
    }
    System.out.println("SHOT_TABLE_MARKDOWN_END");
  }

  private double bucketCenterDistanceMeters(int bucket) {
    return bucket * DISTANCE_BUCKET_METERS;
  }

  private double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private String shotTableSumPreferenceKey(int bucket) {
    return "Shooter.ShotTable.Sum." + bucket;
  }

  private String shotTableCountPreferenceKey(int bucket) {
    return "Shooter.ShotTable.Count." + bucket;
  }

  private String shotTableMinPreferenceKey(int bucket) {
    return "Shooter.ShotTable.Min." + bucket;
  }

  private String shotTableMaxPreferenceKey(int bucket) {
    return "Shooter.ShotTable.Max." + bucket;
  }

  private static class TableEstimate {
    final double recommendedRpm;
    final double rangeMinRpm;
    final double rangeMaxRpm;
    final boolean interpolated;

    TableEstimate(double recommendedRpm, double rangeMinRpm, double rangeMaxRpm, boolean interpolated) {
      this.recommendedRpm = recommendedRpm;
      this.rangeMinRpm = rangeMinRpm;
      this.rangeMaxRpm = rangeMaxRpm;
      this.interpolated = interpolated;
    }
  }

  public boolean isFeedbackWindowOpen() {
    return pendingFeedbackBucket >= 0 && Timer.getFPGATimestamp() <= pendingFeedbackExpirySeconds;
  }

  public double getFeedbackWindowSecondsRemaining() {
    if (!isFeedbackWindowOpen()) {
      return 0.0;
    }
    return Math.max(0.0, pendingFeedbackExpirySeconds - Timer.getFPGATimestamp());
  }

  private Distance wheelRadius() {
    return Inches.of(4).div(2);
  }

  public LinearVelocity getTangentialVelocity() {
    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
  }
}
