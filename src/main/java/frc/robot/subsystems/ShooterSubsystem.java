package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
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

/**
 * Shooter flywheel and the vision-driven shot power model.
 *
 * <h2>How shot power is chosen</h2>
 *
 * <ol>
 *   <li>A caller supplies the range from the ball exit point to the goal.
 *   <li>{@link ProjectileMotion#flywheelRpmForDistance} turns that into an RPM using projectile
 *       physics scaled by one efficiency factor derived from the team's measured reference shot.
 *   <li>Any per-distance trim the drivers have dialled in is added.
 *   <li>If drivers have confirmed real scoring shots at that range, the learned table wins over the
 *       model entirely.
 * </ol>
 *
 * <p>The model exists so the untuned baseline is sensible everywhere; the table exists because the
 * real robot always disagrees with physics somewhere. Neither one alone was enough.
 */
public class ShooterSubsystem extends SubsystemBase {
  /**
   * Speed for the manual spin-up on the operator trigger.
   *
   * <p>Was 3600, which was well above the calibrated reference shot and above the vision clamp
   * ceiling — so pulling the trigger fired noticeably harder than any vision-aimed shot. Matched
   * to the reference so manual and automatic shots behave the same.
   */
  private static final double SHOOTER_SPINUP_RPM = ShooterConstants.REFERENCE_RPM;
  private static final double INVALID_TARGET_WARNING_PERIOD_SECONDS = 5.0;
  private static final double LIMELIGHT_FEEDBACK_STEP_RPM = 75.0;
  private static final double FEEDBACK_WINDOW_SECONDS = 10.0;
  private static final double DISTANCE_BUCKET_METERS = 0.5;
  private static final int DISTANCE_BUCKET_COUNT = 20;
  private static final double DEFAULT_RANGE_HALF_WIDTH_RPM = 175.0;
  private static final int MIN_BUCKETS_FOR_MARKDOWN_TABLE = 3;

  private final SparkMax leaderSpark = new SparkMax(ShooterConstants.kLeaderMotorId,
      MotorType.kBrushless);

  private final SparkMax followerSpark = new SparkMax(ShooterConstants.kFollowerMotorId,
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
      .withDiameter(Inches.of(ShooterConstants.WHEEL_DIAMETER_INCHES))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(ShooterConstants.MAX_MECHANICAL_RPM))
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
  private double shooterAttemptLastBearingDegrees = 0.0;
  private double shooterAttemptLastTurretAngleDegrees = 0.0;
  private double pendingFeedbackDistanceMeters = 0.0;
  private double pendingFeedbackCommandedRpm = 0.0;

  /** Setpoint the vision shot model last asked for, so readiness checks have something to compare. */
  private double commandedShotRpm = 0.0;

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
        () -> RPM.of(percentSupplier.get() * ShooterConstants.MAX_MECHANICAL_RPM))
        .finallyDo(() -> leaderSpark.set(0));
  }

  public Command spinUp() {
    return setSpeed(RPM.of(SHOOTER_SPINUP_RPM));
  }

  /**
   * Stops the flywheel by cutting output, not by asking the velocity loop for 0 RPM.
   *
   * <p>This used to be {@code setSpeed(RPM.of(0))}. That leaves the closed-loop controller running
   * with a 0 setpoint, and with kS = 0.191 V in the feedforward the loop can keep putting voltage
   * into a stationary wheel — which is what made the shooter creep and buzz while "stopped".
   * Commanding zero duty cycle is unambiguous.
   */
  public Command stop() {
    // Momentary, not a hold. It cuts output and finishes immediately, handing the subsystem back
    // to idleCommand() which keeps it at zero. A hold here would never finish, so any
    // Commands.parallel(...stop()...) at the end of a sequence — like the one closing out
    // ShootingCommand — would hang forever waiting on it.
    return runOnce(() -> leaderSpark.set(0.0)).withName("Shooter.stop");
  }

  /**
   * Default command: holds the flywheel at zero output whenever nothing else is using it.
   *
   * <p>Without this the shooter had no default command at all, so after any command ended the
   * SparkMax simply kept applying whatever it was last told. That is why it sat there spinning on
   * its own — nothing was telling it to stop, so it never did.
   */
  public Command idleCommand() {
    return run(() -> leaderSpark.set(0.0)).withName("Shooter.IDLE");
  }

  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
  }

  public Command sysId() {
    return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
  }

  /** RPM the vision shot model is currently asking for. Zero when it is not running. */
  public double getCommandedShotRpm() {
    return commandedShotRpm;
  }

  /** True when the flywheel has reached whatever the vision shot model last asked for. */
  public boolean isAtShotSpeed() {
    return commandedShotRpm > 0.0
        && Math.abs(getSpeed().in(RPM) - commandedShotRpm) <= ShooterConstants.READY_TOLERANCE_RPM;
  }

  /**
   * Spins the flywheel to whatever speed the supplied range calls for, updated every loop.
   *
   * <p>Takes a distance rather than a camera on purpose. The shot model cares about one number —
   * metres from the ball exit to the goal — and nothing about how that number was measured. Keeping
   * vision out of this class means the shot table can be exercised from a tape measure, a
   * simulation, or a dashboard slider, and a broken camera can never masquerade as a shooter bug.
   *
   * @param shooterDistanceMetersSupplier range from the shooter exit to the goal, or a
   *                                      non-positive value when it is unknown
   */
  public Command setSpeedForDistance(DoubleSupplier shooterDistanceMetersSupplier) {
    return shooter.setSpeed(() -> {
      double nowSeconds = Timer.getFPGATimestamp();
      double measuredDistanceMeters = shooterDistanceMetersSupplier.getAsDouble();

      if (!(measuredDistanceMeters > 0.0) || !Double.isFinite(measuredDistanceMeters)) {
        frc.robot.util.RobotLog.put("Shooter/ShotValid", false);
        frc.robot.util.RobotLog.put("Shooter/ShotMessage", "no range available");
        frc.robot.util.RobotLog.put("Shooter/CommandedRPM", 0.0);
        maybeReportInvalidTarget(nowSeconds, "no range available");
        wasLimelightShotValidLastCycle = false;
        commandedShotRpm = 0.0;
        return RPM.of(0);
      }

      double distanceMeters = measuredDistanceMeters;
      boolean distanceOutOfRange =
          distanceMeters < ShooterConstants.MIN_VALID_DISTANCE_METERS
              || distanceMeters > ShooterConstants.MAX_VALID_DISTANCE_METERS;
      if (distanceOutOfRange) {
        // Rather than firing on a nonsense range, fall back to the one distance known calibrated.
        distanceMeters = ShooterConstants.REFERENCE_DISTANCE_METERS;
      }

      int distanceBucket = bucketForDistance(distanceMeters);
      double distanceTrimRpm = limelightDistanceTrimRpm[distanceBucket];
      double modelRpm = modelRpmForDistance(distanceMeters) + distanceTrimRpm;

      TableEstimate tableEstimate = estimateFromShotTable(distanceMeters, modelRpm);
      double rangedRecommended = MathUtil.clamp(
          tableEstimate.recommendedRpm, tableEstimate.rangeMinRpm, tableEstimate.rangeMaxRpm);
      double commandedRPM = MathUtil.clamp(
          rangedRecommended, ShooterConstants.MIN_SHOT_RPM, ShooterConstants.MAX_SHOT_RPM);

      // Flag a shot pinned at the ceiling. Past this range the flywheel cannot go any faster, so
      // the distance/RPM relationship flattens and shots start falling short with no other
      // symptom. Better to say so than to let it look like a tuning problem.
      boolean saturated = commandedRPM >= ShooterConstants.MAX_SHOT_RPM - 1.0
          && modelRpm > ShooterConstants.MAX_SHOT_RPM;
      frc.robot.util.RobotLog.put("Shooter/Saturated", saturated);
      SmartDashboard.putBoolean("Shooter/Saturated", saturated);
      if (saturated) {
        maybeWarnSaturated(nowSeconds, distanceMeters, modelRpm);
      }

      // The headline pair: what range was measured, and what power it produced.
      SmartDashboard.putNumber("Shooter/DistanceMeters", distanceMeters);
      SmartDashboard.putNumber("Shooter/DistanceFeet", distanceMeters * 3.28084);
      SmartDashboard.putNumber("Shooter/CommandedRPM", commandedRPM);

      frc.robot.util.RobotLog.put("Shooter/ShotValid", true);
      frc.robot.util.RobotLog.put("Shooter/ShotMessage",
          tableEstimate.fromTable ? "Learned shot table" : "Physics model anchored on reference shot");
      frc.robot.util.RobotLog.put("Shooter/ModelRPM", modelRpm);
      frc.robot.util.RobotLog.put("Shooter/CommandedRPM", commandedRPM);
      frc.robot.util.RobotLog.put("Shooter/DistanceMeters", distanceMeters);
      frc.robot.util.RobotLog.put("Shooter/MeasuredDistanceMeters", measuredDistanceMeters);
      frc.robot.util.RobotLog.put("Shooter/DistanceOutOfRange", distanceOutOfRange);
      frc.robot.util.RobotLog.put("Shooter/DistanceBucket", distanceBucket);
      frc.robot.util.RobotLog.put("Shooter/DistanceTrimRPM", distanceTrimRpm);
      frc.robot.util.RobotLog.put("Shooter/TableRecommendedRPM", tableEstimate.recommendedRpm);
      frc.robot.util.RobotLog.put("Shooter/TableUsed", tableEstimate.fromTable);
      frc.robot.util.RobotLog.put("Shooter/TransferEfficiency", ProjectileMotion.FLYWHEEL_TRANSFER_EFFICIENCY);

      cacheShooterAttemptSample(nowSeconds, distanceMeters, modelRpm, commandedRPM, distanceBucket);
      wasLimelightShotValidLastCycle = true;
      commandedShotRpm = commandedRPM;

      return RPM.of(commandedRPM);
    }).beforeStarting(() -> {
      activeShooterAttemptId = nextAttemptId();
      lastInvalidTargetWarningTimestamp = -1.0;
      wasLimelightShotValidLastCycle = true;
      shooterAttemptHasValidSample = false;
      warnedInvalidTargetThisAttempt = false;
      commandedShotRpm = 0.0;
      System.out.printf("SHOT_ATTEMPT_START,attempt=%d%n", activeShooterAttemptId);
    }).finallyDo(() -> {
      wasLimelightShotValidLastCycle = true;
      lastInvalidTargetWarningTimestamp = -1.0;
      commandedShotRpm = 0.0;
      if (shooterAttemptHasValidSample) {
        printShotTelemetryRow();
        pendingFeedbackBucket = shooterAttemptLastDistanceBucket;
        pendingFeedbackExpirySeconds = Timer.getFPGATimestamp() + FEEDBACK_WINDOW_SECONDS;
        pendingFeedbackAdjusted = false;
        pendingFeedbackDistanceMeters = shooterAttemptLastDistanceMeters;
        pendingFeedbackCommandedRpm = shooterAttemptLastCommandedRpm;
        System.out.printf(
            "SHOT_FEEDBACK_WINDOW_OPEN,attempt=%d,bucket=%d,expires_s=%.3f,current_trim_rpm=%.1f%n",
            activeShooterAttemptId,
            pendingFeedbackBucket,
            pendingFeedbackExpirySeconds,
            limelightDistanceTrimRpm[pendingFeedbackBucket]);
      }
      System.out.printf("SHOT_ATTEMPT_END,attempt=%d%n", activeShooterAttemptId);
      activeShooterAttemptId = -1;
      shooterAttemptHasValidSample = false;
      warnedInvalidTargetThisAttempt = false;
      pendingFeedbackDistanceMeters = 0.0;
      pendingFeedbackCommandedRpm = 0.0;
    }).withName("Shooter.setSpeedForDistance");
  }

  private double lastSaturationWarningSeconds = -1.0;

  private void maybeWarnSaturated(double nowSeconds, double distanceMeters, double wantedRpm) {
    if (lastSaturationWarningSeconds >= 0.0
        && nowSeconds - lastSaturationWarningSeconds < 5.0) {
      return;
    }
    lastSaturationWarningSeconds = nowSeconds;
    DriverStation.reportWarning(
        String.format(
            "[SHOOTER] Out of range: %.2f m needs %.0f RPM but the flywheel ceiling is %.0f. "
                + "This shot will fall short. Drive closer, or raise "
                + "ShooterConstants.MAX_MECHANICAL_RPM if the hardware allows it.",
            distanceMeters, wantedRpm, ShooterConstants.MAX_SHOT_RPM),
        false);
  }

  /** Monotonic counter so each spin-up attempt can be correlated across log lines. */
  private static int attemptCounter = 0;

  private static synchronized int nextAttemptId() {
    return ++attemptCounter;
  }

  /**
   * Baseline RPM for a range, before any learned-table override.
   *
   * <p>Falls back to the reference shot if the geometry is unsolvable — which happens when the
   * target is so close that a 45 degree launch cannot come back down in time.
   */
  /**
   * RPM for a shot distance, straight from the measured table.
   *
   * <p>This used to call projectile physics scaled by one back-solved efficiency factor. The shape
   * was right but the detail was not — a single global factor cannot absorb ball compression,
   * slip, wheel wear and an approximate hood angle all at once, so it was accurate near the one
   * reference shot and long at the ends. {@link ShotTable} holds measurements instead, where a
   * wrong row is fixed by editing that row and nothing else moves.
   */
  private double modelRpmForDistance(double distanceMeters) {
    double rpm = ShotTable.rawRpmForDistance(distanceMeters);
    if (!Double.isFinite(rpm) || rpm <= 0.0) {
      return ShooterConstants.REFERENCE_RPM;
    }
    return rpm;
  }

  private double rpmForDistanceMeters(double distanceMeters) {
    if (distanceMeters <= 0.0) {
      return 0.0;
    }
    return modelRpmForDistance(distanceMeters);
  }

  @Override
  public void periodic() {
    frc.robot.util.RobotLog.put("Shooter/LeaderVelocity", leaderSpark.getEncoder().getVelocity());
    frc.robot.util.RobotLog.put("Shooter/FollowerVelocity", followerSpark.getEncoder().getVelocity());
    frc.robot.util.RobotLog.put("Shooter/AtShotSpeed", isAtShotSpeed());
    maybeFinalizeFeedbackWindow();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  private void maybeReportInvalidTarget(double nowSeconds, String reason) {
    int attemptId = activeShooterAttemptId;
    if (attemptId < 0) {
      return;
    }

    if (wasLimelightShotValidLastCycle) {
      DriverStation.reportWarning(
          String.format(
              "[SHOT][WARN][SHOOTER][ATTEMPT %d] Limelight target invalid during shoot command (%s).",
              attemptId,
              reason),
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
      double distanceMeters,
      double modelRpm,
      double commandedRPM,
      int distanceBucket) {
    shooterAttemptHasValidSample = true;
    shooterAttemptLastTimestampSeconds = nowSeconds;
    shooterAttemptLastDistanceMeters = distanceMeters;
    shooterAttemptLastEquationRpm = modelRpm;
    shooterAttemptLastCommandedRpm = commandedRPM;
    shooterAttemptLastActualRpm = getSpeed().in(RPM);
    shooterAttemptLastDistanceBucket = distanceBucket;
  }

  private void printShotTelemetryRow() {
    if (!printedShotHeader) {
      System.out.println(
          "SHOT_TABLE_HEADER,source,attempt,time_s,tag_id,distance_m,model_rpm,commanded_rpm,actual_rpm,robot_bearing_deg,turret_angle_deg,launch_angle_deg");
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
        shooterAttemptLastBearingDegrees,
        shooterAttemptLastTurretAngleDegrees,
        ShooterConstants.LAUNCH_ANGLE_DEGREES);
  }

  public Command increaseLimelightPowerForLastShotCommand() {
    return runOnce(() -> applyFeedbackTrim(LIMELIGHT_FEEDBACK_STEP_RPM)).ignoringDisable(true);
  }

  public Command decreaseLimelightPowerForLastShotCommand() {
    return runOnce(() -> applyFeedbackTrim(-LIMELIGHT_FEEDBACK_STEP_RPM)).ignoringDisable(true);
  }

  public Command confirmMadeShotCommand() {
    return runOnce(this::confirmMadeShotNow).ignoringDisable(true);
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
      return new TableEstimate(avg, tableRpmMin[bucket], tableRpmMax[bucket], false, true);
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
      return new TableEstimate(recommended, rangeMin, rangeMax, true, true);
    }

    // Only one side of the table has data. Extrapolating a flat value out to an untested range is
    // worse than trusting the physics curve, so blend toward the model by how far outside the
    // tested range we are.
    if (lower >= 0 || upper >= 0) {
      int nearest = lower >= 0 ? lower : upper;
      double nearestAvg = tableRpmSum[nearest] / tableSampleCount[nearest];
      double bucketsAway = Math.abs(bucket - nearest);
      double modelWeight = MathUtil.clamp(bucketsAway / 3.0, 0.0, 1.0);
      double recommended = lerp(nearestAvg, fallbackRpm, modelWeight);
      double halfWidth = lerp(
          Math.max(tableRpmMax[nearest] - tableRpmMin[nearest], 1.0) / 2.0,
          DEFAULT_RANGE_HALF_WIDTH_RPM,
          modelWeight);
      return new TableEstimate(
          recommended, recommended - halfWidth, recommended + halfWidth, true, modelWeight < 1.0);
    }

    return new TableEstimate(
        fallbackRpm,
        fallbackRpm - DEFAULT_RANGE_HALF_WIDTH_RPM,
        fallbackRpm + DEFAULT_RANGE_HALF_WIDTH_RPM,
        false,
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
    double rpm = MathUtil.clamp(
        commandedRpm, ShooterConstants.MIN_SHOT_RPM, ShooterConstants.MAX_SHOT_RPM);
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

  /** Wipes every learned shot and per-distance trim. Use after changing the shooter mechanically. */
  public Command clearLearnedShotDataCommand() {
    return runOnce(() -> {
      for (int i = 0; i < DISTANCE_BUCKET_COUNT; i++) {
        limelightDistanceTrimRpm[i] = 0.0;
        tableRpmSum[i] = 0.0;
        tableRpmMin[i] = 0.0;
        tableRpmMax[i] = 0.0;
        tableSampleCount[i] = 0;
        savePersistentDistanceTrim(i);
        savePersistentShotTableBucket(i);
      }
      System.out.println("SHOT_TABLE_CLEARED,source=SHOOTER");
    }).ignoringDisable(true).withName("Shooter.clearLearnedShotData");
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
    System.out.println("| Distance (m) | RPM Min | RPM Avg | RPM Max | Model RPM | Samples |");
    System.out.println("| --- | --- | --- | --- | --- | --- |");
    for (int i = 0; i < DISTANCE_BUCKET_COUNT; i++) {
      if (tableSampleCount[i] <= 0) {
        continue;
      }
      double avg = tableRpmSum[i] / tableSampleCount[i];
      double centerDistance = bucketCenterDistanceMeters(i);
      System.out.printf(
          "| %.2f | %.1f | %.1f | %.1f | %.1f | %d |%n",
          centerDistance,
          tableRpmMin[i],
          avg,
          tableRpmMax[i],
          rpmForDistanceMeters(centerDistance),
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
    /** True when learned data contributed, false when this is purely the physics model. */
    final boolean fromTable;

    TableEstimate(
        double recommendedRpm,
        double rangeMinRpm,
        double rangeMaxRpm,
        boolean interpolated,
        boolean fromTable) {
      this.recommendedRpm = recommendedRpm;
      this.rangeMinRpm = rangeMinRpm;
      this.rangeMaxRpm = rangeMaxRpm;
      this.interpolated = interpolated;
      this.fromTable = fromTable;
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
    return Inches.of(ShooterConstants.WHEEL_DIAMETER_INCHES).div(2);
  }

  public LinearVelocity getTangentialVelocity() {
    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
  }
}
