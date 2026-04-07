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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
      Set.of(9, 10, 11, 18, 19, 20, 2, 3, 4, 5, 8, 21, 24, 25, 26, 27);

  private static final double MAX_ONE_DIR_FOV = 90.0;
  private static final double SLOW_ZONE_DEGREES = 20.0;
  private static final double TRACK_TX_FILTER_ALPHA = 0.35;
  private static final double TRACK_DEADBAND_DEGREES = 0.75;
  private static final double TRACK_KP = 0.012;
  private static final double TRACK_MIN_OUTPUT = 0.045;
  private static final double TRACK_MAX_OUTPUT = 0.18;
  private static final double TRACK_OUTPUT_SIGN = 1.0;
  private static final double TRACK_HEALTH_MAX_TX_DEGREES = 3.0;
  private static final double TRACK_HEALTH_MAX_AGE_SECONDS = 0.20;

  public final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375);

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

  private double filteredTrackingTxDegrees = 0.0;
  private double lastTrackingOutput = 0.0;
  private double lastTrackingTargetAngleDegrees = 0.0;
  private int lastTrackingTagId = -1;
  private double lastTrackingTimestampSeconds = -1.0;
  private boolean hasTrackingSample = false;

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
    if (lastTrackingTimestampSeconds < 0.0) {
      return -1.0;
    }
    return Timer.getFPGATimestamp() - lastTrackingTimestampSeconds;
  }

  public boolean isTrackingHealthy() {
    double ageSeconds = getLastValidTrackingAgeSeconds();
    return hasTrackingSample
        && lastTrackingTagId != -1
        && ageSeconds >= 0.0
        && ageSeconds <= TRACK_HEALTH_MAX_AGE_SECONDS
        && Math.abs(filteredTrackingTxDegrees) <= TRACK_HEALTH_MAX_TX_DEGREES;
  }

  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command manualSet(Supplier<Double> dutyCycleSupplier) {
    return run(() -> {
      double rawInput = dutyCycleSupplier.get();
      double currentAngleDegrees = turret.getAngle().in(Degrees);
      double output = applySoftLimitScaling(rawInput, currentAngleDegrees);
      smc.setDutyCycle(output);
      lastTrackingOutput = output;
    }).finallyDo(() -> smc.setDutyCycle(0.0)).withName("Turret.manualSet");
  }

  public Command rezero() {
    return Commands.runOnce(() -> spark.getEncoder().setPosition(0), this)
        .withName("Turret.rezero");
  }

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  public Command trackTarget(LimeLight limelight) {
    return trackTarget(limelight, null, TRACKING_TAG_IDS);
  }

  public Command trackTarget(LimeLight limelight, SwerveSubsystem drivebase) {
    return trackTarget(limelight, drivebase, TRACKING_TAG_IDS);
  }

  public Command trackTarget(LimeLight limelight, SwerveSubsystem drivebase, Set<Integer> allowedTagIds) {
    return run(() -> {
      LimeLight.AprilTagScan scan = limelight.scanDirect(allowedTagIds);

      if (!scan.isValid() || !Double.isFinite(scan.tx)) {
        clearTrackingState();
        smc.setDutyCycle(0.0);
        return;
      }

      if (!hasTrackingSample || scan.tagID != lastTrackingTagId) {
        filteredTrackingTxDegrees = scan.tx;
      } else {
        filteredTrackingTxDegrees +=
            (scan.tx - filteredTrackingTxDegrees) * TRACK_TX_FILTER_ALPHA;
      }

      if (Math.abs(filteredTrackingTxDegrees) < TRACK_DEADBAND_DEGREES) {
        filteredTrackingTxDegrees = 0.0;
      }

      double currentAngleDegrees = turret.getAngle().in(Degrees);
      double output = TRACK_OUTPUT_SIGN * filteredTrackingTxDegrees * TRACK_KP;
      if (Math.abs(filteredTrackingTxDegrees) > TRACK_DEADBAND_DEGREES
          && Math.abs(output) < TRACK_MIN_OUTPUT) {
        output = Math.copySign(TRACK_MIN_OUTPUT, output);
      }
      output = MathUtil.clamp(output, -TRACK_MAX_OUTPUT, TRACK_MAX_OUTPUT);
      output = applySoftLimitScaling(output, currentAngleDegrees);

      smc.setDutyCycle(output);
      lastTrackingOutput = output;
      lastTrackingTargetAngleDegrees = currentAngleDegrees + (filteredTrackingTxDegrees * TRACK_OUTPUT_SIGN);
      lastTrackingTagId = scan.tagID;
      lastTrackingTimestampSeconds = Timer.getFPGATimestamp();
      hasTrackingSample = true;

      Logger.recordOutput("Turret/TrackingState", "TRACKING");
      Logger.recordOutput("Turret/TrackingTagID", scan.tagID);
      Logger.recordOutput("Turret/RawTX", scan.tx);
      Logger.recordOutput("Turret/FilteredTX", filteredTrackingTxDegrees);
      Logger.recordOutput("Turret/TrackingDutyCycle", output);
      Logger.recordOutput("Turret/TrackingTargetAngleDegrees", lastTrackingTargetAngleDegrees);
      Logger.recordOutput("Turret/TrackingAngleErrorDegrees", filteredTrackingTxDegrees);
      Logger.recordOutput("Turret/TargetDistanceM", scan.distance);
      Logger.recordOutput("Turret/TrackingLastValidAgeSec", 0.0);
    }).beforeStarting(this::clearTrackingState)
        .finallyDo(interrupted -> {
          smc.setDutyCycle(0.0);
          lastTrackingOutput = 0.0;
        })
        .withName("Turret.trackTarget");
  }

  private void clearTrackingState() {
    filteredTrackingTxDegrees = 0.0;
    lastTrackingOutput = 0.0;
    lastTrackingTargetAngleDegrees = turret.getAngle().in(Degrees);
    lastTrackingTagId = -1;
    lastTrackingTimestampSeconds = -1.0;
    hasTrackingSample = false;
    Logger.recordOutput("Turret/TrackingState", "NO_TARGET");
    Logger.recordOutput("Turret/TrackingTagID", -1);
    Logger.recordOutput("Turret/FilteredTX", 0.0);
    Logger.recordOutput("Turret/TrackingTargetAngleDegrees", lastTrackingTargetAngleDegrees);
    Logger.recordOutput("Turret/TrackingAngleErrorDegrees", 0.0);
  }

  private double applySoftLimitScaling(double input, double currentAngleDegrees) {
    double distanceToPositiveLimit = MAX_ONE_DIR_FOV - currentAngleDegrees;
    double distanceToNegativeLimit = currentAngleDegrees - (-MAX_ONE_DIR_FOV);
    double scale = 1.0;

    if (input > 0.0 && distanceToPositiveLimit < SLOW_ZONE_DEGREES) {
      scale = Math.max(0.0, distanceToPositiveLimit / SLOW_ZONE_DEGREES);
    } else if (input < 0.0 && distanceToNegativeLimit < SLOW_ZONE_DEGREES) {
      scale = Math.max(0.0, distanceToNegativeLimit / SLOW_ZONE_DEGREES);
    }

    return input * scale;
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();
    Logger.recordOutput("Turret/AngleDegrees", turret.getAngle().in(Degrees));
    Logger.recordOutput("Turret/RawEncoderRotations", spark.getEncoder().getPosition());
    Logger.recordOutput("Turret/LastTrackingOutput", lastTrackingOutput);
    Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
        new Pose3d(
            turretTranslation,
            new Rotation3d(0, 0, turret.getAngle().in(Radians)))
    });
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
