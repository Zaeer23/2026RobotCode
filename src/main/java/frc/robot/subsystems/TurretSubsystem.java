package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Command;
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
  private static final double LIMELIGHT_FORWARD_OFFSET_INCHES = 2.5;     // fairly close to flush

  // Slow zone: ramp down output near soft limits
  private static final double SLOW_ZONE_DEGREES = 20.0;

  // Tracking constants — tune kP if still oscillating/sluggish
  private static final double TRACK_DEADBAND_DEGREES = 2.5;
  private static final double TRACK_KP = 0.008;
  private static final double TRACK_MAX_OUTPUT = 0.3;

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
    return run(() -> {
      LimeLight.AprilTagScan scan = limelight.scan();
      double currentAngle = turret.getAngle().in(Degrees);

      if (!scan.isValid()) {
        spark.set(0);
        Logger.recordOutput("Turret/TrackingState", "NO_TARGET");
        return;
      }

      // Parallax correction based on distance: closer targets need more correction
      double distanceInches = scan.distance * 39.37;
      double parallax = Math.toDegrees(
          Math.atan2(LIMELIGHT_HORIZONTAL_OFFSET_INCHES, distanceInches));

      // Always subtract — limelight is always to the right of turret center
      double targetAngle = currentAngle + (scan.tx - parallax);
      double correctedTX = scan.tx - parallax;

      Logger.recordOutput("Turret/RawTX", scan.tx);
      Logger.recordOutput("Turret/CorrectedTX", correctedTX);
      Logger.recordOutput("Turret/ParallaxDeg", parallax);
      Logger.recordOutput("Turret/TargetDistanceM", scan.distance);

      if (Math.abs(correctedTX) < TRACK_DEADBAND_DEGREES) {
        spark.set(0);
        Logger.recordOutput("Turret/TrackingState", "ON_TARGET");
        return;
      }

      // Negative: positive TX = target is right = rotate right = positive spark output
      // but check your motor inversion — flip sign here if turret goes wrong way
      double output = -correctedTX * TRACK_KP;
      output = MathUtil.clamp(output, -TRACK_MAX_OUTPUT, TRACK_MAX_OUTPUT);
      output = applySoftLimitScaling(output, currentAngle);

      Logger.recordOutput("Turret/TrackingState", "TRACKING");
      Logger.recordOutput("Turret/TrackingOutput", output);

      spark.set(output);
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
        new Pose3d(turretTranslation, new Rotation3d(0, 0, turret.getAngle().in(Radians)))
    });
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}