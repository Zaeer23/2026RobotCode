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
  private static final double LIMELIGHT_REFERENCE_DISTANCE_METERS = 3.5052; // 11 ft 6 in
  private static final double LIMELIGHT_REFERENCE_RPM = SHOOTER_SPINUP_RPM;
  private static final double LIMELIGHT_RPM_PER_METER = 650.0;
  private static final double LIMELIGHT_RPM_TRIM = 50.0;
  private static final double SHOT_CONSOLE_LOG_PERIOD_SECONDS = 0.25;

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
  private double lastShotConsoleLogTimestamp = -1.0;
  private double lastInvalidTargetWarningTimestamp = -1.0;
  private int activeShooterAttemptId = -1;

  public ShooterSubsystem() {
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
    return setSpeed(RPM.of(SHOOTER_SPINUP_RPM));
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

        double distanceMeters = estimateTargetDistanceMeters(scan);
        double equationRPM = rpmForDistanceMeters(distanceMeters);
        double commandedRPM = MathUtil.clamp(equationRPM, LIMELIGHT_SHOT_MIN_RPM, SHOOTER_MAX_RPM);

        Logger.recordOutput("Shooter/LimelightShotValid", true);
        Logger.recordOutput("Shooter/LimelightShotMessage", "Scaled from 11ft 6in reference shot");
        Logger.recordOutput("Shooter/LimelightEstimatedRPM", equationRPM);
        Logger.recordOutput("Shooter/LimelightEquationRPM", equationRPM);
        Logger.recordOutput("Shooter/LimelightCommandedRPM", commandedRPM);
        Logger.recordOutput("Shooter/LimelightDistanceMeters", distanceMeters);
        Logger.recordOutput("Shooter/LimelightReferenceDistanceMeters", LIMELIGHT_REFERENCE_DISTANCE_METERS);
        Logger.recordOutput("Shooter/LimelightReferenceRPM", LIMELIGHT_REFERENCE_RPM);
        Logger.recordOutput("Shooter/LimelightRpmPerMeter", LIMELIGHT_RPM_PER_METER);
        Logger.recordOutput("Shooter/LimelightRpmTrim", LIMELIGHT_RPM_TRIM);
        Logger.recordOutput("Shooter/LimelightTargetTagID", scan.tagID);
        maybePrintShotTelemetry(
            nowSeconds,
            scan,
            distanceMeters,
            equationRPM,
            commandedRPM,
            fixedLaunchAngleDegrees);
        wasLimelightShotValidLastCycle = true;

        return RPM.of(commandedRPM);
    }).beforeStarting(() -> {
      activeShooterAttemptId = LimelightAttemptTracker.nextAttemptId();
      lastShotConsoleLogTimestamp = -1.0;
      lastInvalidTargetWarningTimestamp = -1.0;
      wasLimelightShotValidLastCycle = true;
      System.out.printf(
          "LIMELIGHT_ATTEMPT_START,source=SHOOTER,attempt=%d%n",
          activeShooterAttemptId);
    }).finallyDo(() -> {
      leaderSpark.set(0);
      wasLimelightShotValidLastCycle = true;
      lastShotConsoleLogTimestamp = -1.0;
      lastInvalidTargetWarningTimestamp = -1.0;
      System.out.printf(
          "LIMELIGHT_ATTEMPT_END,source=SHOOTER,attempt=%d%n",
          activeShooterAttemptId);
      activeShooterAttemptId = -1;
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
    double totalAngleRad = Math.toRadians(ProjectileMotion.LIMELIGHT_MOUNT_ANGLE_DEGREES + ty);
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
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  private void maybeReportInvalidTarget(double nowSeconds) {
    int attemptId = activeShooterAttemptId;
    if (wasLimelightShotValidLastCycle) {
      DriverStation.reportWarning(
          String.format(
              "[SHOT][WARN][SHOOTER][ATTEMPT %d] Limelight target invalid during shoot command (no approved tag / no target).",
              attemptId),
          false);
      lastInvalidTargetWarningTimestamp = nowSeconds;
      return;
    }

    if (lastInvalidTargetWarningTimestamp < 0
        || nowSeconds - lastInvalidTargetWarningTimestamp >= 1.5) {
      DriverStation.reportWarning(
          String.format(
              "[SHOT][WARN][SHOOTER][ATTEMPT %d] Still no valid Limelight target while shooter command is active.",
              attemptId),
          false);
      lastInvalidTargetWarningTimestamp = nowSeconds;
    }
  }

  private void maybePrintShotTelemetry(
      double nowSeconds,
      LimeLight.AprilTagScan scan,
      double distanceMeters,
      double equationRPM,
      double commandedRPM,
      double fixedLaunchAngleDegrees) {
    if (!printedShotHeader) {
      System.out.println(
          "SHOT_TABLE_HEADER,source,attempt,time_s,tag_id,distance_m,equation_rpm,commanded_rpm,actual_rpm,tx_deg,ty_deg,launch_angle_deg");
      printedShotHeader = true;
    }

    if (lastShotConsoleLogTimestamp >= 0
        && nowSeconds - lastShotConsoleLogTimestamp < SHOT_CONSOLE_LOG_PERIOD_SECONDS) {
      return;
    }

    lastShotConsoleLogTimestamp = nowSeconds;
    double actualRPM = getSpeed().in(RPM);
    System.out.printf(
        "SHOT_TABLE_ROW,SHOOTER,%d,%.3f,%d,%.3f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f%n",
        activeShooterAttemptId,
        nowSeconds,
        scan.tagID,
        distanceMeters,
        equationRPM,
        commandedRPM,
        actualRPM,
        scan.tx,
        scan.ty,
        fixedLaunchAngleDegrees);
  }

  private Distance wheelRadius() {
    return Inches.of(4).div(2);
  }

  public LinearVelocity getTangentialVelocity() {
    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
  }
}
