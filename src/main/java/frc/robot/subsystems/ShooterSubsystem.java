package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import static edu.wpi.first.units.Units.RPM;

public class ShooterSubsystem extends SubsystemBase {
  private static final double SHOOTER_MAX_RPM = 5600.0;
  private static final double LIMELIGHT_SHOT_MIN_RPM = 3200.0;
  private static final double[] LIMELIGHT_DISTANCE_METERS = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
  private static final double[] LIMELIGHT_DISTANCE_RPM = {3200.0, 3500.0, 3900.0, 4400.0, 5000.0, 5600.0};

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
      .withStatorCurrentLimit(Amps.of(40));

  private final SmartMotorController smc = new SparkWrapper(leaderSpark, DCMotor.getNEO(2), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(5400))
      .withLowerSoftLimit(RPM.of(0))
      .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  private final FlyWheel shooter = new FlyWheel(shooterConfig);

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

public Command setPercentAsRPM(Supplier<Double> percentSupplier) {
    return shooter.setSpeed(
        () -> RPM.of(percentSupplier.get() * SHOOTER_MAX_RPM)
    ).finallyDo(() -> leaderSpark.set(0));
}
  public Command spinUp() {
    return setSpeed(RPM.of(SHOOTER_MAX_RPM));
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
    return shooter.setSpeed(() -> {
        if (!limelight.hasTarget()) {
            Logger.recordOutput("Shooter/LimelightShotValid", false);
            Logger.recordOutput("Shooter/LimelightShotMessage", "No Limelight target");
            Logger.recordOutput("Shooter/LimelightDistanceMeters", 0.0);
            Logger.recordOutput("Shooter/LimelightCommandedRPM", 0.0);
            return RPM.of(0);
        }

        double distanceMeters = estimateTargetDistanceMeters(limelight);
        double commandedRPM = MathUtil.clamp(
            rpmForDistanceMeters(distanceMeters), LIMELIGHT_SHOT_MIN_RPM, SHOOTER_MAX_RPM);

        Logger.recordOutput("Shooter/LimelightShotValid", true);
        Logger.recordOutput("Shooter/LimelightShotMessage", "Distance curve");
        Logger.recordOutput("Shooter/LimelightEstimatedRPM", commandedRPM);
        Logger.recordOutput("Shooter/LimelightCommandedRPM", commandedRPM);
        Logger.recordOutput("Shooter/LimelightDistanceMeters", distanceMeters);

        return RPM.of(commandedRPM);
    }).finallyDo(() -> leaderSpark.set(0));
}

  private double estimateTargetDistanceMeters(LimeLight limelight) {
    double ty = limelight.getTY();
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
    if (distanceMeters <= LIMELIGHT_DISTANCE_METERS[0]) {
      return LIMELIGHT_DISTANCE_RPM[0];
    }

    int lastIndex = LIMELIGHT_DISTANCE_METERS.length - 1;
    if (distanceMeters >= LIMELIGHT_DISTANCE_METERS[lastIndex]) {
      return LIMELIGHT_DISTANCE_RPM[lastIndex];
    }

    for (int i = 1; i < LIMELIGHT_DISTANCE_METERS.length; i++) {
      double upperDistance = LIMELIGHT_DISTANCE_METERS[i];
      if (distanceMeters <= upperDistance) {
        double lowerDistance = LIMELIGHT_DISTANCE_METERS[i - 1];
        double lowerRpm = LIMELIGHT_DISTANCE_RPM[i - 1];
        double upperRpm = LIMELIGHT_DISTANCE_RPM[i];
        double t = (distanceMeters - lowerDistance) / (upperDistance - lowerDistance);
        return lowerRpm + t * (upperRpm - lowerRpm);
      }
    }

    return LIMELIGHT_DISTANCE_RPM[lastIndex];
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

  private Distance wheelRadius() {
    return Inches.of(4).div(2);
  }

  public LinearVelocity getTangentialVelocity() {
    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
  }
}
