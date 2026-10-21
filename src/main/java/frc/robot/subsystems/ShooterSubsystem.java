package frc.robot.subsystems;

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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.RobotLog;
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

/** Fixed-speed flywheel control. */
public class ShooterSubsystem extends SubsystemBase {
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

  private double commandedShotRpm;
  private boolean requestedSpeedReachable;

  public Command setSpeed(AngularVelocity speed) {
    return setSpeedDynamic(() -> speed);
  }

  public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
    // YAMS evaluates suppliers while constructing commands. Sample only during execute so
    // binding a command cannot create a stale target or accidentally claim readiness.
    return Commands.startRun(() -> {
      commandedShotRpm = 0.0;
      requestedSpeedReachable = false;
      smc.startClosedLoopController();
    }, () -> {
      double requestedRpm = speedSupplier.get().in(RPM);
      requestedSpeedReachable = Double.isFinite(requestedRpm) && requestedRpm > 0.0
          && requestedRpm <= ShooterConstants.MAX_MECHANICAL_RPM;
      commandedShotRpm = Double.isFinite(requestedRpm)
          ? MathUtil.clamp(requestedRpm, 0.0, ShooterConstants.MAX_MECHANICAL_RPM) : 0.0;
      if (commandedShotRpm > 0.0) {
        smc.setVelocity(RPM.of(commandedShotRpm));
      } else {
        smc.setDutyCycle(0.0);
      }
    }, this).finallyDo(this::stopNow);
  }

  public Command setPercent(Supplier<Double> percentSupplier) {
    return run(() -> {
      commandedShotRpm = 0.0;
      requestedSpeedReachable = false;
      smc.setDutyCycle(percentSupplier.get());
    }).finallyDo(this::stopNow);
  }

  public Command waitCommand() {
    return new WaitCommand(2.0);
  }

  public Command setPercentAsRPM(Supplier<Double> percentSupplier) {
    return setSpeedDynamic(() -> RPM.of(percentSupplier.get() * ShooterConstants.MAX_MECHANICAL_RPM));
  }

  /** Uses the existing fixed shooter speed. */
  public Command spinUp() {
    return setSpeed(RPM.of(ShooterConstants.REFERENCE_RPM));
  }

  private void stopNow() {
    commandedShotRpm = 0.0;
    requestedSpeedReachable = false;
    smc.setDutyCycle(0.0);
  }

  public Command stop() {
    return runOnce(this::stopNow).withName("Shooter.stop");
  }

  public Command idleCommand() {
    return run(this::stopNow).withName("Shooter.IDLE");
  }

  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
  }

  public Command sysId() {
    return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
  }

  public double getCommandedShotRpm() {
    return commandedShotRpm;
  }

  /** A speed clamped below the required RPM is not a ready shot. */
  public boolean isAtShotSpeed() {
    return requestedSpeedReachable && commandedShotRpm > 0.0
        && Math.abs(getSpeed().in(RPM) - commandedShotRpm) <= ShooterConstants.READY_TOLERANCE_RPM;
  }

  @Override
  public void periodic() {
    RobotLog.put("Shooter/LeaderVelocity", leaderSpark.getEncoder().getVelocity());
    RobotLog.put("Shooter/FollowerVelocity", followerSpark.getEncoder().getVelocity());
    RobotLog.put("Shooter/ActualRPM", getSpeed().in(RPM));
    RobotLog.put("Shooter/CommandedRPM", commandedShotRpm);
    RobotLog.put("Shooter/AtShotSpeed", isAtShotSpeed());
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  public LinearVelocity getTangentialVelocity() {
    double radiusMeters = Inches.of(ShooterConstants.WHEEL_DIAMETER_INCHES).div(2).in(Meters);
    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond) * radiusMeters);
  }
}
