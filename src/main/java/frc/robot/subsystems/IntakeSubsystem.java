package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Removed ThriftyNova import
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper; // Only need SparkWrapper now

public class IntakeSubsystem extends SubsystemBase {

  private static final double INTAKE_SPEED = 1.0;

  // 1. Changed roller motor to SparkMax with Brushless (NEO) type
  private final SparkMax rollerMotor = new SparkMax(Constants.IntakeConstants.kRollerMotorId, MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) 
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  // 2. Changed NovaWrapper to SparkWrapper and NeoVortex to NEO
  private final SmartMotorController smc = new SparkWrapper(rollerMotor, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

  private final FlyWheel intake = new FlyWheel(intakeConfig);

  private final SmartMotorControllerConfig intakePivotSmartMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(25, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
      .withFeedforward(new SimpleMotorFeedforward(0, 10, 0))
      .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withSoftLimit(Degrees.of(0), Degrees.of(150))
      .withStatorCurrentLimit(Amps.of(10))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  // 3. Pivot was already SparkMax, but ensured DCMotor is set to NEO (standard)
  private final SparkMax pivotMotor = new SparkMax(Constants.IntakeConstants.kPivotMotorId, MotorType.kBrushless);

  private final SmartMotorController intakePivotController = new SparkWrapper(pivotMotor, DCMotor.getNEO(1),
      intakePivotSmartMotorConfig);

  private final ArmConfig intakePivotConfig = new ArmConfig(intakePivotController)
      .withSoftLimits(Degrees.of(0), Degrees.of(150))
      .withHardLimit(Degrees.of(0), Degrees.of(155))
      .withStartingPosition(Degrees.of(0))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(2))
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

  private final Arm intakePivot = new Arm(intakePivotConfig);

  public IntakeSubsystem() {
      // Configuration is handled by the YAMS SparkWrapper
  }

  public Command intakeCommand() {
    return intake.set(INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Run");
  }

  public Command ejectCommand() {
    return intake.set(-INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Eject");
  }

  public Command setPivotAngle(Angle angle) {
    return intakePivot.setAngle(angle).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotMotor.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
  }

  public Command deployAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      smc.setDutyCycle(INTAKE_SPEED);
    }, this).finallyDo(() -> {
      smc.setDutyCycle(0);
      setIntakeHold();
    }).withName("Intake.DeployAndRoll");
  }

  public Command backFeedAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
    }, this).finallyDo(() -> {
      smc.setDutyCycle(0);
      setIntakeHold();
    }).withName("Intake.BackFeedAndRoll");
  }

  private void setIntakeStow() {
    intakePivotController.setPosition(Degrees.of(0));
  }

  private void setIntakeFeed() {
    intakePivotController.setPosition(Degrees.of(59));
  }

  private void setIntakeHold() {
    intakePivotController.setPosition(Degrees.of(115));
  }

  private void setIntakeDeployed() {
    intakePivotController.setPosition(Degrees.of(148));
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();
    intakePivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
    intakePivot.simIterate();
  }
}