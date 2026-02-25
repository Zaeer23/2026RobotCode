package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
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

public class KickerSubsystem extends SubsystemBase {

  private static final double KICKER_SPEED = 1.0;
// testing?
  // SPARK MAX motor controller with NEO motor
  private SparkMax kickerSparkMax = new SparkMax(Constants.KickerConstants.kKickerMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) // 4:1 gear reduction
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(20));

  private SmartMotorController smc = new SparkWrapper(kickerSparkMax, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig kickerConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Kicker", TelemetryVerbosity.HIGH);

  private FlyWheel kicker = new FlyWheel(kickerConfig);

  public KickerSubsystem() {
  }

  /**
   * Command to run the kicker forward while held, stops when released.
   */
  public Command feedCommand() {
    return kicker.set(KICKER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Kicker.Feed");
  }

  /**
   * Command to stop the kicker.
   */
  public Command stopCommand() {
    return kicker.set(0).withName("Kicker.Stop");
  }

  @Override
  public void periodic() {
    kicker.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    kicker.simIterate();
  }
}