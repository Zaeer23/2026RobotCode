package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import yams.motorcontrollers.local.SparkWrapper; 

import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
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

public class HopperSubsystem extends SubsystemBase {

  private static final double HOPPER_SPEED = 1.0;

  
  private final SparkMax hopperSparkMax = new SparkMax(Constants.HopperConstants.kHopperMotorId, MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) 
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.20))
      .withOpenLoopRampRate(Seconds.of(0.20));

  
  private final SmartMotorController smc = new SparkWrapper(hopperSparkMax, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig hopperConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(1500))
      .withLowerSoftLimit(RPM.of(-1500))
      .withTelemetry("Hopper", TelemetryVerbosity.HIGH);

  private final FlyWheel hopper = new FlyWheel(hopperConfig);


  public Command feedCommand() {
    return hopper.set(HOPPER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Hopper.Feed");
  }

  public Command backFeedCommand() {
    return hopper.set(-HOPPER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Hopper.BackFeed");
  }

  public Command reverseCommand() {
    return hopper.set(-HOPPER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Hopper.Reverse");
  }

  public Command stopCommand() {
    return hopper.set(0).withName("Hopper.Stop");
  }

  @Override
  public void periodic() {
    hopper.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hopper.simIterate();
  }
}
