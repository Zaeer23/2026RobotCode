package frc.robot.subsystems;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    public HoodSubsystem() {
  }

  public Command setAngle(Angle angle) {
    // return hood.setAngle(angle);
    return Commands.runOnce(() -> {
    });
  }

  public Command setAngleDynamic(Supplier<Angle> hoodAngleSupplier) {
    // TODO: Uncomment when hood is enabled
    // return hood.setAngle(hoodAngleSupplier);
    return Commands.run(() -> {
    });
  }

  public Command stow() {
    return setAngle(Degrees.of(0));
  }

  public Command max() {
    return setAngle(Degrees.of(90));
  }

  public Angle getAngle() {
    return Degrees.of(75);
    // return hood.getAngle();
  }

  public Command set(double dutyCycle) {
    return Commands.runOnce(() -> {
    });
  }

  public Command sysId() {
    // return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
    return Commands.runOnce(() -> {
    });
  }

  @Override
  public void periodic() {
    // hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // hood.simIterate();
  }
}