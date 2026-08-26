package frc.robot.subsystems;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Placeholder for a hood that is not built yet.
 *
 * <p>Right now the shooter fires off a fixed plate, so this subsystem reports the one real launch
 * angle the robot has and does nothing else. {@link #isActuated()} tells callers as much, so
 * readiness checks do not sit waiting for a hood that will never move.
 *
 * <p>To bring a real hood online: implement the motor and controller, make {@link #getAngle()}
 * return the measured angle, and flip {@link #isActuated()} to true. The shot solver in
 * {@link ProjectileMotion} already has an optimal-angle mode ready to drive it.
 */
public class HoodSubsystem extends SubsystemBase {

  public HoodSubsystem() {
  }

  /**
   * Whether the hood can actually move.
   *
   * <p>False while the hood is a fixed plate. Readiness logic uses this to skip the hood check
   * entirely instead of comparing a constant against a setpoint that will never be reached.
   */
  public boolean isActuated() {
    return false;
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

  /**
   * The fixed launch angle of the shooter.
   *
   * <p>This returned a hardcoded 75 degrees while every shot calculation in the project assumed 45,
   * so telemetry and physics disagreed by 30 degrees. It now reports the same constant the shot
   * model uses.
   */
  public Angle getAngle() {
    return Degrees.of(ShooterConstants.LAUNCH_ANGLE_DEGREES);
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
