package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootingCommand extends SequentialCommandGroup {
  private static final double SPINUP_SECONDS = 1.50;
  /**
   * Was 17 seconds — longer than the entire 15 second autonomous period, so the sequence could
   * never reach the steps after it and the shooter never stopped on its own.
   */
  private static final double FIRST_FEED_SECONDS = 6.0;
  private static final double STOP_BACKFEED_SECONDS = 1.00;
  private static final double SECOND_FEED_SECONDS = 1.00;

  public ShootingCommand(
      ShooterSubsystem shooter,
      KickerSubsystem kicker,
      HopperSubsystem hopper) {
    addCommands(
        Commands.deadline(
            Commands.waitSeconds(SPINUP_SECONDS),
            shooter.spinUp()),
        Commands.deadline(
            Commands.waitSeconds(FIRST_FEED_SECONDS),
            shooter.spinUp(),
            kicker.feedCommand(),
            hopper.feedCommand()),
        
        Commands.deadline(
            Commands.waitSeconds(STOP_BACKFEED_SECONDS),
            shooter.spinUp(),
            kicker.stopCommand(),
            hopper.stopCommand()),
        
        
        Commands.parallel(
            shooter.stop(),
            kicker.stopCommand(),
            hopper.stopCommand()));
  }
}
