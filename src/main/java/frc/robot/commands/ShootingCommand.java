package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootingCommand extends SequentialCommandGroup {
  private static final double SPINUP_SECONDS = 2.0;
  private static final double FIRST_FEED_SECONDS = 2.50;
  private static final double FIRST_BACKFEED_SECONDS = 1.00;
  private static final double STOP_BACKFEED_SECONDS = 1.00;
  private static final double SECOND_FEED_SECONDS = 1.00;
  private static final double SECOND_BACKFEED_SECONDS = 1.00;

  public ShootingCommand(
      ShooterSubsystem shooter,
      LimeLight limelight,
      KickerSubsystem kicker,
      HopperSubsystem hopper,
      Set<Integer> hubTagIds) {
    addCommands(
        Commands.deadline(
            Commands.waitSeconds(SPINUP_SECONDS),
            shooter.setSpeedFromLimelight(limelight, 45.0, hubTagIds)),
        Commands.deadline(
            Commands.waitSeconds(FIRST_FEED_SECONDS),
            shooter.setSpeedFromLimelight(limelight, 45.0, hubTagIds),
            kicker.feedCommand(),
            hopper.feedCommand()),
        
        Commands.deadline(
            Commands.waitSeconds(STOP_BACKFEED_SECONDS),
            shooter.setSpeedFromLimelight(limelight, 45.0, hubTagIds),
            kicker.stopCommand(),
            hopper.stopCommand()),
        
        
        Commands.parallel(
            shooter.stop(),
            kicker.stopCommand(),
            hopper.stopCommand()));
  }
}
