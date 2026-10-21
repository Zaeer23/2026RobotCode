package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Superstructure;

/** Bounded autonomous shot using the same speed recovery and intake clearance checks as teleop. */
public class ShootingCommand extends SequentialCommandGroup {
  private static final double SHOT_TIMEOUT_SECONDS = 7.5;

  public ShootingCommand(Superstructure superstructure) {
    addCommands(superstructure.shootWhenReadyCommand().withTimeout(SHOT_TIMEOUT_SECONDS));
  }
}
