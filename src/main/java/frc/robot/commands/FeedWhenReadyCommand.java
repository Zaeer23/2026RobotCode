package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Feeds only after readiness has stayed true, and stops immediately if it is lost. */
public final class FeedWhenReadyCommand extends Command {
  private static final double READY_SETTLE_SECONDS = 0.10;

  private final BooleanSupplier ready;
  private final Consumer<Boolean> setFeeding;
  private final Timer readyTimer = new Timer();

  public FeedWhenReadyCommand(
      BooleanSupplier ready, Consumer<Boolean> setFeeding, Subsystem... requirements) {
    this.ready = ready;
    this.setFeeding = setFeeding;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    readyTimer.stop();
    readyTimer.reset();
    setFeeding.accept(false);
  }

  @Override
  public void execute() {
    if (!ready.getAsBoolean()) {
      readyTimer.stop();
      readyTimer.reset();
      setFeeding.accept(false);
      return;
    }
    readyTimer.start();
    setFeeding.accept(readyTimer.hasElapsed(READY_SETTLE_SECONDS));
  }

  @Override
  public void end(boolean interrupted) {
    readyTimer.stop();
    readyTimer.reset();
    setFeeding.accept(false);
  }
}
