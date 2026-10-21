package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FeedWhenReadyCommandTest {
  private boolean ready;
  private boolean feeding;
  private FeedWhenReadyCommand command;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    command = new FeedWhenReadyCommand(() -> ready, enabled -> feeding = enabled);
    command.initialize();
  }

  @AfterEach
  void tearDown() {
    command.end(true);
    SimHooks.resumeTiming();
  }

  @Test
  void neverFeedsJustBecauseSpinupTakesLongerThanTwoSeconds() {
    for (int cycle = 0; cycle < 200; cycle++) {
      SimHooks.stepTiming(0.02);
      command.execute();
      assertFalse(feeding);
    }
    assertFalse(command.isFinished());
  }

  @Test
  void waitsForStableSpeedAndStopsOnTheFirstNotReadyCycle() {
    ready = true;
    command.execute();
    SimHooks.stepTiming(0.08);
    command.execute();
    assertFalse(feeding);
    SimHooks.stepTiming(0.04);
    command.execute();
    assertTrue(feeding);

    ready = false;
    command.execute();
    assertFalse(feeding);

    ready = true;
    command.execute();
    SimHooks.stepTiming(0.08);
    command.execute();
    assertFalse(feeding, "Recovery must settle again after the previous ball slowed the wheel");
    SimHooks.stepTiming(0.04);
    command.execute();
    assertTrue(feeding);
  }

  @Test
  void releaseStopsFeedAndReschedulingRequiresFreshReadiness() {
    ready = true;
    command.execute();
    SimHooks.stepTiming(0.12);
    command.execute();
    assertTrue(feeding);

    command.end(true);
    assertFalse(feeding);
    command.initialize();
    command.execute();
    assertFalse(feeding);
  }
}
