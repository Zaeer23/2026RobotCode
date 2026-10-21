package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import org.junit.jupiter.api.Test;

class ShooterSubsystemTest {
  /** Actual commands with simulated motor controllers and a controlled flywheel speed reading. */
  @Test
  void fixedRpmReadinessTracksSpeedAndClearsOnStopOrInvalidRequest() {
    assertTrue(HAL.initialize(500, 0));
    TestShooter shooter = new TestShooter();
    try {
      Command spin = shooter.spinUp();
      assertEquals(0.0, shooter.getCommandedShotRpm());
      spin.initialize();
      assertFalse(shooter.isAtShotSpeed());
      spin.execute();
      assertEquals(ShooterConstants.REFERENCE_RPM, shooter.getCommandedShotRpm());
      assertFalse(shooter.isAtShotSpeed(), "an unspun flywheel cannot feed");

      shooter.actualRpm = ShooterConstants.REFERENCE_RPM;
      assertTrue(shooter.isAtShotSpeed());
      shooter.actualRpm -= ShooterConstants.READY_TOLERANCE_RPM + 1.0;
      assertFalse(shooter.isAtShotSpeed(), "a ball slowing the flywheel must pause feed");
      shooter.actualRpm = ShooterConstants.REFERENCE_RPM;
      spin.end(true);
      assertEquals(0.0, shooter.getCommandedShotRpm());
      assertFalse(shooter.isAtShotSpeed(), "coasting at the old speed is not an active shot");

      int[] samples = {0};
      double[] requestRpm = {ShooterConstants.REFERENCE_RPM};
      Command dynamic = shooter.setSpeedDynamic(() -> {
        samples[0]++;
        return RPM.of(requestRpm[0]);
      });
      assertEquals(0, samples[0], "constructing a command must not invoke its speed supplier");
      dynamic.initialize();
      assertEquals(0, samples[0]);
      assertFalse(shooter.isAtShotSpeed());
      dynamic.execute();
      assertEquals(1, samples[0]);
      assertTrue(shooter.isAtShotSpeed());

      for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, -1.0, 0.0}) {
        requestRpm[0] = invalid;
        dynamic.execute();
        assertEquals(0.0, shooter.getCommandedShotRpm());
        assertFalse(shooter.isAtShotSpeed());
      }
      requestRpm[0] = ShooterConstants.MAX_MECHANICAL_RPM + 100.0;
      shooter.actualRpm = ShooterConstants.MAX_MECHANICAL_RPM;
      dynamic.execute();
      assertEquals(ShooterConstants.MAX_MECHANICAL_RPM, shooter.getCommandedShotRpm());
      assertFalse(shooter.isAtShotSpeed(), "a capped motor cannot reach an excessive requested RPM");
      dynamic.end(false);
      assertEquals(0.0, shooter.getCommandedShotRpm());
      assertFalse(shooter.isAtShotSpeed());
    } finally {
      shooter.stop().initialize();
      CommandScheduler.getInstance().unregisterSubsystem(shooter);
    }
  }

  private static final class TestShooter extends ShooterSubsystem {
    private double actualRpm;

    @Override
    public AngularVelocity getSpeed() {
      return RPM.of(actualRpm);
    }
  }
}
