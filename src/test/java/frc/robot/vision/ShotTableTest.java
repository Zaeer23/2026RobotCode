package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShotTable;
import org.junit.jupiter.api.Test;

/** The shot curve must rise with distance and never extrapolate past measured data. */
class ShotTableTest {

  @Test
  void rpmRisesWithDistance() {
    double previous = -1.0;
    for (double d = ShotTable.minDistanceMeters();
         d <= ShotTable.maxDistanceMeters();
         d += 0.1) {
      double rpm = ShotTable.rawRpmForDistance(d);
      assertTrue(rpm > previous,
          String.format("RPM must increase with distance, but %.2f m gave %.0f after %.0f",
              d, rpm, previous));
      previous = rpm;
    }
  }

  @Test
  void clampsRatherThanExtrapolatingOutsideTheTable() {
    double atMin = ShotTable.rawRpmForDistance(ShotTable.minDistanceMeters());
    double atMax = ShotTable.rawRpmForDistance(ShotTable.maxDistanceMeters());
    assertEquals(atMin, ShotTable.rawRpmForDistance(0.1), 1e-6, "closer than the table");
    assertEquals(atMax, ShotTable.rawRpmForDistance(50.0), 1e-6, "further than the table");
  }

  @Test
  void everyTableValueIsReachableByTheFlywheel() {
    for (var entry : ShotTable.entries().entrySet()) {
      assertTrue(entry.getValue() <= ShooterConstants.MAX_MECHANICAL_RPM,
          String.format("%.2f m asks for %.0f RPM, above the %.0f ceiling",
              entry.getKey(), entry.getValue(), ShooterConstants.MAX_MECHANICAL_RPM));
      assertTrue(entry.getValue() > 0.0);
    }
  }

  @Test
  void invalidDistanceGivesZeroRatherThanAGuess() {
    assertEquals(0.0, ShotTable.rawRpmForDistance(-1.0));
    assertEquals(0.0, ShotTable.rawRpmForDistance(0.0));
    assertEquals(0.0, ShotTable.rawRpmForDistance(Double.NaN));
  }

  @Test
  void outputIsClampedToTheCommandableRange() {
    for (double d = 0.5; d <= 10.0; d += 0.25) {
      double rpm = ShotTable.rpmForDistance(d);
      assertTrue(rpm >= ShooterConstants.MIN_SHOT_RPM && rpm <= ShooterConstants.MAX_SHOT_RPM,
          String.format("%.2f m gave %.0f, outside [%.0f, %.0f]",
              d, rpm, ShooterConstants.MIN_SHOT_RPM, ShooterConstants.MAX_SHOT_RPM));
    }
  }
}
