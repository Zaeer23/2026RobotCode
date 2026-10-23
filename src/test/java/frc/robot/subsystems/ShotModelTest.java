package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.ShooterConstants;
import org.junit.jupiter.api.Test;

/**
 * Pins the shot curve to the shots actually measured on this robot.
 *
 * <p>The point of replacing the lookup table was to keep going past the last measured row with
 * the right shape. That is only worth anything if the curve still agrees with the rows it
 * replaced, so those are asserted here as the contract: change the fit constants and these fail
 * until the fit is redone against real data.
 */
public class ShotModelTest {

  /** The nine shots the old table was built from: distance in metres, flywheel RPM. */
  private static final double[][] MEASURED = {
    {1.00, 2150}, {1.50, 2270}, {2.00, 2450}, {2.50, 2640}, {3.00, 2830},
    {3.51, 3010}, {4.00, 3180}, {4.50, 3350}, {5.00, 3520},
  };

  @Test
  public void reproducesEveryMeasuredShot() {
    for (double[] row : MEASURED) {
      double modelled = ShotModel.idealRpm(row[0]);
      assertEquals(row[1], modelled, 20.0,
          String.format("%.2f m: measured %.0f RPM, model says %.0f", row[0], row[1], modelled));
    }
  }

  @Test
  public void rpmRisesWithDistance() {
    double previous = 0;
    for (double d = 1.0; d <= 5.0; d += 0.25) {
      double rpm = ShotModel.idealRpm(d);
      assertTrue(rpm > previous, "RPM must increase with distance, broke at " + d + " m");
      previous = rpm;
    }
  }

  @Test
  public void extrapolatesPastTheLastMeasuredRow() {
    // The whole reason for replacing the table. It clamped everything past 5 m to the 5 m speed,
    // so a shot from 5.5 m and one from 9 m came out identical.
    double atFive = ShotModel.idealRpm(5.0);
    assertTrue(ShotModel.idealRpm(5.5) > atFive, "5.5 m must ask for more than 5.0 m");
    assertTrue(ShotModel.idealRpm(7.0) > ShotModel.idealRpm(5.5), "and 7 m more again");
  }

  @Test
  public void reportsShotsThatTheFlywheelCannotReach() {
    double max = ShotModel.maxRangeMeters();
    assertTrue(ShotModel.isWithinRange(max - 0.2), "just inside the ceiling should be reachable");
    assertFalse(ShotModel.isWithinRange(max + 0.2), "past the ceiling must not claim reachable");
  }

  @Test
  public void clampedRpmNeverExceedsTheMechanicalCeiling() {
    // An out-of-reach shot still has to command something legal rather than a number the
    // flywheel would refuse or try to chase.
    for (double d = 1.0; d <= 12.0; d += 0.5) {
      double rpm = ShotModel.rpmForDistance(d);
      assertTrue(rpm <= ShooterConstants.MAX_SHOT_RPM, d + " m commanded " + rpm);
    }
  }

  @Test
  public void aDistanceWithNoTargetProducesNoShot() {
    // getHubDistanceMeters() returns -1 when no HUB tag is in view. That must not become a
    // plausible-looking RPM.
    assertEquals(0.0, ShotModel.idealRpm(-1.0), 1e-9);
    assertEquals(0.0, ShotModel.rpmForDistance(-1.0), 1e-9);
    assertEquals(0.0, ShotModel.idealRpm(Double.NaN), 1e-9);
  }

  @Test
  public void theOutpostShotIsReachableNowTheCeilingIsHonest() {
    // Outpost opening (0.50, 0.666) to hub centre (4.626, 4.035) is 5.33 m. At the old 3600 RPM
    // ceiling this was 4 cm out of reach, and that was an artificial limit rather than anything
    // the hardware imposed: the flywheel is two NEOs at 1:1, free speed 5676. With the ceiling
    // set from the feedforward instead, the shot is comfortably available.
    double outpostToHub = Math.hypot(4.626 - 0.50, 4.035 - 0.666);
    assertEquals(5.33, outpostToHub, 0.02);
    assertTrue(ShotModel.isWithinRange(outpostToHub),
        "outpost shot needs " + String.format("%.0f", ShotModel.idealRpm(outpostToHub))
            + " RPM against a ceiling of " + ShooterConstants.MAX_SHOT_RPM);
  }

  @Test
  public void reachCoversTheLongShotsTheDriversAskedFor() {
    // 7.5 m was the shot the team said they could already make. It needs 4218 RPM, which the old
    // 3600 ceiling refused outright.
    assertTrue(ShotModel.isWithinRange(7.5),
        "7.5 m needs " + String.format("%.0f", ShotModel.idealRpm(7.5)) + " RPM");
    assertTrue(ShotModel.maxRangeMeters() > 7.5,
        "max range is only " + String.format("%.2f", ShotModel.maxRangeMeters()) + " m");
  }
}
