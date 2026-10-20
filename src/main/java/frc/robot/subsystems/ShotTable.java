package frc.robot.subsystems;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.ShooterConstants;

/**
 * Distance to flywheel RPM, as a table you edit by hand.
 *
 * <h2>Why a table instead of a formula</h2>
 *
 * <p>This replaced a physics model scaled by a single back-solved efficiency factor. The physics
 * was right in shape but wrong in detail, because it models a point mass leaving at a clean angle
 * and ignores everything that actually matters on a real shooter: ball compression, wheel wear,
 * how much the ball slips, air, and a hood that is not quite the angle anyone wrote down. One
 * global fudge factor cannot absorb all of that, so the curve was accurate near the single
 * reference shot and drifted at both ends.
 *
 * <p>A table has no such pretension. Every entry is a measurement. If shots at 3 m are long, you
 * lower the 3 m number and nothing else moves — which is exactly the edit you want to make at
 * 11pm in a shop, and is impossible with an efficiency constant that shifts the whole curve.
 *
 * <h2>Tuning it</h2>
 *
 * <ol>
 *   <li>Park at a tape-measured distance. Check {@code Shooter/DistanceFeet} agrees; if it does
 *       not, fix the range first with BACK+B rather than compensating here.
 *   <li>Shoot. Long means lower the RPM for that row, short means raise it.
 *   <li>Edit the nearest row below, redeploy, repeat.
 * </ol>
 *
 * <p>Between rows the value is linearly interpolated; outside the ends it is clamped to the
 * nearest row, which is why the table should span slightly more than the range you shoot from.
 */
public final class ShotTable {

  private ShotTable() {
  }

  /**
   * Measured shots: horizontal metres from the shooter exit to the goal, and the flywheel RPM
   * that scored from there.
   *
   * <p>Seeded from projectile physics anchored on the one confirmed shot (3200 RPM at 3.51 m) and
   * then shaded down across the board, because shots were consistently going long. These are a
   * starting point, not gospel. Replace each row with a real measurement as you take it.
   */
  private static final double[][] MEASURED_SHOTS = {
      // distance (m), RPM
      {1.00, 2150},
      {1.50, 2270},
      {2.00, 2450},
      {2.50, 2640},
      {3.00, 2830},
      {3.51, 3010},   // the confirmed reference shot, shaded for the observed overshoot
      {4.00, 3180},
      {4.50, 3350},
      {5.00, 3520},
  };

  private static final InterpolatingDoubleTreeMap TABLE = buildTable();

  private static InterpolatingDoubleTreeMap buildTable() {
    InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
    for (double[] row : MEASURED_SHOTS) {
      table.put(row[0], row[1]);
    }
    return table;
  }

  /**
   * Flywheel RPM for a given shot distance.
   *
   * <p>Interpolates between measured rows, and clamps to the nearest row outside the table rather
   * than extrapolating. Extrapolating a shot curve past the last measurement is how you get a
   * confident number for a shot nobody has ever taken.
   *
   * @param shooterDistanceMeters horizontal distance from the ball exit to the goal
   * @return RPM, clamped to the shooter's commandable range
   */
  public static double rpmForDistance(double shooterDistanceMeters) {
    if (!(shooterDistanceMeters > 0.0) || !Double.isFinite(shooterDistanceMeters)) {
      return 0.0;
    }
    double rpm = TABLE.get(shooterDistanceMeters);
    return Math.max(ShooterConstants.MIN_SHOT_RPM,
        Math.min(ShooterConstants.MAX_SHOT_RPM, rpm));
  }

  /** Raw table value before clamping, so telemetry can show when a shot is out of range. */
  public static double rawRpmForDistance(double shooterDistanceMeters) {
    if (!(shooterDistanceMeters > 0.0) || !Double.isFinite(shooterDistanceMeters)) {
      return 0.0;
    }
    return TABLE.get(shooterDistanceMeters);
  }

  /** Shortest distance in the table. Shots closer than this all use the first row. */
  public static double minDistanceMeters() {
    return MEASURED_SHOTS[0][0];
  }

  /** Longest distance in the table. Shots further than this all use the last row. */
  public static double maxDistanceMeters() {
    return MEASURED_SHOTS[MEASURED_SHOTS.length - 1][0];
  }

  /** The table as an ordered map, for printing. */
  public static Map<Double, Double> entries() {
    TreeMap<Double, Double> out = new TreeMap<>();
    for (double[] row : MEASURED_SHOTS) {
      out.put(row[0], row[1]);
    }
    return out;
  }

  /** Prints the table so it can be checked against what the robot is actually doing. */
  public static void print() {
    System.out.println("=== SHOT TABLE ===");
    System.out.println("  distance        RPM");
    for (double[] row : MEASURED_SHOTS) {
      System.out.printf("  %.2f m (%4.1f ft)  %6.0f%n", row[0], row[0] * 3.28084, row[1]);
    }
    System.out.printf("  outside %.2f-%.2f m the nearest row is held, never extrapolated%n",
        minDistanceMeters(), maxDistanceMeters());
    System.out.println("==================");
  }
}
