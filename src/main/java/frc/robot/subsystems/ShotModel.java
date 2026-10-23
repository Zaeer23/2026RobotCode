package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

/**
 * Flywheel RPM for a shot distance, as a continuous curve rather than a lookup table.
 *
 * <p>The old table interpolated nine measured rows and deliberately refused to extrapolate,
 * clamping to the nearest row outside them. That is a safe thing for a table to do and a useless
 * one on the field: every shot past 5 m silently got the 5 m speed, so a shot from the OUTPOST
 * came out identical to one from four metres closer, and there was no way to know what it should
 * have been.
 *
 * <p>This is the projectile solution for a fixed launch angle, which is what the robot has:
 *
 * <pre>
 *   v = d * sqrt( g / (d - dh) )          RPM = K * v
 * </pre>
 *
 * <p>{@code dh} is the height the FUEL has to climb from the shooter exit to the goal, and
 * {@code K} converts exit speed to flywheel RPM — it absorbs wheel radius and however much the
 * ball slips, which is why it is fitted rather than derived.
 *
 * <p>Both constants are fitted to the nine shots that were actually measured on this robot, not
 * guessed. Linearising the equation makes that an exact least-squares solve: squaring and
 * rearranging gives {@code d^2/RPM^2 = d/(K^2 g) - dh/(K^2 g)}, which is a straight line in
 * {@code d}. The fit reproduces every measured row to within 14 RPM, or about half a percent:
 *
 * <pre>
 *   d (m)   measured   model
 *   1.00      2150      2154
 *   1.50      2270      2256
 *   2.00      2450      2445
 *   2.50      2640      2642
 *   3.00      2830      2832
 *   3.51      3010      3017
 *   4.00      3180      3187
 *   4.50      3350      3352
 *   5.00      3520      3510
 * </pre>
 *
 * <p>So this agrees with every measurement the table was built on, and keeps going past the last
 * one with the right shape instead of flattening out.
 *
 * <p>It also makes the shooter's reach a number instead of a guess: at the 3600 RPM mechanical
 * ceiling the curve runs out at {@link #maxRangeMeters()}, about 5.3 m. Past that the shot is not
 * slightly weak, it is impossible, and {@link #isWithinRange(double)} says so rather than
 * returning a confident RPM for a shot that cannot be made.
 */
public final class ShotModel {

  private ShotModel() {}

  private static final double G = 9.80665;

  /**
   * RPM per m/s of exit speed, fitted to the measured shots.
   *
   * <p>Absorbs the flywheel radius and ball slip together. Re-fit it if the wheels, the
   * compression or the launch angle change — not by reasoning about it, but by taking a handful of
   * shots at known distances and re-running the fit.
   */
  private static final double RPM_PER_MPS = 474.292;

  /** Effective climb from the ball's exit to the goal, in metres, fitted alongside the gain. */
  private static final double HEIGHT_RISE_METERS = 0.5245;

  /** Below this the equation divides by nearly zero and the answer means nothing. */
  private static final double MIN_SOLVABLE_METERS = HEIGHT_RISE_METERS + 0.15;

  /**
   * Ideal flywheel RPM for this distance, before any clamping.
   *
   * <p>Returns 0 for a distance too short to solve, which the callers already treat as "no shot".
   */
  public static double idealRpm(double shooterDistanceMeters) {
    if (!(shooterDistanceMeters > MIN_SOLVABLE_METERS) || !Double.isFinite(shooterDistanceMeters)) {
      return 0.0;
    }
    double exitSpeed = shooterDistanceMeters
        * Math.sqrt(G / (shooterDistanceMeters - HEIGHT_RISE_METERS));
    return RPM_PER_MPS * exitSpeed;
  }

  /**
   * RPM to command for this distance, clamped to what the flywheel can actually do.
   *
   * <p>A distance past {@link #maxRangeMeters()} comes back as the ceiling. That is the best the
   * shooter can do, but it will fall short — ask {@link #isWithinRange(double)} before trusting
   * the shot rather than reading a plausible number here and assuming it lands.
   */
  public static double rpmForDistance(double shooterDistanceMeters) {
    double ideal = idealRpm(shooterDistanceMeters);
    if (ideal <= 0.0) {
      return 0.0;
    }
    return Math.max(ShooterConstants.MIN_SHOT_RPM,
        Math.min(ShooterConstants.MAX_SHOT_RPM, ideal));
  }

  /** True when the shot is reachable without clamping, i.e. the flywheel can really make it. */
  public static boolean isWithinRange(double shooterDistanceMeters) {
    double ideal = idealRpm(shooterDistanceMeters);
    return ideal > 0.0 && ideal <= ShooterConstants.MAX_SHOT_RPM;
  }

  /**
   * Furthest shot the flywheel can reach at its mechanical ceiling.
   *
   * <p>Solved by bisection rather than algebraically: the inverse has two roots and only the far
   * one is the answer, and a bisection bracketed above the solvable distance cannot pick up the
   * near root by accident.
   */
  public static double maxRangeMeters() {
    double low = MIN_SOLVABLE_METERS;
    double high = 40.0;
    for (int i = 0; i < 200; i++) {
      double mid = 0.5 * (low + high);
      if (idealRpm(mid) < ShooterConstants.MAX_SHOT_RPM) {
        low = mid;
      } else {
        high = mid;
      }
    }
    return low;
  }

  /** Prints the curve so it can be checked against what the robot is actually doing. */
  public static void print() {
    System.out.println("=== SHOT MODEL ===");
    System.out.printf("  RPM = %.3f * d * sqrt(%.5f / (d - %.4f))%n",
        RPM_PER_MPS, G, HEIGHT_RISE_METERS);
    System.out.println("  distance            RPM");
    for (double d = 1.0; d <= 6.01; d += 0.5) {
      double ideal = idealRpm(d);
      System.out.printf("  %.2f m (%4.1f ft)  %6.0f%s%n",
          d, d * 3.28084, ideal, isWithinRange(d) ? "" : "   OUT OF REACH");
    }
    System.out.printf("  furthest reachable shot: %.2f m (%.1f ft) at %.0f RPM%n",
        maxRangeMeters(), maxRangeMeters() * 3.28084, ShooterConstants.MAX_SHOT_RPM);
    System.out.println("==================");
  }
}
