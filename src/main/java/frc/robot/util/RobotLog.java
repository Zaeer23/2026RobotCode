package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Safe replacement for AdvantageKit's {@code Logger.recordOutput}.
 *
 * <h2>Why this exists</h2>
 *
 * <p>The project called {@code Logger.recordOutput(...)} from 73 sites, every loop, while
 * {@code Logger.start()} was never called and {@code Robot} extends {@link
 * edu.wpi.first.wpilibj.TimedRobot} rather than AdvantageKit's {@code LoggedRobot}. AdvantageKit
 * drives its own lifecycle from {@code LoggedRobot}, so nothing was ever draining what those calls
 * queued up. It accumulated until the roboRIO ran out of heap and the robot died mid-match.
 *
 * <p>Worth being clear: those calls were not merely leaking, they were also doing nothing useful.
 * With no receivers and no started logger there was no log being written, so removing them costs
 * no capability at all.
 *
 * <p>This writes to SmartDashboard instead, rate limited, with no queue and no growth. Values are
 * keyed, so repeated writes to the same name overwrite rather than accumulate.
 *
 * <p>To adopt AdvantageKit properly later: change {@code Robot} to extend {@code LoggedRobot},
 * call {@code Logger.addDataReceiver(new WPILOGWriter(...))} and {@code Logger.start()} in
 * {@code robotInit}, then point this class at {@code Logger} again.
 */
public final class RobotLog {

  /** Minimum gap between publishes OF THE SAME KEY, in seconds. 10 Hz. */
  private static final double PERIOD_SECONDS = 1.0 / Throttle.DASHBOARD_HZ;

  /**
   * Last publish time per key.
   *
   * <p>Per-key, not one shared throttle. A single shared throttle lets only ONE key through per
   * window, so with 73 call sites each value would update roughly once every seven seconds, on a
   * rotating basis -- every reading on the dashboard would look frozen or stale at random.
   *
   * <p>Bounded by the number of distinct keys, which is fixed at compile time, so this cannot
   * grow without limit the way the thing it replaced did.
   */
  private static final java.util.concurrent.ConcurrentHashMap<String, Double> LAST_PUBLISH =
      new java.util.concurrent.ConcurrentHashMap<>();

  /** Set false to silence all telemetry, e.g. to rule it out while chasing a performance problem. */
  private static volatile boolean enabled = true;

  private RobotLog() {
  }

  public static void setEnabled(boolean value) {
    enabled = value;
  }

  private static boolean ready(String key) {
    if (!enabled) {
      return false;
    }
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    Double last = LAST_PUBLISH.get(key);
    if (last != null && now - last < PERIOD_SECONDS) {
      return false;
    }
    LAST_PUBLISH.put(key, now);
    return true;
  }

  public static void put(String key, double value) {
    if (ready(key)) {
      SmartDashboard.putNumber(key, value);
    }
  }

  public static void put(String key, boolean value) {
    if (ready(key)) {
      SmartDashboard.putBoolean(key, value);
    }
  }

  public static void put(String key, String value) {
    if (ready(key)) {
      SmartDashboard.putString(key, value);
    }
  }

  public static void put(String key, int value) {
    put(key, (double) value);
  }

  public static void put(String key, long value) {
    put(key, (double) value);
  }

  /** Arrays are summarised rather than published; full arrays are rarely read on a dashboard. */
  public static void put(String key, double[] value) {
    if (ready(key)) {
      SmartDashboard.putNumber(key + "/count", value == null ? 0 : value.length);
    }
  }

  public static void put(String key, int[] value) {
    if (ready(key)) {
      SmartDashboard.putNumber(key + "/count", value == null ? 0 : value.length);
    }
  }

  /** Objects are not published. Kept so call sites that logged poses still compile. */
  public static void put(String key, Object value) {
    // Deliberately a no-op: pose and struct logging needs a real logger, not SmartDashboard.
  }
}
