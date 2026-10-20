package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Rate limiter for telemetry.
 *
 * <p>Robot code runs at 50 Hz, but almost nothing a human looks at needs updating 50 times a
 * second. Publishing everything every loop put roughly 7,700 NetworkTables updates per second on
 * the radio, which it cannot drain. NT4 queues what it cannot send, so the backlog grows, every
 * value arrives progressively later, and the queue itself is unbounded memory.
 *
 * <p>That is worth being precise about: the symptom was not jitter. It was a delay that grew the
 * longer the robot stayed on, because a queue that fills faster than it drains never recovers.
 *
 * <p>Dashboards are comfortable at 10 Hz. Logging to disk is cheap and can stay fast.
 */
public class Throttle {

  /** Comfortable for anything a person reads off a screen. */
  public static final double DASHBOARD_HZ = 10.0;

  /** For values that only matter when something changed. */
  public static final double SLOW_HZ = 2.0;

  private final double periodSeconds;
  private double lastRunSeconds = -1.0;

  public Throttle(double hz) {
    this.periodSeconds = 1.0 / Math.max(0.1, hz);
  }

  /**
   * True when enough time has passed to do the work again.
   *
   * <p>Call in an {@code if} around the publishing block, not around the computation that feeds
   * it, so the robot keeps thinking at full rate and only the reporting slows down.
   */
  public boolean ready() {
    double now = Timer.getFPGATimestamp();
    if (lastRunSeconds < 0.0 || now - lastRunSeconds >= periodSeconds) {
      lastRunSeconds = now;
      return true;
    }
    return false;
  }

  /** Forces the next {@link #ready()} to return true. */
  public void reset() {
    lastRunSeconds = -1.0;
  }
}
