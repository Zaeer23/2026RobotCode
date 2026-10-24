package frc.robot.cyberpsycho;

/**
 * Watches one D-pad for UP UP DOWN DOWN LEFT RIGHT LEFT RIGHT.
 *
 * <p>Fed the raw POV angle every loop, it counts PRESSES, not samples: holding a direction for a
 * second is one press, the same as tapping it. A press is the D-pad moving onto a new cardinal
 * direction.
 *
 * <p>Diagonals are ignored rather than treated as a release. Thumbs are sloppy, and a quick UP
 * often reads as UP, UP-RIGHT, UP for a frame or two; counting that as two presses would make the
 * code unreliable to enter on purpose. The cost is that a direction repeated on purpose (the two
 * UPs, the two DOWNs) needs the pad to come back to centre in between, which it does anyway.
 *
 * <p>It matches the LAST eight presses, so stray presses before the code do not matter and a
 * wrong press only costs the presses since. A gap longer than {@link #MAX_GAP_SECONDS} starts the
 * attempt over, so D-pad presses scattered across a match can never add up to the code by
 * accident.
 *
 * <p>Pure logic with no WPILib calls, so it is unit tested without a robot.
 */
public final class KonamiCode {

  /** The four D-pad directions that count as presses. */
  public enum Press {
    UP, DOWN, LEFT, RIGHT
  }

  /** The code itself, in order. */
  private static final Press[] SEQUENCE = {
      Press.UP, Press.UP, Press.DOWN, Press.DOWN,
      Press.LEFT, Press.RIGHT, Press.LEFT, Press.RIGHT
  };

  /** Longest pause allowed between two presses before the attempt starts over. */
  public static final double MAX_GAP_SECONDS = 1.5;

  /** The last presses, oldest at {@link #head} once the buffer is full. */
  private final Press[] recent = new Press[SEQUENCE.length];
  private int head = 0;
  private int count = 0;

  /** Direction the pad is on right now, or null when centred. Diagonals leave it alone. */
  private Press held = null;

  private double lastPressSeconds = Double.NEGATIVE_INFINITY;

  /**
   * Takes this loop's D-pad reading.
   *
   * @param povDegrees  the POV angle in degrees, or -1 when the pad is centred
   * @param nowSeconds  a monotonic timestamp
   * @return true on the one loop the final press of the code lands, false every other time
   */
  public boolean update(int povDegrees, double nowSeconds) {
    if (povDegrees >= 0 && povDegrees % 90 != 0) {
      return false;
    }

    Press now = fromPov(povDegrees);
    if (now == held) {
      return false;
    }
    held = now;
    if (now == null) {
      return false;
    }

    if (nowSeconds - lastPressSeconds > MAX_GAP_SECONDS) {
      count = 0;
    }
    lastPressSeconds = nowSeconds;

    recent[head] = now;
    head = (head + 1) % recent.length;
    if (count < recent.length) {
      count++;
    }
    if (count < recent.length) {
      return false;
    }

    for (int i = 0; i < SEQUENCE.length; i++) {
      if (recent[(head + i) % recent.length] != SEQUENCE[i]) {
        return false;
      }
    }

    // Consumed, so the same eight presses cannot fire it again.
    count = 0;
    return true;
  }

  private static Press fromPov(int povDegrees) {
    switch (povDegrees) {
      case 0:
        return Press.UP;
      case 90:
        return Press.RIGHT;
      case 180:
        return Press.DOWN;
      case 270:
        return Press.LEFT;
      default:
        return null;
    }
  }
}
