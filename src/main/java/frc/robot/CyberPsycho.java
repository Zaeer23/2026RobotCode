package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * A one-way switch into drive-everything mode, armed by a D-pad sequence.
 *
 * <p>Up, up, down, down, left, right, left, right on the driver's D-pad. Once it latches the robot
 * gives the drivetrain everything it has, stops commanding every mechanism, and turns the camera
 * into a plain video feed.
 *
 * <p><b>There is no way out.</b> That is the point, and it is also the reason the sequence is
 * eight specific presses in order rather than a button: nothing accidental enters this. A reboot
 * clears it, because the latch lives in memory and is never persisted — booting into a mode with
 * no mechanisms and no speed limit is not something anyone should be able to do by surprise.
 *
 * <p>Why it is safe to be irreversible: this mode <em>removes</em> capability rather than adding
 * it. Nothing is commanded that was not commanded before, the mechanisms are released rather than
 * driven, and the only thing that grows is the speed cap the driver already had their hand on.
 */
public final class CyberPsycho {

  /** POV angles in degrees: up, up, down, down, left, right, left, right. */
  private static final int[] SEQUENCE = {0, 0, 180, 180, 270, 90, 270, 90};

  /**
   * How long a partial sequence survives without the next press.
   *
   * <p>Without this a stray D-pad press from any point in a match sits in the buffer forever and
   * eventually lines up with seven more by coincidence.
   */
  private static final double STEP_TIMEOUT_SECONDS = 1.5;

  /** NT key the dashboard watches to know it should go haywire. */
  public static final String NT_KEY = "CyberPsycho/Active";

  private static boolean active = false;
  private static int progress = 0;
  private static int lastPov = -1;
  private static final Timer sinceLastStep = new Timer();

  private CyberPsycho() {}

  /** True once the sequence has been entered. Never goes back to false without a reboot. */
  public static boolean isActive() {
    return active;
  }

  /**
   * Feeds both controllers' D-pads in, taking whichever is actually pressed.
   *
   * <p>Either pad can enter it on purpose. A controller that is not plugged in reports -1
   * forever, and this robot has had its driver pad unplugged before while every binding on it
   * silently did nothing — a sequence that only one pad could enter would be unenterable exactly
   * when someone most wanted it.
   */
  public static void update(int driverPov, int operatorPov) {
    update(driverPov >= 0 ? driverPov : operatorPov);
  }

  /**
   * Feeds the current D-pad reading in. Call every loop.
   *
   * @param pov the POV angle in degrees, or -1 when nothing is pressed
   */
  public static void update(int pov) {
    publish();

    if (active) {
      return;
    }

    // Edge-triggered: a held direction is one press, not sixty.
    if (pov == lastPov) {
      if (progress > 0 && sinceLastStep.hasElapsed(STEP_TIMEOUT_SECONDS)) {
        progress = 0;
      }
      return;
    }
    lastPov = pov;

    if (pov < 0) {
      return;
    }

    if (progress > 0 && sinceLastStep.hasElapsed(STEP_TIMEOUT_SECONDS)) {
      progress = 0;
    }

    if (pov == SEQUENCE[progress]) {
      progress++;
      sinceLastStep.restart();
      // Say so on every accepted press. Entering a long sequence blind, with no idea whether any
      // of it registered, is how someone decides it is broken when they simply mistimed a press.
      System.out.println("[CYBERPSYCHO] " + progress + "/" + SEQUENCE.length);
      if (progress == SEQUENCE.length) {
        latch();
      }
    } else {
      // A wrong press is not always a reset: it might be the first step of a fresh attempt.
      progress = (pov == SEQUENCE[0]) ? 1 : 0;
      sinceLastStep.restart();
    }
  }

  private static void latch() {
    active = true;
    publish();
    DriverStation.reportWarning(
        "[CYBERPSYCHO] ENGAGED. Drivetrain unlocked to the motors' limit, all mechanisms "
            + "released, camera is a plain video feed. This cannot be switched off — reboot the "
            + "robot code to leave it.", false);
    System.out.println("""

        ============================================================
          C Y B E R P S Y C H O
          All power to the wheels. Nothing else answers.
          No way out but a reboot.
        ============================================================
        """);
  }

  private static void publish() {
    var table = NetworkTableInstance.getDefault().getTable(frc.robot.DashboardTelemetry.TABLE);
    table.getEntry(NT_KEY).setBoolean(active);
    // Progress as well, so a half-entered sequence is visible rather than invisible.
    table.getEntry("CyberPsycho/Progress").setDouble(progress);
  }
}
