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

  /** Dashboard writes 0, 1 or 2 here. Two is armed; anything less backs out. */
  public static final String NT_CONFIRMATIONS = "CyberPsycho/Confirmations";

  /** Robot tells the dashboard it is armed and waiting for AUTO. */
  public static final String NT_ARMED = "CyberPsycho/Armed";

  /** Set once AUTO has finished in this mode, which is when the camera takes the screen. */
  public static final String NT_AUTO_DONE = "CyberPsycho/AutoDone";

  /** How many confirmations arming takes. */
  public static final int CONFIRMATIONS_REQUIRED = 2;

  private static boolean armed = false;
  private static boolean autoDone = false;
  private static boolean active = false;
  private static int progress = 0;
  private static int lastPov = -1;
  private static final Timer sinceLastStep = new Timer();

  private CyberPsycho() {}

  /**
   * Power levels, selected with the bumpers once the mode is live.
   *
   * <p>The scale multiplies the chassis velocity the modules can produce. There is no level above
   * 1.0 because there is nothing above it: the modules do 5.45 m/s and asking for more only means
   * asking for a number the drive cannot reach.
   *
   * <p>What varies with these is how hard the motors are worked, and the sources on NEOs do not
   * agree. A full-size NEO is generally reckoned to survive a whole match at 40 A; 60 A is the
   * usual drivetrain ceiling and what this robot already runs; one view on Chief Delphi puts
   * failure at 60 A within seconds, which is about sustained stall rather than driving. The
   * distinction that matters: driving fast on carpet draws far less than pushing into something
   * that will not move, and a plow spends its life doing the second.
   *
   * <p>So EASE exists as a real option rather than a courtesy. If the motors are getting hot,
   * dropping a level is the lever, and it is on a bumper for a reason.
   */
  public enum Power {
    EASE(0.70, "EASE"),
    FULL(1.00, "FULL");

    public final double scale;
    public final String label;

    Power(double scale, String label) {
      this.scale = scale;
      this.label = label;
    }
  }

  private static Power power = Power.FULL;

  /** Current power level. Only meaningful while the mode is live. */
  public static Power power() {
    return power;
  }

  /**
   * Steps the power level, but only in CYBERPSYCHO.
   *
   * <p>Deliberately inert otherwise. Outside the mode the dashboard's speed selector is what sets
   * the cap, and a bumper that silently overrode it would mean two controls fighting over the
   * same number with no indication of which won.
   */
  public static void stepPower(int direction) {
    if (!active) {
      return;
    }
    Power[] levels = Power.values();
    int index = Math.max(0, Math.min(levels.length - 1, power.ordinal() + direction));
    if (levels[index] != power) {
      power = levels[index];
      System.out.println("[CYBERPSYCHO] power -> " + power.label);
      publish();
    }
  }

  /** True once the sequence has been entered. Never goes back to false without a reboot. */
  public static boolean isActive() {
    return active;
  }

  /**
   * True while armed and waiting for AUTO, which is the one point this is reversible.
   *
   * <p>Arming is not the mode. It is a decision to start AUTO in the mode, taken before the match
   * when there is time to change your mind, and backing out is just dropping a confirmation. The
   * moment AUTO begins {@link #engage()} closes the latch and that is the end of reversibility.
   */
  public static boolean isArmed() {
    return armed;
  }

  /** True once AUTO has run in this mode. The dashboard hands the screen to the camera here. */
  public static boolean isAutoDone() {
    return autoDone;
  }

  /**
   * Reads the dashboard's confirmation count and arms or disarms accordingly.
   *
   * <p>Two separate confirmations, not one, and either can be taken back. A single button that
   * throws the robot into a mode it cannot leave is a button somebody eventually leans on.
   *
   * <p>Once the latch is closed this stops listening: confirmations are about getting IN, and
   * there is no corresponding way out.
   */
  public static void pollArming() {
    if (active) {
      return;
    }
    var table = NetworkTableInstance.getDefault()
        .getTable(frc.robot.DashboardTelemetry.TABLE);
    int confirmations = (int) table.getEntry(NT_CONFIRMATIONS).getDouble(0);
    boolean wantArmed = confirmations >= CONFIRMATIONS_REQUIRED;

    if (wantArmed != armed) {
      armed = wantArmed;
      System.out.println("[CYBERPSYCHO] " + (armed
          ? "ARMED. AUTO will start in CYBERPSYCHO."
          : "disarmed. AUTO will run normally."));
    }
    table.getEntry(NT_ARMED).setBoolean(armed);
  }

  /**
   * Closes the latch because AUTO is starting and the robot was armed for it.
   *
   * <p>Separate from the D-pad code on purpose: that is a mid-match decision made with a
   * controller, this is a pre-match one made on the dashboard, and they reach the same
   * irreversible state by different routes.
   */
  public static void engage() {
    if (!armed || active) {
      return;
    }
    latch();
  }

  /** Called when AUTO ends. In this mode it is the cue for the camera to take the screen. */
  public static void autonomousFinished() {
    if (!active) {
      return;
    }
    autoDone = true;
    NetworkTableInstance.getDefault()
        .getTable(frc.robot.DashboardTelemetry.TABLE)
        .getEntry(NT_AUTO_DONE)
        .setBoolean(true);
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
    table.getEntry(NT_ARMED).setBoolean(armed);
    table.getEntry(NT_AUTO_DONE).setBoolean(autoDone);
    table.getEntry("CyberPsycho/Power").setString(power.label);
    table.getEntry("CyberPsycho/PowerScale").setDouble(power.scale);
  }
}
