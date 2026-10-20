package frc.robot.controls;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An Xbox controller whose USB port can change at runtime.
 *
 * <p>{@code CommandXboxController} binds to a fixed port forever. This reads the port from a
 * supplier on every single poll, so the dashboard can reassign which physical controller is the
 * driver without unplugging anything and without rebooting the robot. Triggers created from this
 * keep working across the swap, because they ask for the port each time rather than capturing it.
 *
 * <p>Axis and button numbers follow the standard Xbox mapping used by the driver station.
 */
public class RemappableController {

  // Xbox axis indices as the driver station reports them.
  private static final int AXIS_LEFT_X = 0;
  private static final int AXIS_LEFT_Y = 1;
  private static final int AXIS_LEFT_TRIGGER = 2;
  private static final int AXIS_RIGHT_TRIGGER = 3;
  private static final int AXIS_RIGHT_X = 4;
  private static final int AXIS_RIGHT_Y = 5;

  // Xbox button indices, 1-based as the driver station reports them.
  private static final int BTN_A = 1;
  private static final int BTN_B = 2;
  private static final int BTN_X = 3;
  private static final int BTN_Y = 4;
  private static final int BTN_LEFT_BUMPER = 5;
  private static final int BTN_RIGHT_BUMPER = 6;
  private static final int BTN_BACK = 7;
  private static final int BTN_START = 8;
  private static final int BTN_LEFT_STICK = 9;
  private static final int BTN_RIGHT_STICK = 10;

  /** Trigger pull past this counts as pressed, for R2/L2 used as buttons. */
  public static final double TRIGGER_THRESHOLD = 0.25;

  private final String label;
  private final IntSupplier portSupplier;

  /**
   * @param label        name used in diagnostics, e.g. "driver"
   * @param portSupplier where the port comes from, read fresh on every access
   */
  public RemappableController(String label, IntSupplier portSupplier) {
    this.label = label;
    this.portSupplier = portSupplier;
  }

  public int port() {
    return portSupplier.getAsInt();
  }

  public String label() {
    return label;
  }

  /** True when something is actually plugged into this controller's current port. */
  public boolean isConnected() {
    return DriverStation.isJoystickConnected(port());
  }

  // ---- Axes -----------------------------------------------------------------
  // All return 0 when nothing is plugged in, so an absent controller reads as "not moving"
  // rather than throwing or returning stale values.

  private double axis(int index) {
    int p = port();
    if (!DriverStation.isJoystickConnected(p)) {
      return 0.0;
    }
    double value = DriverStation.getStickAxis(p, index);
    return Double.isFinite(value) ? value : 0.0;
  }

  public double getLeftX() {
    return axis(AXIS_LEFT_X);
  }

  public double getLeftY() {
    return axis(AXIS_LEFT_Y);
  }

  public double getRightX() {
    return axis(AXIS_RIGHT_X);
  }

  public double getRightY() {
    return axis(AXIS_RIGHT_Y);
  }

  /** L2, 0 to 1. */
  public double getLeftTriggerAxis() {
    return axis(AXIS_LEFT_TRIGGER);
  }

  /** R2, 0 to 1. */
  public double getRightTriggerAxis() {
    return axis(AXIS_RIGHT_TRIGGER);
  }

  // ---- Buttons --------------------------------------------------------------

  private boolean rawButton(int index) {
    int p = port();
    return DriverStation.isJoystickConnected(p) && DriverStation.getStickButton(p, index);
  }

  /** POV/D-pad angle in degrees, or -1 when not pressed. */
  public int pov() {
    int p = port();
    return DriverStation.isJoystickConnected(p) ? DriverStation.getStickPOV(p, 0) : -1;
  }

  /** Raw axis by index, for anything not covered by the named accessors. */
  public double getRawAxis(int index) {
    return axis(index);
  }

  /** Raw button by 1-based index, for anything not covered by the named accessors. */
  public Trigger button(int index) {
    return new Trigger(() -> rawButton(index));
  }

  public Trigger a() {
    return new Trigger(() -> rawButton(BTN_A));
  }

  public Trigger b() {
    return new Trigger(() -> rawButton(BTN_B));
  }

  public Trigger x() {
    return new Trigger(() -> rawButton(BTN_X));
  }

  public Trigger y() {
    return new Trigger(() -> rawButton(BTN_Y));
  }

  public Trigger leftBumper() {
    return new Trigger(() -> rawButton(BTN_LEFT_BUMPER));
  }

  public Trigger rightBumper() {
    return new Trigger(() -> rawButton(BTN_RIGHT_BUMPER));
  }

  public Trigger back() {
    return new Trigger(() -> rawButton(BTN_BACK));
  }

  public Trigger start() {
    return new Trigger(() -> rawButton(BTN_START));
  }

  public Trigger leftStick() {
    return new Trigger(() -> rawButton(BTN_LEFT_STICK));
  }

  public Trigger rightStick() {
    return new Trigger(() -> rawButton(BTN_RIGHT_STICK));
  }

  /** L2 treated as a button. */
  public Trigger leftTrigger() {
    return new Trigger(() -> getLeftTriggerAxis() > TRIGGER_THRESHOLD);
  }

  /** R2 treated as a button. */
  public Trigger rightTrigger() {
    return new Trigger(() -> getRightTriggerAxis() > TRIGGER_THRESHOLD);
  }

  public Trigger povUp() {
    return new Trigger(() -> pov() == 0);
  }

  public Trigger povRight() {
    return new Trigger(() -> pov() == 90);
  }

  public Trigger povDown() {
    return new Trigger(() -> pov() == 180);
  }

  public Trigger povLeft() {
    return new Trigger(() -> pov() == 270);
  }

  /** Rumble, ignored when nothing is plugged in. */
  public void setRumble(double value) {
    int p = port();
    if (!DriverStation.isJoystickConnected(p)) {
      return;
    }
    new edu.wpi.first.wpilibj.GenericHID(p)
        .setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, value);
  }
}
