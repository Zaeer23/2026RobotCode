package frc.robot.controls;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

/**
 * Which physical USB port each logical controller reads from, and which button each action uses.
 *
 * <p>Everything here lives on NetworkTables so the dashboard can change it live, and in
 * {@link Preferences} so it survives a reboot. Nothing in robot code refers to a USB port number
 * directly; code asks for the DRIVER or the OPERATOR and this decides where that currently comes
 * from.
 *
 * <h2>Why ports are indirect</h2>
 *
 * <p>A {@code CommandXboxController} is built around a fixed port, so swapping which physical
 * controller drives meant unplugging USB and hoping the driver station renumbered them the way you
 * wanted. Worse, a controller that is simply not plugged in makes every binding on it silently do
 * nothing — the buttons are not broken, they are never polled, which is indistinguishable from a
 * software bug from the driver seat. That cost a debugging session here already. Ports are now
 * data, swappable from the dashboard mid-practice, and reported when they change.
 */
public class ControlConfig {

  /** NT table the dashboard reads and writes. */
  public static final String TABLE = "Controls";

  private static final String KEY_DRIVER_PORT = "DriverPort";
  private static final String KEY_OPERATOR_PORT = "OperatorPort";
  private static final String KEY_PROFILE = "Profile";

  private static final int DEFAULT_DRIVER_PORT = 0;
  private static final int DEFAULT_OPERATOR_PORT = 1;

  private final NetworkTable table;

  private int driverPort;
  private int operatorPort;
  private String profile;

  private int lastReportedDriverPort = -1;
  private int lastReportedOperatorPort = -1;

  public ControlConfig() {
    table = NetworkTableInstance.getDefault().getTable(TABLE);

    driverPort = (int) Preferences.getDouble(pref(KEY_DRIVER_PORT), DEFAULT_DRIVER_PORT);
    operatorPort = (int) Preferences.getDouble(pref(KEY_OPERATOR_PORT), DEFAULT_OPERATOR_PORT);
    profile = Preferences.getString(pref(KEY_PROFILE), "Default");

    table.getEntry(KEY_DRIVER_PORT).setDouble(driverPort);
    table.getEntry(KEY_OPERATOR_PORT).setDouble(operatorPort);
    table.getEntry(KEY_PROFILE).setString(profile);
  }

  /** Re-reads the dashboard's values. Call once per loop. */
  public void poll() {
    int newDriver = clampPort((int) table.getEntry(KEY_DRIVER_PORT).getDouble(driverPort));
    int newOperator = clampPort((int) table.getEntry(KEY_OPERATOR_PORT).getDouble(operatorPort));
    String newProfile = table.getEntry(KEY_PROFILE).getString(profile);

    // Two logical controllers on one port means one of them is reading someone else's sticks.
    // Refusing the change is safer than silently handing the drivebase to the operator.
    if (newDriver == newOperator) {
      DriverStation.reportWarning(
          String.format(
              "[CONTROLS] Refused port change: driver and operator both set to %d. Keeping %d/%d.",
              newDriver, driverPort, operatorPort),
          false);
      table.getEntry(KEY_DRIVER_PORT).setDouble(driverPort);
      table.getEntry(KEY_OPERATOR_PORT).setDouble(operatorPort);
      return;
    }

    if (newDriver != driverPort || newOperator != operatorPort) {
      driverPort = newDriver;
      operatorPort = newOperator;
      Preferences.setDouble(pref(KEY_DRIVER_PORT), driverPort);
      Preferences.setDouble(pref(KEY_OPERATOR_PORT), operatorPort);
      System.out.printf("[CONTROLS] Ports -> driver=%d operator=%d%n", driverPort, operatorPort);
    }

    if (!newProfile.equals(profile)) {
      profile = newProfile;
      Preferences.setString(pref(KEY_PROFILE), profile);
      System.out.println("[CONTROLS] Profile -> " + profile);
    }

    publishConnectionState();
  }

  /**
   * Publishes which ports are actually populated.
   *
   * <p>The dashboard shows this so an unplugged controller is visible instead of presenting as
   * dead buttons.
   */
  private void publishConnectionState() {
    boolean driverUp = DriverStation.isJoystickConnected(driverPort);
    boolean operatorUp = DriverStation.isJoystickConnected(operatorPort);

    table.getEntry("DriverConnected").setBoolean(driverUp);
    table.getEntry("OperatorConnected").setBoolean(operatorUp);

    StringBuilder present = new StringBuilder();
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      if (DriverStation.isJoystickConnected(port)) {
        present.append(present.length() == 0 ? "" : ", ")
            .append(port).append(':').append(DriverStation.getJoystickName(port));
      }
    }
    table.getEntry("PortsPresent").setString(present.length() == 0 ? "none" : present.toString());

    if (driverPort != lastReportedDriverPort || operatorPort != lastReportedOperatorPort) {
      lastReportedDriverPort = driverPort;
      lastReportedOperatorPort = operatorPort;
      if (!driverUp) {
        DriverStation.reportWarning(
            "[CONTROLS] Driver port " + driverPort + " has no controller. Its bindings are inert.",
            false);
      }
      if (!operatorUp) {
        DriverStation.reportWarning(
            "[CONTROLS] Operator port " + operatorPort
                + " has no controller. Its bindings are inert.",
            false);
      }
    }
  }

  public int driverPort() {
    return driverPort;
  }

  public int operatorPort() {
    return operatorPort;
  }

  public String profile() {
    return profile;
  }

  /** Swaps the two logical controllers. No USB replugging needed. */
  public void swapControllers() {
    int oldDriver = driverPort;
    driverPort = operatorPort;
    operatorPort = oldDriver;
    table.getEntry(KEY_DRIVER_PORT).setDouble(driverPort);
    table.getEntry(KEY_OPERATOR_PORT).setDouble(operatorPort);
    Preferences.setDouble(pref(KEY_DRIVER_PORT), driverPort);
    Preferences.setDouble(pref(KEY_OPERATOR_PORT), operatorPort);
    System.out.printf("[CONTROLS] Swapped -> driver=%d operator=%d%n", driverPort, operatorPort);
  }

  private static int clampPort(int port) {
    return Math.max(0, Math.min(DriverStation.kJoystickPorts - 1, port));
  }

  private static String pref(String key) {
    return TABLE + "." + key;
  }
}
