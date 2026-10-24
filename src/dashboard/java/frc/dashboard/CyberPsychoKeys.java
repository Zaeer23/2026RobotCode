package frc.dashboard;

/**
 * The NetworkTables contract between the robot's CYBERPSYCHO mode and this dashboard.
 *
 * <p>Duplicated here rather than imported from the robot code. The dashboard runs on a laptop
 * with no robot attached and must not drag robot classes and their HAL dependencies onto its
 * classpath to read four string constants.
 *
 * <p>Kept in step by hand with {@code frc.robot.CyberPsycho}. If a key is renamed there and not
 * here the symptom is a dashboard that simply never reacts, so change both.
 */
final class CyberPsychoKeys {

  private CyberPsychoKeys() {}

  /** Dashboard writes 0, 1 or 2. Two arms the robot; anything less backs out. */
  static final String CONFIRMATIONS = "CyberPsycho/Confirmations";

  /** Robot is armed and AUTO will start in the mode. Still reversible. */
  static final String ARMED = "CyberPsycho/Armed";

  /** The latch is closed. Not reversible. */
  static final String ACTIVE = "CyberPsycho/Active";

  /** AUTO has finished in the mode, which is when the camera takes the screen. */
  static final String AUTO_DONE = "CyberPsycho/AutoDone";

  /** Current power level, for display. */
  static final String POWER = "CyberPsycho/Power";

  static final int CONFIRMATIONS_REQUIRED = 2;
}
