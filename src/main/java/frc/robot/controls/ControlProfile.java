package frc.robot.controls;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A named set of button assignments.
 *
 * <p>Actions are referred to by name, and a profile says which physical button each name lives on.
 * Bindings are registered once against the ACTION, so switching profiles re-points every binding
 * at once with nothing to re-register and no robot restart.
 *
 * <p>The previous arrangement hard-coded buttons into every binding and gated them behind
 * {@code isCurrentShooterProfile()} / {@code isAltShooterProfile()} checks. Adding a profile meant
 * editing every binding in RobotContainer and adding another branch to every predicate, so in
 * practice nobody added one. Profiles are data now.
 */
public class ControlProfile {

  /** Everything the drivers can be asked to do. Add here, not in RobotContainer. */
  public enum Action {
    // Driver
    LINE_UP_ON_TAG,
    ZERO_GYRO,
    LOCK_WHEELS,
    CLIMB_ALIGN,
    TURRET_TEST_RAW,
    TURRET_TEST_LEFT,
    TURRET_TEST_RIGHT,
    TURRET_REZERO,
    CAMERA_DUMP,
    CAMERA_CALIBRATE,
    CAMERA_TOGGLE_FILTER,

    // Operator
    SHOOT,
    SPIN_UP_ONLY,
    FEED,
    INTAKE,
    HOPPER_BACKFEED
  }

  /** Physical inputs a profile can map an action onto. */
  public enum Input {
    A, B, X, Y,
    LEFT_BUMPER, RIGHT_BUMPER,
    LEFT_TRIGGER, RIGHT_TRIGGER,
    BACK, START,
    LEFT_STICK, RIGHT_STICK,
    POV_UP, POV_DOWN, POV_LEFT, POV_RIGHT,
    /** Explicitly unbound. Keeps an action visible in the map without it doing anything. */
    NONE
  }

  /** Which controller an action is read from. */
  public enum Role {
    DRIVER,
    OPERATOR
  }

  /**
   * One assignment: an action, on a role's input, optionally combined with a second input.
   *
   * @param role      which controller it is read from
   * @param input     the primary input
   * @param modifier  a second input that must be held at the same time, or NONE for a plain
   *                  single-button binding
   */
  public record Binding(Role role, Input input, Input modifier) {

    public static Binding of(Role role, Input input) {
      return new Binding(role, input, Input.NONE);
    }

    /**
     * Two buttons that must be held together.
     *
     * <p>Combining inputs buys back buttons on a controller that has run out of them, and keeps
     * destructive or diagnostic actions off a single accidental press.
     */
    public static Binding combo(Role role, Input input, Input modifier) {
      return new Binding(role, input, modifier);
    }

    /** Behind BACK, the conventional modifier for diagnostics. */
    public static Binding modified(Role role, Input input) {
      return new Binding(role, input, Input.BACK);
    }

    public static Binding unbound() {
      return new Binding(Role.DRIVER, Input.NONE, Input.NONE);
    }

    public boolean isCombo() {
      return modifier != Input.NONE;
    }

    /** e.g. {@code "OPERATOR BACK+Y"} or {@code "DRIVER LEFT_TRIGGER"}. */
    public String describe() {
      if (input == Input.NONE) {
        return "unbound";
      }
      return role + " " + (isCombo() ? modifier + "+" + input : input.toString());
    }
  }

  private final String name;
  private final Map<Action, Binding> bindings = new LinkedHashMap<>();

  public ControlProfile(String name) {
    this.name = name;
  }

  public String name() {
    return name;
  }

  public ControlProfile bind(Action action, Role role, Input input) {
    bindings.put(action, Binding.of(role, input));
    return this;
  }

  public ControlProfile bindModified(Action action, Role role, Input input) {
    bindings.put(action, Binding.modified(role, input));
    return this;
  }

  /** Binds an action to two inputs held together. */
  public ControlProfile bindCombo(Action action, Role role, Input input, Input modifier) {
    bindings.put(action, Binding.combo(role, input, modifier));
    return this;
  }

  /** Replaces one binding at runtime, which is how the dashboard editor applies a change. */
  public void rebind(Action action, Binding binding) {
    bindings.put(action, binding);
  }

  /** Unbound actions return an explicit NONE rather than null, so callers need no null checks. */
  public Binding get(Action action) {
    return bindings.getOrDefault(action, Binding.unbound());
  }

  public Map<Action, Binding> all() {
    return java.util.Collections.unmodifiableMap(bindings);
  }

  // ---- Built-in profiles ----------------------------------------------------

  /** The control set the team drives with. R2 shoots, driver L2 lines up. */
  public static ControlProfile standard() {
    return new ControlProfile("Standard")
        .bind(Action.LINE_UP_ON_TAG, Role.DRIVER, Input.LEFT_TRIGGER)
        .bind(Action.ZERO_GYRO, Role.DRIVER, Input.A)
        .bind(Action.LOCK_WHEELS, Role.DRIVER, Input.LEFT_BUMPER)
        .bind(Action.CLIMB_ALIGN, Role.DRIVER, Input.START)
        .bind(Action.TURRET_REZERO, Role.DRIVER, Input.B)
        .bindModified(Action.TURRET_TEST_RAW, Role.OPERATOR, Input.POV_UP)
        .bindModified(Action.TURRET_TEST_LEFT, Role.OPERATOR, Input.POV_LEFT)
        .bindModified(Action.TURRET_TEST_RIGHT, Role.OPERATOR, Input.POV_RIGHT)
        .bindModified(Action.CAMERA_DUMP, Role.OPERATOR, Input.Y)
        .bindModified(Action.CAMERA_CALIBRATE, Role.OPERATOR, Input.B)
        .bindModified(Action.CAMERA_TOGGLE_FILTER, Role.OPERATOR, Input.X)
        .bind(Action.SHOOT, Role.OPERATOR, Input.RIGHT_TRIGGER)
        .bind(Action.FEED, Role.OPERATOR, Input.A)
        .bind(Action.INTAKE, Role.OPERATOR, Input.B)
        .bind(Action.HOPPER_BACKFEED, Role.OPERATOR, Input.X);
  }

  /** Triggers do the feeding; bumpers nudge the turret. */
  public static ControlProfile triggerFeed() {
    return new ControlProfile("Trigger Feed")
        .bind(Action.LINE_UP_ON_TAG, Role.DRIVER, Input.LEFT_TRIGGER)
        .bind(Action.ZERO_GYRO, Role.DRIVER, Input.A)
        .bind(Action.LOCK_WHEELS, Role.DRIVER, Input.LEFT_BUMPER)
        .bind(Action.CLIMB_ALIGN, Role.DRIVER, Input.START)
        .bind(Action.SPIN_UP_ONLY, Role.OPERATOR, Input.RIGHT_TRIGGER)
        .bind(Action.FEED, Role.OPERATOR, Input.LEFT_TRIGGER)
        .bind(Action.INTAKE, Role.OPERATOR, Input.A)
        .bind(Action.HOPPER_BACKFEED, Role.OPERATOR, Input.Y)
        .bindModified(Action.CAMERA_DUMP, Role.OPERATOR, Input.Y);
  }

  /** Everything on one controller, for testing with a single gamepad. */
  public static ControlProfile singleDriver() {
    return new ControlProfile("Single Controller")
        .bind(Action.LINE_UP_ON_TAG, Role.DRIVER, Input.LEFT_TRIGGER)
        .bind(Action.SHOOT, Role.DRIVER, Input.RIGHT_TRIGGER)
        .bind(Action.ZERO_GYRO, Role.DRIVER, Input.A)
        .bind(Action.INTAKE, Role.DRIVER, Input.B)
        .bind(Action.FEED, Role.DRIVER, Input.X)
        .bindModified(Action.CAMERA_DUMP, Role.DRIVER, Input.Y)
        .bindModified(Action.TURRET_TEST_RAW, Role.DRIVER, Input.POV_UP);
  }

  /** All built-in profiles, in dashboard order. */
  public static java.util.List<ControlProfile> builtIn() {
    return java.util.List.of(standard(), triggerFeed(), singleDriver());
  }
}
