package frc.robot.controls;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.ControlProfile.Action;
import frc.robot.controls.ControlProfile.Binding;
import frc.robot.controls.ControlProfile.Input;
import frc.robot.controls.ControlProfile.Role;

/**
 * Binds commands to ACTIONS rather than to buttons.
 *
 * <p>A binding is registered once and stays registered. Its trigger asks the active profile, every
 * poll, which physical input the action currently lives on — so switching profiles re-points every
 * binding at once with nothing re-registered and no restart.
 *
 * <p>That live lookup is the whole point. Doing it the obvious way, by registering different
 * bindings per profile, means every binding needs a {@code isProfileX()} guard and adding a
 * profile means touching all of them. It is why there were only ever two profiles and why adding
 * a third was not worth the effort.
 */
public class ControlBinder {

  private final RemappableController driver;
  private final RemappableController operator;
  private final ControlConfig config;

  private final List<ControlProfile> profiles;

  public ControlBinder(
      RemappableController driver,
      RemappableController operator,
      ControlConfig config) {
    this(driver, operator, config, ControlProfile.builtIn());
  }

  public ControlBinder(
      RemappableController driver,
      RemappableController operator,
      ControlConfig config,
      List<ControlProfile> profiles) {
    this.driver = driver;
    this.operator = operator;
    this.config = config;
    this.profiles = profiles;
  }

  /** The profile named by the dashboard, falling back to the first one. */
  public ControlProfile activeProfile() {
    String wanted = config.profile();
    for (ControlProfile profile : profiles) {
      if (profile.name().equalsIgnoreCase(wanted)) {
        return profile;
      }
    }
    return profiles.get(0);
  }

  public List<ControlProfile> profiles() {
    return profiles;
  }

  /**
   * A trigger that is true whenever the action's currently-mapped input is pressed.
   *
   * <p>Resolved on every poll, not captured, which is what makes profile switching live.
   */
  public Trigger trigger(Action action) {
    return new Trigger(() -> isActionPressed(action));
  }

  /** Runs a command while the action is held. */
  public void whileHeld(Action action, Command command) {
    trigger(action).whileTrue(command);
  }

  /** Runs a command once each time the action is pressed. */
  public void onPress(Action action, Command command) {
    trigger(action).onTrue(command);
  }

  private boolean isActionPressed(Action action) {
    Binding binding = activeProfile().get(action);
    if (binding.input() == Input.NONE) {
      return false;
    }

    RemappableController controller =
        binding.role() == Role.DRIVER ? driver : operator;

    // A combo requires its modifier held. A PLAIN binding requires the modifier NOT held,
    // otherwise pressing BACK+X would fire both the combo and the plain X action at once.
    if (binding.isCombo()) {
      if (!isPressed(controller, binding.modifier())) {
        return false;
      }
    } else if (isPressed(controller, ControlProfile.Input.BACK)) {
      // BACK is reserved as the conventional modifier, so plain bindings yield while it is held.
      return false;
    }

    return isPressed(controller, binding.input());
  }

  private boolean isPressed(RemappableController controller, Input input) {
    switch (input) {
      case A: return controller.a().getAsBoolean();
      case B: return controller.b().getAsBoolean();
      case X: return controller.x().getAsBoolean();
      case Y: return controller.y().getAsBoolean();
      case LEFT_BUMPER: return controller.leftBumper().getAsBoolean();
      case RIGHT_BUMPER: return controller.rightBumper().getAsBoolean();
      case LEFT_TRIGGER: return controller.getLeftTriggerAxis()
          > RemappableController.TRIGGER_THRESHOLD;
      case RIGHT_TRIGGER: return controller.getRightTriggerAxis()
          > RemappableController.TRIGGER_THRESHOLD;
      case BACK: return controller.back().getAsBoolean();
      case START: return controller.start().getAsBoolean();
      case LEFT_STICK: return controller.leftStick().getAsBoolean();
      case RIGHT_STICK: return controller.rightStick().getAsBoolean();
      case POV_UP: return controller.pov() == 0;
      case POV_RIGHT: return controller.pov() == 90;
      case POV_DOWN: return controller.pov() == 180;
      case POV_LEFT: return controller.pov() == 270;
      case NONE:
      default:
        return false;
    }
  }

  /** Publishes the profile list and the active map so the dashboard can show and switch them. */
  public void publish(edu.wpi.first.networktables.NetworkTable table) {
    // Pick up any rebind the dashboard wrote, then clear it so it applies exactly once.
    String request = table.getEntry("RebindRequest").getString("");
    if (!request.isBlank()) {
      applyRebindRequest(request);
      table.getEntry("RebindRequest").setString("");
    }

    table.getEntry("InputOptions").setStringArray(
        java.util.Arrays.stream(Input.values()).map(Enum::name).toArray(String[]::new));
    table.getEntry("ActionOptions").setStringArray(
        java.util.Arrays.stream(Action.values()).map(Enum::name).toArray(String[]::new));

    String[] names = profiles.stream().map(ControlProfile::name).toArray(String[]::new);
    table.getEntry("ProfileOptions").setStringArray(names);

    ControlProfile active = activeProfile();
    table.getEntry("ActiveProfile").setString(active.name());

    StringBuilder map = new StringBuilder();
    active.all().forEach((action, binding) -> {
      if (binding.input() == Input.NONE) {
        return;
      }
      map.append(action).append('=').append(binding.describe()).append('\n');
    });
    table.getEntry("ActiveMap").setString(map.toString());
  }

  /**
   * Applies a rebind requested by the dashboard.
   *
   * <p>The dashboard writes a single line, {@code ACTION=ROLE MODIFIER+INPUT}, and this parses it
   * into the active profile. Editing live rather than requiring a redeploy is the whole point: a
   * control change you cannot try until the next build is a control change nobody makes.
   *
   * <p>Anything unparseable is ignored rather than throwing. A malformed string from a dashboard
   * must never be able to take the robot down mid-match.
   */
  public void applyRebindRequest(String request) {
    if (request == null || request.isBlank()) {
      return;
    }
    try {
      String[] halves = request.split("=", 2);
      if (halves.length != 2) {
        return;
      }
      Action action = Action.valueOf(halves[0].trim().toUpperCase());
      String[] parts = halves[1].trim().split("\\s+");
      if (parts.length < 2) {
        return;
      }

      Role role = Role.valueOf(parts[0].trim().toUpperCase());
      String inputSpec = parts[1].trim().toUpperCase();

      Binding binding;
      if (inputSpec.contains("+")) {
        String[] combo = inputSpec.split(java.util.regex.Pattern.quote("+"), 2);
        binding = Binding.combo(
            role,
            Input.valueOf(combo[1].trim()),
            Input.valueOf(combo[0].trim()));
      } else {
        binding = Binding.of(role, Input.valueOf(inputSpec));
      }

      activeProfile().rebind(action, binding);
      System.out.println("[CONTROLS] Rebound " + action + " -> " + binding.describe());
    } catch (IllegalArgumentException e) {
      System.out.println("[CONTROLS] Ignored malformed rebind: " + request);
    }
  }
}
