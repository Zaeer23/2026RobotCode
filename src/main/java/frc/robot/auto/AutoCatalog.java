package frc.robot.auto;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DashboardTelemetry;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Every autonomous routine the drive team can pick, and a picture of what each one does.
 *
 * <p>Owns the auto chooser, so a routine cannot be offered without also getting a preview. The
 * preview is built from the same {@code .auto} and {@code .path} files the routine runs, and
 * published once at boot as JSON under {@code Dashboard/Auto/Catalog}. The dashboard draws it on
 * the field: where the robot starts, where it drives, where it shoots, and roughly when.
 *
 * <p>Building it here from the deployed files is the point. What the driver sees is what is on the
 * robot, not what is on whichever laptop happens to be open. (With no robot connected, the
 * dashboard builds the same previews from the project's own files with {@link
 * AutoPreviewBuilder}, the class that does the work here too.)
 *
 * <p>Paths are published as authored, on the BLUE side. The dashboard mirrors them for red the
 * same way PathPlanner does when it runs them.
 *
 * <p>Nothing in here can stop the robot booting. A routine whose preview fails to build is still
 * in the chooser and still runs; its preview carries the error instead, so the drive team sees it
 * in the pits rather than finding out on the field.
 */
public class AutoCatalog {

  /** SmartDashboard key of the chooser. Shuffleboard and the dashboard both find it here. */
  public static final String CHOOSER_KEY = "Auto Routine";

  /** Key of the preview JSON, in the {@link DashboardTelemetry#TABLE} table. */
  public static final String CATALOG_KEY = "Auto/Catalog";

  private record Entry(String label, Command command, boolean isDefault, String autoFile,
      String inPlaceTitle, double inPlaceSeconds) {
  }

  private final SendableChooser<Command> chooser = new SendableChooser<>();
  private final Function<String, Command> pathPlannerLoader;
  private final List<Entry> entries = new ArrayList<>();
  private final AutoPreviewBuilder previews =
      new AutoPreviewBuilder(message -> DriverStation.reportWarning(message, false));

  /** @param pathPlannerLoader builds the command for a PathPlanner auto, given its file name */
  public AutoCatalog(Function<String, Command> pathPlannerLoader) {
    this.pathPlannerLoader = pathPlannerLoader;
  }

  /**
   * How long a named command is expected to take, for the preview's timeline.
   *
   * <p>Only the ones that hold the routine up need this. Anything unset counts as instant, which
   * is right for the ones that only start a mechanism or run alongside a path.
   */
  public void setStepSeconds(String namedCommand, double seconds) {
    previews.setStepSeconds(namedCommand, seconds);
  }

  /** A PathPlanner routine, from {@code deploy/pathplanner/autos/<autoFile>.auto}. */
  public void addPathPlanner(String label, String autoFile, boolean isDefault) {
    add(new Entry(label, pathPlannerLoader.apply(autoFile), isDefault, autoFile, null, 0.0));
  }

  /** A routine that never drives: it runs one thing from wherever the robot was placed. */
  public void addInPlace(String label, Command command, String title, double seconds) {
    add(new Entry(label, command, false, null, title, seconds));
  }

  /** A routine that does nothing at all. */
  public void addNothing(String label, Command command) {
    add(new Entry(label, command, false, null, null, 0.0));
  }

  private void add(Entry entry) {
    entries.add(entry);
    if (entry.isDefault()) {
      chooser.setDefaultOption(entry.label(), entry.command());
    } else {
      chooser.addOption(entry.label(), entry.command());
    }
  }

  public Command getSelected() {
    return chooser.getSelected();
  }

  /** Puts the chooser on SmartDashboard and publishes every preview. Call once, after adding. */
  public void publish() {
    SmartDashboard.putData(CHOOSER_KEY, chooser);

    List<ObjectNode> routines = new ArrayList<>();
    for (Entry entry : entries) {
      routines.add(entry.autoFile() != null
          ? previews.pathPlanner(entry.label(), entry.autoFile(), entry.isDefault())
          : previews.inPlace(entry.label(), entry.isDefault(), entry.inPlaceTitle(),
              entry.inPlaceSeconds()));
    }

    NetworkTableInstance.getDefault()
        .getTable(DashboardTelemetry.TABLE)
        .getEntry(CATALOG_KEY)
        .setString(previews.catalog(routines));
  }

  /** Flattens one {@code .auto} file into the ordered steps a person would describe it as. */
  ArrayNode stepsOf(String autoFile) throws Exception {
    return previews.stepsOf(autoFile);
  }
}
