package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import frc.robot.controls.ControlConfig;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.vision.FieldTags;
import frc.robot.vision.Limelight;

/**
 * Publishes one flat, stable NetworkTables namespace for the driver station dashboard.
 *
 * <p>Deliberately separate from the AdvantageKit logging and from Shuffleboard. Those exist for
 * after-the-fact analysis and for debugging; this is the short list of things someone actually
 * looks at mid-match, under a {@code Dashboard/} prefix that the GUI can rely on not moving.
 *
 * <p>Keep it short. A match dashboard with forty numbers on it is a dashboard nobody reads.
 */
public class DashboardTelemetry {

  public static final String TABLE = "Dashboard";

  private final NetworkTable table;
  private final Superstructure superstructure;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final SwerveSubsystem drivebase;
  private final Limelight camera;
  private final ControlConfig controls;

  public DashboardTelemetry(
      Superstructure superstructure,
      ShooterSubsystem shooter,
      TurretSubsystem turret,
      SwerveSubsystem drivebase,
      Limelight camera,
      ControlConfig controls) {
    this.superstructure = superstructure;
    this.shooter = shooter;
    this.turret = turret;
    this.drivebase = drivebase;
    this.camera = camera;
    this.controls = controls;
    this.table = NetworkTableInstance.getDefault().getTable(TABLE);
  }

  /** Published once at startup: the field size and every tag's pose, so the GUI can draw them. */
  public void publishFieldLayout() {
    table.getEntry("Field/LengthMeters").setDouble(FieldTags.FIELD_LENGTH_METERS);
    table.getEntry("Field/WidthMeters").setDouble(FieldTags.FIELD_WIDTH_METERS);

    // Flat [id, x, y, z, yawDegrees] per tag. A flat array rather than nested structures because
    // it survives any NT client without a schema.
    double[] tags = new double[32 * 5];
    int i = 0;
    for (int id = 1; id <= 32; id++) {
      var pose = FieldTags.poseOf(id);
      if (pose.isEmpty()) {
        continue;
      }
      tags[i++] = id;
      tags[i++] = pose.get().getX();
      tags[i++] = pose.get().getY();
      tags[i++] = pose.get().getZ();
      tags[i++] = pose.get().getRotation().getZ() * 180.0 / Math.PI;
    }
    table.getEntry("Field/Tags").setDoubleArray(java.util.Arrays.copyOf(tags, i));
  }

  /**
   * Robot field pose derived from a seen tag, independent of odometry.
   *
   * <p>A tag's field pose is known exactly, and the camera gives where that tag sits relative to
   * the robot. Combined with the gyro heading that fixes the robot on the field — no odometry, no
   * accumulated drift, valid the instant a tag comes into view. Drawing it next to the tag layout
   * is what makes "are we about to lose sight of the hub" visible before it happens.
   *
   * <p>Only the heading comes from the gyro. If that is wrong the estimate rotates about the tag,
   * so the dashboard shows the heading source alongside it.
   */
  private void publishVisionFieldPose() {
    var target = camera.getTarget();
    if (target.isEmpty()) {
      table.getEntry("Vision/PoseValid").setBoolean(false);
      return;
    }

    var tag = target.get();
    var tagPose = FieldTags.poseOf(tag.id);
    if (tagPose.isEmpty()) {
      table.getEntry("Vision/PoseValid").setBoolean(false);
      return;
    }

    // Tag position in the robot frame, rotated into field axes by the robot's heading, then
    // subtracted from the tag's known field position to land on the robot.
    double headingRadians = drivebase.getHeading().getRadians();
    double cos = Math.cos(headingRadians);
    double sin = Math.sin(headingRadians);

    double tagFieldOffsetX = tag.robotFrameX() * cos - tag.robotFrameY() * sin;
    double tagFieldOffsetY = tag.robotFrameX() * sin + tag.robotFrameY() * cos;

    table.getEntry("Vision/EstimatedX").setDouble(tagPose.get().getX() - tagFieldOffsetX);
    table.getEntry("Vision/EstimatedY").setDouble(tagPose.get().getY() - tagFieldOffsetY);
    table.getEntry("Vision/EstimatedHeadingDegrees").setDouble(Math.toDegrees(headingRadians));
    table.getEntry("Vision/PoseValid").setBoolean(true);

    // Every visible tag, so the GUI can highlight which ones are in view right now.
    var visible = camera.getVisibleTargets();
    double[] ids = new double[visible.size()];
    for (int i = 0; i < visible.size(); i++) {
      ids[i] = visible.get(i).id;
    }
    table.getEntry("Vision/VisibleTagIds").setDoubleArray(ids);
  }

  private final frc.robot.util.Throttle publishRate =
      new frc.robot.util.Throttle(frc.robot.util.Throttle.DASHBOARD_HZ);

  /** Call once per robot loop. */
  public void update() {
    // Gated to 10 Hz. At 50 Hz this table alone was 1,850 NT updates a second, and the radio
    // cannot drain that; the queue backs up and every value arrives later than the last.
    if (!publishRate.ready()) {
      return;
    }

    // ---- Match state ----
    table.getEntry("MatchTime").setDouble(DriverStation.getMatchTime());
    table.getEntry("Enabled").setBoolean(DriverStation.isEnabled());
    table.getEntry("Autonomous").setBoolean(DriverStation.isAutonomous());
    table.getEntry("Mode").setString(
        DriverStation.isTest() ? "TEST" : DriverStation.isAutonomous() ? "AUTO" : "TELEOP");
    table.getEntry("FMSAttached").setBoolean(DriverStation.isFMSAttached());
    table.getEntry("Alliance").setString(
        DriverStation.getAlliance().map(Enum::name).orElse("UNKNOWN"));
    table.getEntry("BatteryVolts").setDouble(RobotController.getBatteryVoltage());
    table.getEntry("Brownout").setBoolean(RobotController.isBrownedOut());

    // ---- Vision ----
    table.getEntry("Vision/Status").setString(camera.getStatus());
    table.getEntry("Vision/Connected").setBoolean(camera.isConnected());
    table.getEntry("Vision/HasTarget").setBoolean(camera.hasTarget());
    table.getEntry("Vision/VisibleTags").setString(camera.describeVisible());
    table.getEntry("Vision/TagId").setDouble(
        camera.getTarget().map(t -> (double) t.id).orElse(-1.0));
    table.getEntry("Vision/RangeFeet").setDouble(
        camera.getTarget().map(t -> t.rangeMeters * 3.28084).orElse(0.0));
    table.getEntry("Vision/Filter").setString(camera.getTargetFilter().toString());
    table.getEntry("Vision/StreamUrl").setString(Limelight.streamUrlHint());

    publishVisionFieldPose();

    // ---- Shooter ----
    double shotDistanceMeters = superstructure.getHubDistanceMeters();
    table.getEntry("Shooter/DistanceFeet").setDouble(
        shotDistanceMeters > 0 ? shotDistanceMeters * 3.28084 : 0.0);
    table.getEntry("Shooter/CommandedRPM").setDouble(shooter.getCommandedShotRpm());
    table.getEntry("Shooter/ActualRPM").setDouble(shooter.getSpeed().in(RPM));
    table.getEntry("Shooter/AtSpeed").setBoolean(shooter.isAtShotSpeed());

    // ---- Turret ----
    table.getEntry("Turret/AngleDegrees").setDouble(turret.getRawAngle().in(Degrees));
    table.getEntry("Turret/OnTarget").setBoolean(turret.isOnTarget());
    table.getEntry("Turret/ErrorDegrees").setDouble(
        Double.isFinite(turret.getTrackingErrorDegrees())
            ? turret.getTrackingErrorDegrees()
            : 0.0);

    // ---- Readiness: the single light worth watching mid-match ----
    table.getEntry("ReadyToShoot").setBoolean(
        camera.hasTarget() && turret.isOnTarget() && shooter.isAtShotSpeed());

    // ---- Drivebase ----
    var pose = drivebase.getPose();
    table.getEntry("Drive/X").setDouble(pose.getX());
    table.getEntry("Drive/Y").setDouble(pose.getY());
    table.getEntry("Drive/HeadingDegrees").setDouble(pose.getRotation().getDegrees());

    // ---- Controls ----
    table.getEntry("Controls/DriverPort").setDouble(controls.driverPort());
    table.getEntry("Controls/OperatorPort").setDouble(controls.operatorPort());
  }
}
