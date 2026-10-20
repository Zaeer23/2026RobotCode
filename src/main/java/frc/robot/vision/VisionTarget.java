package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.LimeLightConstants;

/**
 * One AprilTag, fully validated.
 *
 * <p>An instance of this class existing is a guarantee: every field is finite, the range is
 * positive and physically plausible, and the ID is a real 2026 field tag. Nothing downstream needs
 * to re-check any of that, and nothing downstream can be handed a NaN.
 *
 * <p>That guarantee is the point. The previous version passed raw camera numbers straight through,
 * so a single bad frame could put a NaN into the aim solve, through the clamps (comparisons
 * against NaN are all false, so every guard silently passes it along) and into
 * {@code spark.set()} — which moves a motor in whatever way the firmware feels like.
 */
public final class VisionTarget {

  /** Fiducial ID. Always a real 2026 field tag. */
  public final int id;

  /** Horizontal angle from the camera to the tag, degrees. POSITIVE means the tag is RIGHT. */
  public final double txDegrees;

  /** Vertical angle from the camera to the tag, degrees. POSITIVE means the tag is UP. */
  public final double tyDegrees;

  /** Fraction of the image the tag fills, percent. */
  public final double areaPercent;

  /** Horizontal ground range from the lens to the tag, meters. Always positive. */
  public final double rangeMeters;

  /** How the range was obtained, for telemetry. */
  public final String rangeSource;

  /** What the tag is bolted to. */
  public final TagFunction function;

  VisionTarget(
      int id,
      double txDegrees,
      double tyDegrees,
      double areaPercent,
      double rangeMeters,
      String rangeSource) {
    this.id = id;
    this.txDegrees = txDegrees;
    this.tyDegrees = tyDegrees;
    this.areaPercent = areaPercent;
    this.rangeMeters = rangeMeters;
    this.rangeSource = rangeSource;
    this.function = FieldTags.functionOf(id);
  }

  // ---- Robot-frame geometry -------------------------------------------------
  // Resolved once, here, so the turret and the drivebase can never disagree about where the tag
  // is. The Limelight's tx is POSITIVE to the RIGHT while the robot frame is CCW-positive, so the
  // sign flips exactly once -- in robotFrameY()/robotFrameX() below and nowhere else.
  //
  // Robot frame: +X forward, +Y left, origin at robot centre.

  /** Bearing from the LENS to the tag, CCW-positive degrees. */
  private double lensBearingRadians() {
    return Math.toRadians(-txDegrees + LimeLightConstants.MOUNT_YAW_DEGREES);
  }

  /** Tag position forward of robot centre, meters. */
  public double robotFrameX() {
    return LimeLightConstants.MOUNT_FORWARD_METERS
        + rangeMeters * Math.cos(lensBearingRadians());
  }

  /** Tag position left of robot centre, meters. */
  public double robotFrameY() {
    return LimeLightConstants.MOUNT_LEFT_METERS
        + rangeMeters * Math.sin(lensBearingRadians());
  }

  /**
   * Bearing from ROBOT CENTRE to the tag, CCW-positive degrees. Zero means dead ahead.
   *
   * <p>Not the same as -tx. The lens sits 8.5 inches right of centre and never moves, so a tag
   * centred in the camera is genuinely off to the robot's right -- by about 6 degrees at 2 m and
   * over 12 at 1 m. Driving tx to zero would square the CAMERA up on the tag, not the robot.
   */
  public double bearingFromRobotCenterDegrees() {
    return Math.toDegrees(Math.atan2(robotFrameY(), robotFrameX()));
  }

  /** Ground range from robot centre to the tag, meters. */
  public double rangeFromRobotCenterMeters() {
    return Math.hypot(robotFrameX(), robotFrameY());
  }

  /** Which alliance owns this tag. */
  public Optional<Alliance> alliance() {
    return FieldTags.allianceOf(id);
  }

  /** Height of this tag's centre above the carpet, meters. */
  public double heightMeters() {
    return function.heightMeters();
  }

  @Override
  public String toString() {
    return String.format(
        "%s tx=%.2f ty=%.2f %.2fm(%s)",
        FieldTags.describe(id), txDegrees, tyDegrees, rangeMeters, rangeSource);
  }
}
