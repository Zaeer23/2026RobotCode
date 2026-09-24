package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** One AprilTag as the camera currently sees it. Immutable snapshot of a single frame. */
public class TagObservation {

  /** Fiducial ID reported by the camera. */
  public final int id;

  /** Horizontal angle from the camera crosshair to the tag, degrees. POSITIVE means RIGHT. */
  public final double txDegrees;

  /** Vertical angle from the camera crosshair to the tag, degrees. POSITIVE means UP. */
  public final double tyDegrees;

  /** Fraction of the image the tag fills, percent. Useful as a crude closeness signal. */
  public final double areaPercent;

  /** Straight-line range from the camera lens to the tag, meters, from the on-board 3D solve. */
  public final double distanceToCameraMeters;

  /** Straight-line range from the robot origin to the tag, meters, from the on-board 3D solve. */
  public final double distanceToRobotMeters;

  /** Single-tag pose ambiguity. 0 is a confident solve; above ~0.3 is not trustworthy. */
  public final double ambiguity;

  /** Which structure this tag is bolted to. */
  public final TagFunction function;

  private TagObservation(
      int id,
      double txDegrees,
      double tyDegrees,
      double areaPercent,
      double distanceToCameraMeters,
      double distanceToRobotMeters,
      double ambiguity) {
    this.id = id;
    this.txDegrees = txDegrees;
    this.tyDegrees = tyDegrees;
    this.areaPercent = areaPercent;
    this.distanceToCameraMeters = distanceToCameraMeters;
    this.distanceToRobotMeters = distanceToRobotMeters;
    this.ambiguity = ambiguity;
    this.function = FieldTags.functionOf(id);
  }

  static TagObservation of(
      int id,
      double txDegrees,
      double tyDegrees,
      double areaPercent,
      double distanceToCameraMeters,
      double distanceToRobotMeters,
      double ambiguity) {
    return new TagObservation(
        id, txDegrees, tyDegrees, areaPercent,
        distanceToCameraMeters, distanceToRobotMeters, ambiguity);
  }

  /** Which alliance owns this tag, or empty if it is not a 2026 field tag. */
  public Optional<Alliance> alliance() {
    return FieldTags.allianceOf(id);
  }

  /** Known field pose of this tag from the official layout. */
  public Optional<Pose3d> fieldPose() {
    return FieldTags.poseOf(id);
  }

  /** Height of this tag's centre above the carpet, meters. */
  public double heightMeters() {
    return function.heightMeters();
  }

  /** True when this is a real 2026 field tag rather than a stray or misread ID. */
  public boolean isFieldTag() {
    return function != TagFunction.UNKNOWN;
  }

  @Override
  public String toString() {
    return String.format(
        "%s tx=%.2f ty=%.2f area=%.2f%% dist=%.2fm",
        FieldTags.describe(id), txDegrees, tyDegrees, areaPercent, distanceToCameraMeters);
  }
}
