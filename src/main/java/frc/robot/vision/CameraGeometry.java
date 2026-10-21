package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation3d;

/** Projects Limelight principal-point angles through the measured camera mounting rotation. */
public final class CameraGeometry {
  private CameraGeometry() {}

  /**
   * An unnormalised sight ray in robot coordinates (+X forward, +Y left, +Z up).
   *
   * <p>Limelight tx/ty are separate image-plane angles, so the camera ray is
   * (1, -tan(tx), tan(ty)). Pitch must be applied before yaw: treating tx as a ground bearing
   * introduces a sideways error whenever a pitched camera observes an off-axis target.
   */
  public static Translation3d robotRelativeRay(
      double txDegrees, double tyDegrees, double pitchDegrees, double yawDegrees) {
    double pitch = Math.toRadians(pitchDegrees);
    double yaw = Math.toRadians(yawDegrees);
    double cameraLeft = -Math.tan(Math.toRadians(txDegrees));
    double cameraUp = Math.tan(Math.toRadians(tyDegrees));
    double forward = Math.cos(pitch) - cameraUp * Math.sin(pitch);
    double up = Math.sin(pitch) + cameraUp * Math.cos(pitch);
    return new Translation3d(
        forward * Math.cos(yaw) - cameraLeft * Math.sin(yaw),
        forward * Math.sin(yaw) + cameraLeft * Math.cos(yaw),
        up);
  }

  /** Ground range from a known target height; NaN for a near-horizontal or impossible ray. */
  public static double groundRangeMeters(
      double txDegrees, double tyDegrees, double pitchDegrees, double heightDifferenceMeters) {
    Translation3d ray = robotRelativeRay(txDegrees, tyDegrees, pitchDegrees, 0.0);
    double horizontal = Math.hypot(ray.getX(), ray.getY());
    double elevation = Math.atan2(ray.getZ(), horizontal);
    double scale = heightDifferenceMeters / ray.getZ();
    if (!Double.isFinite(scale) || scale <= 0.0
        || Math.abs(elevation) <= Math.toRadians(1.0)
        || Math.abs(elevation) >= Math.toRadians(89.0)) {
      return Double.NaN;
    }
    return scale * horizontal;
  }

  /** Pitch that matches a tape-measured ground range while keeping lens height fixed. */
  public static double calibratedPitchDegrees(
      double txDegrees, double tyDegrees, double rangeMeters, double heightDifferenceMeters) {
    double right = Math.tan(Math.toRadians(txDegrees));
    double up = Math.tan(Math.toRadians(tyDegrees));
    double rayLength = Math.sqrt(1.0 + right * right + up * up);
    double requiredUp = heightDifferenceMeters
        / Math.hypot(rangeMeters, heightDifferenceMeters) * rayLength;
    // An asin argument outside [-1, 1] means no mounting pitch can fit this measurement.
    return Math.toDegrees(Math.asin(requiredUp / Math.hypot(1.0, up)) - Math.atan(up));
  }
}
