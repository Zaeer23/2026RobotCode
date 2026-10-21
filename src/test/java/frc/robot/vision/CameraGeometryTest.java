package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.TurretAimCommand;
import org.junit.jupiter.api.Test;

class CameraGeometryTest {
  private static final double EPSILON = 1e-9;

  @Test
  void reconstructsTagStraightBehindTurretWithPitchedOffsetCamera() {
    // Generate camera angles from a known physical point, independently of the aim solver.
    double x = TurretConstants.PIVOT_TO_ROBOT_CENTER.getX() - 2.0;
    double y = TurretConstants.PIVOT_TO_ROBOT_CENTER.getY();
    VisionTarget target = observe(x, y, FieldTags.heightMeters(10));

    assertEquals(x, target.robotFrameX(), EPSILON);
    assertEquals(y, target.robotFrameY(), EPSILON);
    assertEquals(TurretConstants.ENCODER_OFFSET_DEGREES,
        TurretAimCommand.solveTurretAngle(target, 0.0), EPSILON);
  }

  @Test
  void reconstructsOffAxisTagOnEitherSideAcrossShootingRanges() {
    for (double distance : new double[] {1.0, 2.0, 4.0, 7.0}) {
      for (double lateral : new double[] {-0.8, 0.0, 0.8}) {
        double x = TurretConstants.PIVOT_TO_ROBOT_CENTER.getX() - distance;
        VisionTarget target = observe(x, lateral, FieldTags.heightMeters(10));
        assertEquals(x, target.robotFrameX(), EPSILON);
        assertEquals(lateral, target.robotFrameY(), EPSILON);
        double expected = Math.toDegrees(Math.atan2(-lateral, distance))
            + TurretConstants.ENCODER_OFFSET_DEGREES;
        assertEquals(expected, TurretAimCommand.solveTurretAngle(target, 0.0), EPSILON);
      }
    }
  }

  @Test
  void cameraRightIsRobotLeftWhenCameraFacesRearward() {
    Translation3d ray = CameraGeometry.robotRelativeRay(10.0, 0.0, 20.0, 180.0);
    assertTrue(ray.getX() < 0.0);
    assertTrue(ray.getY() > 0.0);
  }

  @Test
  void centeredTagMatchesTheVerticalPlaneRangeFormula() {
    double range = CameraGeometry.groundRangeMeters(0.0, 5.0, 20.0, 1.0);
    assertEquals(1.0 / Math.tan(Math.toRadians(25.0)), range, EPSILON);
  }

  @Test
  void doesNotInventRangeForHorizontalOrWrongHeightRay() {
    assertTrue(Double.isNaN(CameraGeometry.groundRangeMeters(10.0, -20.0, 20.0, 1.0)));
    assertTrue(Double.isNaN(CameraGeometry.groundRangeMeters(0.0, 5.0, 20.0, -1.0)));
    assertTrue(Double.isNaN(CameraGeometry.groundRangeMeters(Double.NaN, 5.0, 20.0, 1.0)));
  }

  @Test
  void rangeCalibrationAccountsForOffAxisAngle() {
    double range = CameraGeometry.groundRangeMeters(30.0, 5.0, 20.0, 1.0);
    assertEquals(20.0, CameraGeometry.calibratedPitchDegrees(30.0, 5.0, range, 1.0), EPSILON);
  }

  @Test
  void positiveTrimMovesAimCounterclockwiseWithoutChangingTargetPosition() {
    VisionTarget target = observe(TurretConstants.PIVOT_TO_ROBOT_CENTER.getX() - 3.0,
        TurretConstants.PIVOT_TO_ROBOT_CENTER.getY(), FieldTags.heightMeters(10));
    double nominal = TurretAimCommand.solveTurretAngle(target, 0.0);
    assertEquals(nominal + 0.5, TurretAimCommand.solveTurretAngle(target, 0.5), EPSILON);
    assertEquals(nominal - 0.5, TurretAimCommand.solveTurretAngle(target, -0.5), EPSILON);
  }

  private static VisionTarget observe(double robotX, double robotY, double robotZ) {
    double x = robotX - LimeLightConstants.MOUNT_FORWARD_METERS;
    double y = robotY - LimeLightConstants.MOUNT_LEFT_METERS;
    double z = robotZ - LimeLightConstants.MOUNT_HEIGHT_METERS;
    double yaw = Math.toRadians(LimeLightConstants.MOUNT_YAW_DEGREES);
    double pitch = Math.toRadians(LimeLightConstants.MOUNT_PITCH_DEGREES);
    double horizontalForward = x * Math.cos(yaw) + y * Math.sin(yaw);
    double cameraLeft = -x * Math.sin(yaw) + y * Math.cos(yaw);
    double cameraForward = horizontalForward * Math.cos(pitch) + z * Math.sin(pitch);
    double cameraUp = -horizontalForward * Math.sin(pitch) + z * Math.cos(pitch);
    double tx = Math.toDegrees(Math.atan2(-cameraLeft, cameraForward));
    double ty = Math.toDegrees(Math.atan2(cameraUp, cameraForward));
    double range = CameraGeometry.groundRangeMeters(tx, ty,
        LimeLightConstants.MOUNT_PITCH_DEGREES, z);
    return new VisionTarget(10, tx, ty, 0.5, range, "TY");
  }
}
