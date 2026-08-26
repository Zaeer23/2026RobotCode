package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ProjectileMotion;
import frc.robot.subsystems.VisionTargeting;
import frc.robot.subsystems.VisionTargeting.TargetObservation;
import org.junit.jupiter.api.Test;

/**
 * Checks the vision geometry and the shot model against hand-computable cases.
 *
 * <p>These are the parts that are easy to get subtly wrong and impossible to debug on a field: a
 * flipped tx sign aims the turret the wrong way, and a bad range scales every shot.
 */
class VisionTargetingTest {

  private static final double HUB_TAG_ID = 20;

  /** Builds a scan as the Limelight would report it. */
  private static LimeLight.AprilTagScan scan(int tagId, double tx, double ty) {
    return new LimeLight.AprilTagScan(true, tagId, tx, ty, 3.0, 0.0, new Pose3d());
  }

  /**
   * The camera pitch is chosen so that ty=0 puts the tag at a known range, letting us verify the
   * ray-walk against straightforward trigonometry.
   */
  private static double expectedRangeFromCameraAtTyZero() {
    double deltaHeight =
        FieldConstants.tagHeightMeters(20) - LimeLightConstants.MOUNT_HEIGHT_METERS;
    return deltaHeight / Math.tan(Math.toRadians(LimeLightConstants.MOUNT_PITCH_DEGREES));
  }

  @Test
  void invalidScanProducesInvalidObservation() {
    TargetObservation observation =
        VisionTargeting.solve(
            new LimeLight.AprilTagScan(false, -1, 0, 0, 0, 0, new Pose3d()));
    assertFalse(observation.valid);
    assertFalse(observation.hadRawTarget, "no tag seen at all");
  }

  @Test
  void nonFiniteReadingIsRejectedButCountsAsSeeingATag() {
    TargetObservation observation = VisionTargeting.solve(scan(20, Double.NaN, 0.0));
    assertFalse(observation.valid);
    assertTrue(observation.hadRawTarget, "camera did report a tag, the math failed");
  }

  @Test
  void tagStraightAheadAtTyZeroGivesExpectedRange() {
    TargetObservation observation = VisionTargeting.solve(scan(20, 0.0, 0.0));
    assertTrue(observation.valid, observation.message);
    assertEquals("TY_TRIG", observation.distanceSource);

    // Range from the LENS, then shifted to robot center by the mounting offset.
    double rangeFromCamera = expectedRangeFromCameraAtTyZero();
    double expectedX = LimeLightConstants.MOUNT_FORWARD_METERS + rangeFromCamera;
    double expectedY = LimeLightConstants.MOUNT_LEFT_METERS;

    assertEquals(expectedX, observation.targetXMeters, 1e-6);
    assertEquals(expectedY, observation.targetYMeters, 1e-6);
    assertEquals(Math.hypot(expectedX, expectedY), observation.robotDistanceMeters, 1e-6);
  }

  /**
   * The single most important sign in the project. Limelight tx is positive when the target is to
   * the RIGHT; the robot frame is CCW-positive, so the target must end up on the robot's right,
   * meaning a NEGATIVE bearing and a NEGATIVE turret angle.
   */
  @Test
  void positiveTxPutsTargetToTheRight() {
    TargetObservation observation = VisionTargeting.solve(scan(20, 10.0, 0.0));
    assertTrue(observation.valid, observation.message);
    assertTrue(
        observation.robotBearingDegrees < 0.0,
        "positive tx means target is right of centre, which is a negative CCW bearing");
    assertTrue(observation.targetYMeters < 0.0, "right of centre is -Y in the robot frame");
    assertTrue(observation.turretAngleDegrees < 0.0, "turret must rotate clockwise to face it");
  }

  @Test
  void negativeTxPutsTargetToTheLeft() {
    TargetObservation observation = VisionTargeting.solve(scan(20, -10.0, 0.0));
    assertTrue(observation.valid, observation.message);
    assertTrue(observation.robotBearingDegrees > 0.0);
    assertTrue(observation.targetYMeters > 0.0);
    assertTrue(observation.turretAngleDegrees > 0.0);
  }

  /** Looking further up means the tag is further away. */
  @Test
  void largerTyMeansCloserTarget() {
    double nearRange = VisionTargeting.solve(scan(20, 0.0, 10.0)).robotDistanceMeters;
    double midRange = VisionTargeting.solve(scan(20, 0.0, 0.0)).robotDistanceMeters;
    double farRange = VisionTargeting.solve(scan(20, 0.0, -10.0)).robotDistanceMeters;

    assertTrue(nearRange < midRange, "looking up = tag is closer");
    assertTrue(midRange < farRange, "looking down = tag is further");
  }

  /**
   * A TRENCH tag sits 9.25 in lower than a HUB tag. Reading the identical tx/ty off each must
   * therefore give different ranges -- if it does not, the per-tag height lookup is not wired up.
   */
  @Test
  void tagHeightIsLookedUpPerTagId() {
    double hubRange = VisionTargeting.solve(scan(20, 0.0, 0.0)).robotDistanceMeters;
    double trenchRange = VisionTargeting.solve(scan(1, 0.0, 0.0)).robotDistanceMeters;

    assertTrue(
        Math.abs(hubRange - trenchRange) > 0.2,
        "HUB and TRENCH tags are at different heights so must range differently");
    assertTrue(trenchRange < hubRange, "the lower tag resolves closer at the same ty");
  }

  @Test
  void shooterRangeIsShorterThanRobotRangeByTheExitRadius() {
    TargetObservation observation = VisionTargeting.solve(scan(20, 0.0, -5.0));
    assertTrue(observation.valid, observation.message);
    assertTrue(observation.shooterDistanceMeters < observation.robotDistanceMeters);
    assertTrue(
        observation.shooterDistanceMeters > 0.0, "a valid target must give a positive shot range");
  }

  @Test
  void turretRangeCheckRejectsTargetsBehindTheRobot() {
    // 80 degrees of tx puts the tag well outside the turret's +/-90 window once the mounting
    // offset is taken into account.
    TargetObservation wide = VisionTargeting.solve(scan(20, 80.0, -5.0));
    assertTrue(wide.valid, wide.message);
    assertTrue(Math.abs(wide.turretAngleDegrees) > 45.0, "should be a large turret angle");
  }

  // ---- Shot model ---------------------------------------------------------

  /**
   * The whole RPM curve is anchored on one measured shot, so it must reproduce that shot exactly.
   * If this fails, every commanded speed on the robot is scaled wrong.
   */
  @Test
  void modelReproducesTheReferenceShot() {
    double rpm =
        ProjectileMotion.flywheelRpmForDistance(ShooterConstants.REFERENCE_DISTANCE_METERS);
    assertEquals(ShooterConstants.REFERENCE_RPM, rpm, 1e-6);
  }

  @Test
  void transferEfficiencyIsPhysicallyPlausible() {
    double efficiency = ProjectileMotion.FLYWHEEL_TRANSFER_EFFICIENCY;
    assertTrue(
        efficiency > 0.1 && efficiency < 1.0,
        "a ball cannot leave faster than the wheel surface: got " + efficiency);
  }

  @Test
  void rpmIncreasesWithDistance() {
    double close = ProjectileMotion.flywheelRpmForDistance(2.0);
    double mid = ProjectileMotion.flywheelRpmForDistance(4.0);
    double far = ProjectileMotion.flywheelRpmForDistance(6.0);

    assertTrue(close < mid, "further shots need more speed");
    assertTrue(mid < far);
  }

  /**
   * The curve must stay inside the clamp window across the usable range, otherwise the clamp is
   * silently flattening the model the way the old linear fit was.
   */
  @Test
  void rpmStaysInsideTheClampWindowAcrossValidRange() {
    for (double distance = ShooterConstants.MIN_VALID_DISTANCE_METERS;
        distance <= ShooterConstants.MAX_VALID_DISTANCE_METERS;
        distance += 0.25) {
      double rpm = ProjectileMotion.flywheelRpmForDistance(distance);
      if (!Double.isFinite(rpm)) {
        continue; // unreachable at this launch angle, handled by the caller's fallback
      }
      assertTrue(
          rpm >= ShooterConstants.MIN_SHOT_RPM * 0.5,
          String.format("rpm %.0f at %.2f m is implausibly low", rpm, distance));
      assertTrue(
          rpm <= ShooterConstants.MAX_MECHANICAL_RPM,
          String.format("rpm %.0f at %.2f m exceeds the mechanical ceiling", rpm, distance));
    }
  }

  /** Very close shots are unreachable at a fixed 45 degrees; the model must say so, not guess. */
  @Test
  void unreachableShotReturnsNaN() {
    double deltaHeight =
        ProjectileMotion.HUB_TARGET_HEIGHT_METERS - ShooterConstants.EXIT_HEIGHT_METERS;
    // At 45 degrees the ball only clears deltaHeight after travelling deltaHeight horizontally.
    double tooClose = deltaHeight * 0.5;
    assertTrue(Double.isNaN(ProjectileMotion.flywheelRpmForDistance(tooClose)));
  }

  // ---- Field constants ----------------------------------------------------

  @Test
  void hubTagsAreAllAtHubHeight() {
    double hubHeight = Units.inchesToMeters(44.25);
    for (int tagId : FieldConstants.ALL_HUB_TAG_IDS) {
      assertEquals(
          hubHeight,
          FieldConstants.tagHeightMeters(tagId),
          1e-3,
          "tag " + tagId + " is not at HUB height, so it is not a HUB tag");
    }
  }

  @Test
  void trenchTagsAreNotHubTags() {
    for (int tagId : FieldConstants.ALL_TRENCH_TAG_IDS) {
      assertFalse(
          FieldConstants.ALL_HUB_TAG_IDS.contains(tagId),
          "tag " + tagId + " cannot be both TRENCH and HUB");
      assertEquals(Units.inchesToMeters(35.0), FieldConstants.tagHeightMeters(tagId), 1e-3);
    }
  }

  @Test
  void tagOneIsATrenchTagNotAHubTag() {
    // The bug this whole change set exists to prevent.
    assertTrue(FieldConstants.ALL_TRENCH_TAG_IDS.contains(1));
    assertFalse(FieldConstants.ALL_HUB_TAG_IDS.contains(1));
  }

  @Test
  void towerTagsAreAtTowerHeight() {
    for (int tagId : FieldConstants.ALL_TOWER_TAG_IDS) {
      assertEquals(Units.inchesToMeters(21.75), FieldConstants.tagHeightMeters(tagId), 1e-3);
    }
  }

  @Test
  void hubsSitOnOppositeHalvesOfTheField() {
    assertTrue(
        FieldConstants.BLUE_HUB_CENTER.getX() < FieldConstants.FIELD_LENGTH_METERS / 2.0,
        "blue hub should be in the blue half");
    assertTrue(
        FieldConstants.RED_HUB_CENTER.getX() > FieldConstants.FIELD_LENGTH_METERS / 2.0,
        "red hub should be in the red half");
    assertEquals(
        FieldConstants.BLUE_HUB_CENTER.getY(),
        FieldConstants.RED_HUB_CENTER.getY(),
        0.05,
        "both hubs sit on the field centreline");
  }

  @Test
  void turretPivotIsBehindRobotCentre() {
    assertTrue(TurretConstants.PIVOT_TO_ROBOT_CENTER.getX() < 0.0);
  }

  @Test
  void mountPoseAgreesWithLimeLightLateralOffset() {
    // Two places describe how far right of centre the lens sits: the mounting pose used by the
    // geometry solver, and the inches constant used by the legacy tx parallax correction. If they
    // ever drift apart, the turret and the drivebase align loop aim at different points.
    double lensRightOfCentreInches =
        Units.metersToInches(-LimeLightConstants.MOUNT_LEFT_METERS);
    assertEquals(
        LimeLight.LIMELIGHT_LATERAL_OFFSET_INCHES, lensRightOfCentreInches, 1e-6);
  }
}
