// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(12.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }
  public static class KickerConstants {
    public static final int kKickerMotorId = 33; //NOT TEMPORARY, this is the actual ID for the kicker motor
  }
  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class ClimberConstants {
    public static final int kClimberMotorId = 36; 
    public static final double kClimberSpeed = 0.8; 
  }
   
  public static class ShooterConstants {
    // 2 Neos, 4in shooter wheels
    public static final int kLeaderMotorId = 32;
    public static final int kFollowerMotorId = 31;
    public static final double kShooterSpeed = 1.0;

    /** Flywheel diameter. Ball exit speed is derived from surface speed at this radius. */
    public static final double WHEEL_DIAMETER_INCHES = 4.0;

    /**
     * Fixed launch angle of the shooter, in degrees above horizontal. The hood is not actuated, so
     * this is a hard mechanical property of the robot and every distance/RPM calculation uses it.
     */
    public static final double LAUNCH_ANGLE_DEGREES = 45.0;

    /** Height above the carpet at which the ball leaves the shooter. */
    public static final double EXIT_HEIGHT_METERS = 0.60;

    // ---- Shot model anchor -------------------------------------------------
    // The RPM curve is projectile physics scaled by one empirical efficiency factor, and that
    // factor is back-solved from a single measured shot. To recalibrate the ENTIRE curve: park at
    // REFERENCE_DISTANCE_METERS, trim RPM until shots score, and put that RPM here.

    /** Distance from the shooter exit to the hub for the reference shot. 11 ft 6 in. */
    public static final double REFERENCE_DISTANCE_METERS = 3.5052;

    /** Flywheel RPM that scores from REFERENCE_DISTANCE_METERS. */
    public static final double REFERENCE_RPM = 3200.0;

    /**
     * Commanded RPM is clamped to this window regardless of what the model asks for.
     *
     * <p>The floor used to sit at 3200, which was above what the old model asked for at every
     * distance under about 3.4 m. Close shots were therefore all fired at the clamp rather than at
     * the computed speed, so the distance model did nothing at short range.
     */
    public static final double MIN_SHOT_RPM = 2200.0;
    public static final double MAX_SHOT_RPM = 4800.0;

    /** Mechanical ceiling; the flywheel soft limit. */
    public static final double MAX_MECHANICAL_RPM = 5600.0;

    /** Vision distances outside this window are rejected as bad measurements. */
    public static final double MIN_VALID_DISTANCE_METERS = 0.8;
    public static final double MAX_VALID_DISTANCE_METERS = 8.0;

    /** How close the flywheel must be to its setpoint to count as ready to fire. */
    public static final double READY_TOLERANCE_RPM = 100.0;
  }

  public static class TurretConstants {
    // 1 Neo, 6.875 in diameter, 4:1 gearbox, 10:1 pivot gearing, non-continuous
    // 360 deg please work holy.
    public static final int kMotorId = 35;

    /**
     * Turret pivot axis relative to robot center, in the robot frame (+X forward, +Y left, +Z up).
     * The turret sits behind center on the robot centerline.
     */
    public static final Translation3d PIVOT_TO_ROBOT_CENTER = new Translation3d(-0.205, 0.0, 0.375);

    /** Usable travel in each direction from the turret zero position. */
    public static final double MAX_ONE_DIR_FOV_DEGREES = 90.0;

    /**
     * Encoder reading, in degrees, when the turret physically points straight out the front of the
     * robot. Re-measure after any rebuild that disturbs the turret zero: vision tracking commands
     * ABSOLUTE turret angles, so an error here becomes a constant aiming bias.
     */
    public static final double ENCODER_OFFSET_DEGREES = 0.0;

    /** Distance from the pivot axis to the ball exit, along the turret aim direction. */
    public static final double SHOOTER_EXIT_RADIUS_METERS = Units.inchesToMeters(10.0);

    /** Turret counts as on-target for firing inside this error. */
    public static final double ON_TARGET_TOLERANCE_DEGREES = 1.5;
  }
   public static class HopperConstants {
    public static final int kHopperMotorId = 43;// temporary value, will need to be updated when hopper is implemented
  }
  public static class HoodConstants {
    // 1 Neo, 0-90 degree variability, 50:1 reduction
    public static final int kMotorId = 199999;
  } // temporary value, will need to be updated when hood is implemented
  public static class IntakeConstants {
    // SparkFlex controlling the intake flywheel
    public static final int kPivotMotorId = 42;
    public static final int kRollerMotorId = 41; // temporary values, will need to be updated when intake is implemented

    /**
     * Master switch for the intake. TEMPORARILY DISABLED — set back to true to restore it.
     *
     * <p>While false, nothing anywhere commands the intake: the operator bindings are not
     * registered, the default command actively holds both motors at zero, the pivot bounce is
     * dropped from the feed sequence, and autonomous skips the collection step. Flipping this one
     * value back to true restores every one of those in place.
     */
    public static final boolean ENABLED = false;
  }

    // Limelight values for limelight
    public static class LimeLightConstants {
      /** NetworkTables name of the camera. */
      public static final String NAME = "limelight";

      /**
       * AprilTag pipeline index, or -1 to leave the camera on whatever pipeline it is already set
       * to.
       *
       * <p>Defaults to -1 on purpose. Hardcoding an index means every boot yanks the camera onto
       * that pipeline, and if it is not actually the AprilTag one the Limelight goes blind with no
       * obvious cause. Only set a real number here once you have confirmed the index in the web UI.
       */
      public static final int APRILTAG_PIPELINE = -1;

      // ---- Mounting pose (MEASURE THESE ON THE REAL ROBOT) -------------------
      // Lens position in the robot frame: +X forward, +Y left, +Z up. The camera is bolted to the
      // CHASSIS, so it does not rotate with the turret and tx is NOT turret error.
      // Enter the same numbers in the Limelight web UI robot-space camera pose so the on-board
      // solve agrees with the math here.

      /** Lens distance forward of robot center. Negative means behind center. */
      public static final double MOUNT_FORWARD_METERS = -0.205;

      /**
       * Lens distance LEFT of robot center. The camera sits to the right of the turret pivot and
       * the pivot is on the centerline, so this is negative. Matches the 9 in lateral offset
       * already measured in LimeLight.LIMELIGHT_LATERAL_OFFSET_INCHES.
       */
      public static final double MOUNT_LEFT_METERS = -Units.inchesToMeters(9.0);

      /** Lens height above the carpet. */
      public static final double MOUNT_HEIGHT_METERS = Units.inchesToMeters(20.0);

      /** Camera pitch. POSITIVE means the lens is tilted UP from horizontal. */
      public static final double MOUNT_PITCH_DEGREES = 20.0;

      /** Camera yaw. POSITIVE means the lens is rotated CCW (toward robot left) from straight ahead. */
      public static final double MOUNT_YAW_DEGREES = 0.0;

      /** Height of the HUB AprilTag centers, from the official 2026 field layout. */
      public static final double HUB_TAG_HEIGHT_METERS = frc.robot.FieldConstants.tagHeightMeters(20);

      /** Latency beyond what the Limelight reports, covering NT transport and scheduler jitter. */
      public static final double LATENCY_FUDGE_MS = 20.0;

      //the horizontal and vertical distance between the center of a camera's imaging sensor and the point where the optical axis intersects that sensor (the principal point).

      public static final double[] principalPixelOffset = {-2.774 , 22.549};

      // horizontal and vertical FOV
      public static final double horizontalFOV = 54.505;
      public static final double verticalFOV = 42.239;

      // focal lengths in terms of pixel dimensions for the x and y axes, respectively
      // (f_x , f_y)
      public static final double[] focalLengths = {1221.445 , 1223.998};

      // principal point, pixel coordinates of the center of the image
      // (c_x , c_y)
      public static final double[] principalPoint = {637.226 , 502.549};

      // skew coefficient, we probably won't use this, it's always gonna be 1.
      public static final double skewCoefficient = 1.000;


      // competition april tag types

      

    }

  /**
   * Gains for the automatic drivebase lineup. The chassis yaws to face the hub and drives to a
   * fixed standoff range; the turret handles fine aim on top of that.
   */
  public static class AutoLineupConstants {
    /** Range from robot center to the hub tag that the drivebase drives to. */
    public static final double TARGET_DISTANCE_METERS = ShooterConstants.REFERENCE_DISTANCE_METERS;

    public static final double DISTANCE_TOLERANCE_METERS = 0.15;
    public static final double BEARING_TOLERANCE_DEGREES = 3.0;

    /** Yaw controller, radians per second per degree of bearing error. */
    public static final double ROTATION_KP = 0.045;
    public static final double MAX_OMEGA_RAD_PER_SEC = 2.0;
    public static final double OMEGA_SLEW_RAD_PER_SEC_SQ = 6.0;

    /** Range controller, meters per second per meter of range error. */
    public static final double FORWARD_KP = 1.6;
    public static final double MAX_FORWARD_METERS_PER_SEC = 1.6;
    public static final double FORWARD_SLEW_METERS_PER_SEC_SQ = 2.5;

    /**
     * Hold off on closing range until the bearing error is inside this, so the robot turns toward
     * the hub first instead of driving a long arc.
     */
    public static final double TURN_ONLY_BEARING_DEGREES = 25.0;

    /** Consecutive in-tolerance cycles required before the lineup reports finished. */
    public static final int SETTLE_CYCLES = 6;

    /** Cycles of missing vision tolerated before the lineup gives up and stops the drivebase. */
    public static final int MAX_MISSING_CYCLES = 25;
  }
}
