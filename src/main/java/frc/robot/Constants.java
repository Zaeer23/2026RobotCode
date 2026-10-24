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
  public static final double MAX_SPEED  = Units.feetToMeters(10);
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
    /**
     * Whether a climber is actually wired to {@link #kClimberMotorId}.
     *
     * <p>False: this robot has no climber and nothing is connected to CAN ID 36. While false,
     * ClimberSubsystem never constructs a SparkMax, so the ID is never opened and the bus stays
     * quiet. Opening it is what produced the repeating REVLib timeout, because configure() on an
     * ID with no device behind it retries and reports a fault on every loop.
     */
    public static final boolean PRESENT = false;

    /** Reserved for the climber. Not populated on this robot - see {@link #PRESENT}. */
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

    /**
     * Constant correction subtracted from every vision range, in meters. Currently 7 inches.
     *
     * <p>The camera reports consistently FAR, so the shot curve was being asked for a longer shot
     * than the real one and every ball went long. A fixed bias like this is the signature of an
     * origin mismatch rather than a bad angle: the ty solve returns range to the tag's PLANE, the
     * tag sits proud of the structure it is bolted to, and the measuring tape goes to a different
     * reference point again. An angular error would grow with distance; this does not.
     *
     * <p>To re-measure: park at a tape-measured distance, compare {@code Shooter/DistanceFeet},
     * and adjust by the difference. If the error instead GROWS with distance, this is the wrong
     * knob — fix MOUNT_HEIGHT_METERS or MOUNT_PITCH_DEGREES with BACK+B first.
     */
    public static final double RANGE_BIAS_METERS = Units.inchesToMeters(7.0);

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

    /**
     * Flywheel soft limit. Declared first so MAX_SHOT_RPM can use it.
     *
     * <p>Was 3600, which was never a mechanical limit — it was a number well under what the
     * hardware does. The flywheel is two NEOs at 1:1, so its free speed is the motor's 5676 RPM,
     * and 3600 is 63% of that. It was capping range at 5.29 m and putting the OUTPOST shot
     * 4 cm out of reach for no physical reason.
     *
     * <p>4500 is chosen from the SysID feedforward rather than guessed. At kS 0.191 and kV 0.11858
     * volts per rotation/second, holding 4500 RPM takes 9.08 V, which leaves 2.9 V of control
     * authority at a fresh 12 V battery and still has headroom when a match sags it to 11. That
     * is 79% of free speed, so the motors are not being asked to run at the top of their curve
     * where they have no torque left to recover between shots.
     *
     * <p>Going further is where it stops being conservative: 5000 RPM needs 10.07 V and leaves
     * under 2 V at a FRESH battery, which is nothing once it sags.
     *
     * <p>The 40 A stator limit is unchanged. It governs how fast the wheel spins back up between
     * shots, not how fast it can turn, so a higher ceiling costs recovery time rather than
     * risking the motors.
     */
    public static final double MAX_MECHANICAL_RPM = 4500.0;

    /**
     * Upper end of the commanded range.
     *
     * <p>Capped at the mechanical ceiling on purpose. This was a flat 4800 while the flywheel soft
     * limit sat at 3600, so for any shot the model priced above 3600 the commanded speed and the
     * achievable speed quietly diverged: the distance/RPM relationship went flat without anything
     * saying so. Deriving it from the ceiling means the model can never ask for a speed the
     * mechanism cannot reach, and {@code Shooter/Saturated} flags when a shot is at the limit.
     */
    public static final double MAX_SHOT_RPM = MAX_MECHANICAL_RPM;

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
     * Encoder reading, in degrees, when the turret physically points in ZERO_ROBOT_DIRECTION_DEGREES
     * (straight out the back). Re-measure after any rebuild that disturbs the turret zero: vision tracking commands
     * ABSOLUTE turret angles, so an error here becomes a constant aiming bias.
     *
     * <p>Measured on the robot 2026-10-22, in two passes. The turret first aimed about 10 degrees
     * right (CW) of the tag, by the same amount whether the robot was close to it or far from it.
     * A bias that does not change with range is angular, not positional, so it is the turret zero
     * rather than the camera's 8.5 inch side offset or the range bias -- those would both grow as
     * the robot closed in. +10 left about 5 degrees of the same bias still to the right, so the
     * full correction is 15. Positive is CCW, which swings the shot back left.
     *
     * <p>This value and {@code Turret/AimTrimDegrees} are added at the same point in
     * {@code TurretAimCommand.solveTurretAngle}, so they stack. If the trim was dialled in to
     * cover this bias, zero it (Turret/ResetAimTrim on the dashboard) or the correction applies
     * twice and the shot swings 20 degrees the other way.
     */
    public static final double ENCODER_OFFSET_DEGREES = 15.0;

    /**
     * Robot-frame direction the shooter points when the turret encoder reads zero, CCW-positive.
     * 0 would be straight out the front; 180 is straight out the back.
     *
     * <p>The turret is mounted at the BACK and shoots rearward, so its zero is 180 degrees from
     * robot forward. The aim solve produces a robot-frame direction, and this is what converts it
     * into an encoder angle. Without it every commanded angle was 180 degrees out, and since the
     * turret only travels +/-90 those commands clamped at a soft limit instead of aiming.
     *
     * <p>This is the same 180 already baked into {@code getRobotAdjustedAngle()}.
     */
    public static final double ZERO_ROBOT_DIRECTION_DEGREES = 180.0;

    /** Distance from the pivot axis to the ball exit, along the turret aim direction. */
    public static final double SHOOTER_EXIT_RADIUS_METERS = Units.inchesToMeters(10.0);

    /** Turret counts as on-target for firing inside this error. */
    public static final double ON_TARGET_TOLERANCE_DEGREES = 1.5;
  }
   public static class HopperConstants {
    public static final int kHopperMotorId = 43;// temporary value, will need to be updated when hopper is implemented
  }
  public static class IntakeConstants {
    // SparkFlex controlling the intake flywheel
    public static final int kPivotMotorId = 42;
    public static final int kRollerMotorId = 41; // temporary values, will need to be updated when intake is implemented

    /**
     * Master switch for the intake.
     *
     * <p>While false, nothing anywhere commands the intake: the operator bindings are not
     * registered, the default command actively holds both motors at zero, the pivot bounce is
     * dropped from the feed sequence, and autonomous skips the collection step. Flipping this one
     * value back to true restores every one of those in place.
     */
    public static final boolean ENABLED = true;
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
      public static final int APRILTAG_PIPELINE = 0;

      // ---- Mounting pose (MEASURE THESE ON THE REAL ROBOT) -------------------
      // Lens position in the robot frame: +X forward, +Y left, +Z up. The camera is bolted to the
      // CHASSIS, so it does not rotate with the turret and tx is NOT turret error.
      // Enter the same numbers in the Limelight web UI robot-space camera pose so the on-board
      // solve agrees with the math here.

      /** Lens distance forward of robot center. Negative means behind center. */
      public static final double MOUNT_FORWARD_METERS = -0.205;

      /**
       * Lens distance LEFT of robot centre, in the robot frame.
       *
       * <p>Robot left means left while looking forward from behind the robot. An observer standing
       * at the front and looking toward the rear sees +Y on their RIGHT. Verify the physical lens
       * position before changing this sign; a lateral mounting error changes parallax with range.
       *
       * <p>8.375 in (8 1/4 + 1/8) tape-measured on the robot 2026-10-23, replacing an estimated
       * 8.5. Worth knowing how little that refinement buys: this term enters the aim as
       * {@code atan(error / range)}, so an eighth of an inch is 0.18 deg at 1 m and 0.09 deg at
       * 2 m -- far below what anyone can see downrange.
       *
       * <p>What this term DOES explain is a bias that shrinks with distance, because the whole
       * 8.375 in correction is worth 12 deg at 1 m, 6 deg at 2 m and 2.5 deg at 5 m. So an aim
       * error with that shape is this correction landing wrong, not the turret zero -- a wrong
       * turret zero is a fixed number of degrees at every range. The usual reason it lands wrong
       * is {@code rangeMeters} being wrong, since the correction is applied at that range, and
       * range comes from MOUNT_HEIGHT_METERS and MOUNT_PITCH_DEGREES below.
       */
      public static final double MOUNT_LEFT_METERS = Units.inchesToMeters(8.375);

      /** Lens height above the carpet. */
      public static final double MOUNT_HEIGHT_METERS = Units.inchesToMeters(20.0);

      /** Camera pitch. POSITIVE means the lens is tilted UP from horizontal. */
      public static final double MOUNT_PITCH_DEGREES = 20.0;

      /** Camera yaw. POSITIVE means the lens is rotated CCW (toward robot left) from straight ahead. */
      /**
       * Camera yaw in the robot frame, CCW-positive. 0 looks out the FRONT, 180 looks out the BACK.
       *
       * <p>180 because the Limelight faces the same way as the turret, out the back of the robot.
       * This was 0, which placed every detected tag in FRONT of the robot — a 180 degree error in
       * the aim solve. The rotation also handles handedness: facing backwards, a tag to the
       * camera's right is on the robot's LEFT, and {@code bearing = MOUNT_YAW - tx} gets that right
       * without any extra sign flipping.
       */
      public static final double MOUNT_YAW_DEGREES = 180.0;

      /** Height of the HUB AprilTag centers, from the official 2026 field layout. */
      public static final double HUB_TAG_HEIGHT_METERS = frc.robot.vision.TagFunction.HUB.heightMeters();

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
