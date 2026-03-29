package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/*

  File Destination:    frc/robot/subsystems/vision/LimeLight.java

  Command Execution:  frc/robot/commands/vision/LimeLightRunner.java

*/
public class LimeLight {

  public static double ALIGN_KP = 0.06;
  public static double ALIGN_MAX_OMEGA_RAD_S = 1.5;
  public static double ALIGN_TOLERANCE_DEGREES = 1.0;

  // Physical offset of the limelight lens from the robot's true center (inches).
  // Positive = limelight is to the RIGHT of center when viewed from above.
  // Measure from center of robot to center of limelight lens, left/right axis only.
  public static final double LIMELIGHT_LATERAL_OFFSET_INCHES = 9.0; // tune: measure physically

    private final NetworkTable table;

    public LimeLight(String name) {
        table = NetworkTableInstance.getDefault().getTable(name);
    }

    //some bic data access thingymabobies

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public double getTX() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getTY() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getTA() {
        return table.getEntry("ta").getDouble(0);
    }

    public int getTagID() {
        return (int) table.getEntry("tid").getDouble(-1);
    }

    public void lightMode(int number) {
        table.getEntry("ledMode").setNumber(number);
    }
    // this is just optional helpers

    // Positive tx means target is to the right
    public double getTurnCorrection(double kP) {
        return -getTX() * kP;
    }

    // Example forward correction using area
    public double getForwardCorrection(double desiredArea, double kP) {
        return (desiredArea - getTA()) * kP;
    }

    // Shane NEED TO CHANGE MATH, currently require height values, must make the math flexible based on concurrent data.
  // If possible, record limelight position in the future. Use (scale / ta) for simplier distance calculations if needed.

  // example: (return 30666 / limelight.getTA())

  // done! just need to test it, builders pls finish robot 🥺🥺
 
  /*
  12.236 at 20cm
  3.192% at 40cm
  2.025% at 50cm

  pow curve = y = 4202.278x^2 - 1.949067
   */

  public double getLimelightAprilDistance_BasedScales()
  {
    double givenScale = 4202.278; //30665.9;
    double limelightScale = getTA();

    return givenScale / limelightScale;
  }

  // this is locally to what the camera is seeing
  public Pose3d getLimelightPose3d()
  {
    double distance = getLimelightAprilDistance_BasedScales();

    double tx = Math.toRadians(getTX());
    double ty = Math.toRadians(getTY());

    double x = distance * Math.sin(tx);
    double y = distance * Math.sin(ty);
    double z = distance * Math.cos(ty);

    return new Pose3d(new Translation3d(x, y, z), new Rotation3d()); //we don't need to know the rotation
  }


  // Deprecated until we can get the limelight position, but this is the more accurate way to do it, just need to make it flexible for different heights and angles
  public double getLimelightAprilDistance_BasedHeights() 
  {
    double targetOffsetAngle_Vertical = getTX();

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0; 

    // distance from the target to the floor
    double goalHeightInches = 60.0; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return distance;
  }

  public boolean isCentered() {
        return hasTarget() && Math.abs(getCorrectedTX()) < ALIGN_TOLERANCE_DEGREES;
  }

  /**
   * Returns TX corrected for the limelight's lateral offset from robot center.
   * When correctedTX == 0, the robot center (not the limelight) is aimed at the target.
   *
   * correctedTX = tx - atan2(lateralOffset, distance)
   *
   * The parallax term is always subtracted because the limelight is always
   * to the right of robot center — it will always read slightly more rightward
   * than a centered camera would, so we compensate by subtracting.
   */
  public double getCorrectedTX() {
    if (!hasTarget()) return 0.0;
    double distanceInches = getLimelightAprilDistance_BasedScales() * 39.37;
    double parallax = Math.toDegrees(
        Math.atan2(LIMELIGHT_LATERAL_OFFSET_INCHES, distanceInches));
    return getTX() - parallax;
  }

  public double getAlignOmega() {
        if (!hasTarget()) return 0.0;
        // Use corrected TX so we align robot center, not the limelight, to the target
        double omega = -getCorrectedTX() * ALIGN_KP;
        return Math.max(-ALIGN_MAX_OMEGA_RAD_S, Math.min(ALIGN_MAX_OMEGA_RAD_S, omega));
    }

  // commands

  public static void setupPortForwardingUSB(int usbIndex) {
        String ip = "172.29." + usbIndex + ".1";
        int basePort = 5800 + (usbIndex * 10);

        for (int i = 0; i < 10; i++) {
            PortForwarder.add(basePort + i, ip, 5800 + i);
        }
    }

  /**
   * Spins the robot in place until robot center is aimed at the target.
   * Uses getCorrectedTX() to account for limelight being offset from center.
   * Bind this to a button in RobotContainer.
   */
  public Command alignCommand(SwerveSubsystem drivebase) {
    return Commands.run(
            () -> drivebase.drive(
                Translation2d.kZero,   // no translation — spin in place
                getAlignOmega(),       // proportional rotation toward corrected target
                true                   // field-relative keeps heading stable
            ),
            drivebase
        )
        .until(this::isCentered)
        .withName("LimeLight.align");
        //this should work now :)
  }

  public static class AprilTagScan {
    public final boolean hasTarget;
    public final int tagID;
    public final double tx;
    public final double ty;
    public final double distance;
    public final Pose3d pose;

    public AprilTagScan(boolean hasTarget, int tagID, double tx, double ty, double distance, Pose3d pose) {
        this.hasTarget = hasTarget;
        this.tagID = tagID;
        this.tx = tx;
        this.ty = ty;
        this.distance = distance;
        this.pose = pose;
    }

    public boolean isValid() {
        return hasTarget && tagID != -1;
    }
}

public AprilTagScan scan() {
    boolean target = hasTarget();
    return new AprilTagScan(
        target,
        getTagID(),
        getTX(),
        getTY(),
        target ? getLimelightAprilDistance_BasedScales() : 0.0,
        target ? getLimelightPose3d() : new Pose3d()
    );
    
}

}