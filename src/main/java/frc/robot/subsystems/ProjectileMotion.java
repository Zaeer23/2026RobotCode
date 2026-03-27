package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.LimeLight;

/* 
        APOLOGIES IF THE ROBOT EXPLORES BECAUSE OF MY TRASH CODING, I'M NOT A PHYSICIST, 
        JUST A GUY WHO WANTS TO SHOOT BALLS INTO HOOPS 😭😭😭😭😭

        FROM SHANE; I'll add a feature to have a "impact angle" later, lowkirk tired rn

        basically this code makes it so you can calculate some stuff using given inputs
        and in return receive a "ShotSolution" object that contains the turret angle, launch angle,
        and launch speed needed to hit the target.

        there are three modes:
        
            1. Global Pose (odometry/GPS),

                    ShotSolution sol = ProjectileMotion.solveFromPose(
                        drivebase.getPose(),                        // robot field pose
                        ProjectileMotion.getHubTarget(isRedAlliance),
                        ProjectileMotion.HUB_TARGET_HEIGHT_METERS,
                        45.0 // hood's fixed angle in degrees
                    );

            2. Limelight only (no global pose needed),

                    ShotSolution sol = ProjectileMotion.solveFromLimelight(limelight, 45.0);

            
            3. Moving robot compensation (advanced, not yet implemented)

                    ShotSolution sol = ProjectileMotion.solveMovingRobotFromPose(
                        drivebase.getPose(),
                        drivebase.getFieldVelocity(),  // ChassisSpeeds in field frame
                        ProjectileMotion.getHubTarget(isRedAlliance),
                        ProjectileMotion.HUB_TARGET_HEIGHT_METERS,
                        45.0
                    );


            some key things to tune before competition:

                SHOOTER_EXIT_HEIGHT_METERS
                LIMELIGHT_MOUNT_HEIGHT_METERS
                LIMELIGHT_MOUNT_ANGLE_DEGREES
                SHOOTER_FORWARD_OFFSET_METERS


 */


public class ProjectileMotion {

            // 2026 REBUILT GAME CONSTANTS  (all above the carpet)

    // Height of the HUB scoring 
    public static final double HUB_TARGET_HEIGHT_METERS = Units.inchesToMeters(41.0);

    // Height of the HUB AprilTag centers. deriving distance from Limelight ty with the height formula.
    public static final double HUB_APRILTAG_HEIGHT_METERS = Units.inchesToMeters(44.25);
 
    // Radius of the HUB scoring opening.
    public static final double HUB_OPENING_RADIUS_METERS = Units.inchesToMeters(41.0 / 2.0);

    // FUEL ball diameter.
    public static final double FUEL_DIAMETER_METERS = Units.inchesToMeters(5.91);

    // acceleration of gravity.
    public static final double g = 9.80665;

            // Physical shooter constants

    public static double SHOOTER_EXIT_HEIGHT_METERS =       0.60; // Height of the shooter exit point.
    public static double LIMELIGHT_MOUNT_HEIGHT_METERS =    Units.inchesToMeters(20.0); // Limelight mounting height.
    public static double LIMELIGHT_MOUNT_ANGLE_DEGREES =    0.0; //Limelight mounting pitch angle (degrees, positive = tilted up)
    public static double SHOOTER_FORWARD_OFFSET_METERS =    -0.30; //Horizontal offset from robot center to shooter exit point

    public static class ShotSolution {

        public final double turretAngleDegrees;
        public final double launchAngleDegrees;
        public final double launchSpeedMetersPerSecond;
        public final double horizontaldistanceMeters;

        // approximate flywheel rpm assuming 4-inch radiius wheel
        public final double estimatedFlywheelRPM;
        public final boolean valid;
        public final String message;

        private ShotSolution(double turretDeg, double launchDeg, double speedMps, double distM, boolean valid, String msg) {
            this.turretAngleDegrees                 = turretDeg;
            this.launchAngleDegrees                  = launchDeg;
            this.launchSpeedMetersPerSecond          = speedMps;
            this.horizontaldistanceMeters            = distM;

            double wheelRadius = Units.inchesToMeters(4.0) / 2.0;
            this.estimatedFlywheelRPM                = valid ? (speedMps / wheelRadius) * (60.0 / (2.0 * Math.PI)) : 0.0;
            this.valid                                = valid;
            this.message                              = msg;
        }

        static ShotSolution invalid(String reason) {
            return new ShotSolution(0, 0, 0, 0, false, reason);
        }

        @Override
        public String toString() { // regex makes me pain ong
            if (!valid) return "ShotSolution[INVALID: " + message + "]";
            return String.format(
                "ShotSolution[turret=%.1f° | launch=%.1f° | speed=%.2f m/s | dist=%.2f m | ~%.0f RPM | %s]",
                turretAngleDegrees, launchAngleDegrees, launchSpeedMetersPerSecond,
                horizontaldistanceMeters, estimatedFlywheelRPM, message);
        }

    }

    public static ShotSolution solveFromPose(
            Pose2d robotPose,
            Translation2d targetFieldPose,
            double targetHeightM,
            double fixedLaunchAngleDegrees) {
 
        // compute the shooter's actual exit position in field coords
        Translation2d shooterExit = getShooterExitFieldPos(robotPose);
 
        // vector from shooter exit to target (horizontal plane)
        double dx = targetFieldPose.getX() - shooterExit.getX();
        double dy = targetFieldPose.getY() - shooterExit.getY();

        double horizontalDist = Math.hypot(dx, dy);
 
        // required turret heading = angle of (dx, dy) relative to robot forward
        double absoluteTargetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg        = robotPose.getRotation().getDegrees();
        double turretAngleDeg         = absoluteTargetAngleDeg - robotHeadingDeg;

        turretAngleDeg = normalizeAngle(turretAngleDeg); // just incase
 
        // height delta: target is above (or below) shooter exit
        double deltaHeight = targetHeightM - SHOOTER_EXIT_HEIGHT_METERS;
 
        // solve for launch speed at the fixed launch angle
        return solveSpeed(horizontalDist, deltaHeight, fixedLaunchAngleDegrees, turretAngleDeg,
                "Global-pose mode");
    }



    public static ShotSolution solveFromLimelight(
            LimeLight limelight,
            double fixedLaunchAngleDegrees) {
 
        if (!limelight.hasTarget()) {
            return ShotSolution.invalid("Limelight has no target");
        }
 
        double tx = limelight.getTX(); // horizontal offset in degrees
        double ty = limelight.getTY(); // vertical offset in degrees
 
        double totalAngleRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEGREES + ty);

        if (Math.abs(totalAngleRad) < 1e-6) {
            return ShotSolution.invalid("Limelight angle too close to horizontal — cannot compute distance");
        }

        double distanceToHub = (HUB_APRILTAG_HEIGHT_METERS - LIMELIGHT_MOUNT_HEIGHT_METERS) / Math.tan(totalAngleRad);
 
        if (distanceToHub <= 0) {
            return ShotSolution.invalid("Limelight computed negative distance — check mount constants");
        }
 
        double turretAngleDeg = tx; // the limelight tx IS the turret error in local frame
 
        double deltaHeight = HUB_TARGET_HEIGHT_METERS - SHOOTER_EXIT_HEIGHT_METERS;
 
        return solveSpeed(distanceToHub, deltaHeight, fixedLaunchAngleDegrees, turretAngleDeg,
                "Limelight mode (dist=" + String.format("%.2f", distanceToHub) + "m, tx=" + tx + "°)");
    }




    public static ShotSolution solveFromPoseOptimalAngle(
            Pose2d robotPose,
            Translation2d targetFieldPose,
            double targetHeightM) {
 
        Translation2d shooterExit = getShooterExitFieldPos(robotPose);

        double dx = targetFieldPose.getX() - shooterExit.getX();
        double dy = targetFieldPose.getY() - shooterExit.getY();

        double horizontalDist = Math.hypot(dx, dy);
 
        double absoluteTargetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg        = robotPose.getRotation().getDegrees();

        double turretAngleDeg         = normalizeAngle(absoluteTargetAngleDeg - robotHeadingDeg);
 
        double deltaHeight = targetHeightM - SHOOTER_EXIT_HEIGHT_METERS;
 
        return solveOptimalAngleAndSpeed(horizontalDist, deltaHeight, turretAngleDeg, "Global-pose optimal-angle mode");
    }


    public static ShotSolution solveFromLimelightOptimalAngle(LimeLight limelight) {
        if (!limelight.hasTarget()) {
            return ShotSolution.invalid("Limelight has no target");
        }
 
        double tx = limelight.getTX();
        double ty = limelight.getTY();
 
        double totalAngleRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEGREES + ty);
        if (Math.abs(totalAngleRad) < 1e-6) {
            return ShotSolution.invalid("Limelight angle too close to horizontal");
        }
        double distanceToHub = (HUB_APRILTAG_HEIGHT_METERS - LIMELIGHT_MOUNT_HEIGHT_METERS)
                / Math.tan(totalAngleRad);
 
        if (distanceToHub <= 0) {
            return ShotSolution.invalid("Negative limelight distance");
        }
 
        double deltaHeight = HUB_TARGET_HEIGHT_METERS - SHOOTER_EXIT_HEIGHT_METERS;
        return solveOptimalAngleAndSpeed(distanceToHub, deltaHeight, tx, "Limelight optimal-angle mode");
    }


    // solving speeds



    private static ShotSolution solveOptimalAngleAndSpeed(
            double horizDist, double deltaHeight,
            double turretAngleDeg, String tag) {
 
        if (horizDist <= 0) {
            return ShotSolution.invalid(tag + ": zero horizontal distance");
        }
 
        double optimalAngleDeg = 45.0 + (Math.toDegrees(Math.atan2(deltaHeight, horizDist)) / 2.0);
 
        // Clamp to a physically reasonable range for the shooter mechanism
        optimalAngleDeg = Math.max(20.0, Math.min(75.0, optimalAngleDeg));
 
        return solveSpeed(horizDist, deltaHeight, optimalAngleDeg, turretAngleDeg,
                tag + " [optimalAngle=" + String.format("%.1f", optimalAngleDeg) + "°]");
    }

    private static ShotSolution solveSpeed(
            double horizDist, double deltaHeight,
            double launchAngleDeg, double turretAngleDeg,
            String tag) {
 
        if (horizDist <= 0) {
            return ShotSolution.invalid(tag + ": zero/negative horizontal distance");
        }
 
        double theta    = Math.toRadians(launchAngleDeg);
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);
 
        double denominator = (horizDist * tanTheta) - deltaHeight;

        if (denominator <= 0) {
            return ShotSolution.invalid(tag + ": target unreachable at this angle "
                    + "(launchAngle=" + launchAngleDeg + "° too shallow or Δh=" + deltaHeight + "m)");
        }
        
        // might have to fact check this.
        double v2 = (g * horizDist * horizDist) / (2.0 * cosTheta * cosTheta * denominator);

        if (v2 < 0) {
            return ShotSolution.invalid(tag + ": negative v² — physically impossible");
        }
 
        double v = Math.sqrt(v2);

        return new ShotSolution(turretAngleDeg, launchAngleDeg, v, horizDist, true, tag);
    }




    // utility


    public static double estimateFlightTime(ShotSolution sol) {
        if (!sol.valid) return 0.0;

        double vx = sol.launchSpeedMetersPerSecond * Math.cos(Math.toRadians(sol.launchAngleDegrees));

        if (vx < 1e-6) return 0.0;

        return sol.horizontaldistanceMeters / vx;
    }


    // Compute the shooter exit position in field coordinates
    private static Translation2d getShooterExitFieldPos(Pose2d robotPose) {
        double heading = robotPose.getRotation().getRadians();

        double sx = robotPose.getX() + SHOOTER_FORWARD_OFFSET_METERS * Math.cos(heading);
        double sy = robotPose.getY() + SHOOTER_FORWARD_OFFSET_METERS * Math.sin(heading);

        return new Translation2d(sx, sy);
    }

    // Normalise angle to the range (−180, 180].
    public static double normalizeAngle(double deg) {
        while (deg > 180.0)  deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;

        return deg;
    }

    public static double speedToRPM(double speedMps, double wheelRadiusIn) {
        double radius = Units.inchesToMeters(wheelRadiusIn);
        double angularSpeed = (speedMps / radius);

        return (angularSpeed * 60.0) / (2.0 * Math.PI);
    }

    public static final Translation2d RED_HUB_CENTER   = new Translation2d(8.27, 4.04);

    public static final Translation2d BLUE_HUB_CENTER  = RED_HUB_CENTER;

    public static Translation2d getHubTarget(boolean isRedAlliance) {
        return isRedAlliance ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    }

}