package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ProjectileMotion.ShotSolution;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import java.util.Map;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

/**
 * ShuffleboardManager — centralized pre-match and in-match telemetry dashboard.
 */
public class ShuffleboardManager {

    // References
    private final LimeLight      limelight;
    private final Superstructure superstructure;
    private final SwerveSubsystem drivebase;
    private final TurretSubsystem turret; // Added reference for diagnostics

    // Field widget (shared across tabs)
    private final Field2d field = new Field2d();

    // Pre-Match tab
    private final SimpleWidget entryAlliance;
    private final SimpleWidget entryDriverStation;
    private final SimpleWidget entryFMSConnected;
    private final SimpleWidget entryShooterOk;
    private final SimpleWidget entryTurretOk;
    private final SimpleWidget entryLimelightConnected;
    private final SimpleWidget entryOdometryOk;

    // Shooting tab
    private final SimpleWidget entryLLHasTarget;
    private final SimpleWidget entryLLTargetCount;
    private final SimpleWidget entryLLTX;
    private final SimpleWidget entryLLTY;
    private final SimpleWidget entryLLDistance;
    private final SimpleWidget entryLLTagID;
    private final SimpleWidget entryShotValid;
    private final SimpleWidget entryShotMessage;
    private final SimpleWidget entryShotTurretDeg;
    private final SimpleWidget entryShotLaunchDeg;
    private final SimpleWidget entryShotDistM;
    private final SimpleWidget entryShotRPM;
    private final SimpleWidget entryShooterActualRPM;
    private final SimpleWidget entryTurretActualDeg;
    private final SimpleWidget entryReadyToShoot;
    private final SimpleWidget entryHoodAngleDeg;

    // Turret Diagnostics (New)
    private final SimpleWidget entryTurretRawRotations;
    private final SimpleWidget entryTurretYAMSAngle;

    // Drive tab
    private final SimpleWidget entryRobotX;
    private final SimpleWidget entryRobotY;
    private final SimpleWidget entryRobotHeading;
    private final SimpleWidget entryVelocityX;
    private final SimpleWidget entryVelocityY;
    private final SimpleWidget entryTotalSpeed;

    public ShuffleboardManager(
            LimeLight limelight,
            Superstructure superstructure,
            SwerveSubsystem drivebase,
            TurretSubsystem turret) {

        this.limelight      = limelight;
        this.superstructure = superstructure;
        this.drivebase      = drivebase;
        this.turret         = turret;


        // --- TAB 1: PRE-MATCH SAFETY CHECKLIST ---
        ShuffleboardTab preMatch = Shuffleboard.getTab("Pre-Match");

        ShuffleboardLayout statusLayout = preMatch
                .getLayout("Station & Alliance", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 4)
                .withProperties(Map.of("Label position", "LEFT"));

        entryAlliance = statusLayout.add("Alliance", "Unknown");
        entryDriverStation = statusLayout.add("DS Number", -1);
        entryFMSConnected = statusLayout.add("FMS Connected", false).withWidget(BuiltInWidgets.kBooleanBox);

        ShuffleboardLayout subsysLayout = preMatch
                .getLayout("Subsystem Readiness", BuiltInLayouts.kList)
                .withPosition(2, 0)
                .withSize(2, 5)
                .withProperties(Map.of("Label position", "LEFT"));

        entryShooterOk = subsysLayout.add("Shooter Motor OK", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryTurretOk = subsysLayout.add("Turret Motor OK", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryLimelightConnected = subsysLayout.add("Limelight Connected", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryOdometryOk = subsysLayout.add("Odometry Valid", false).withWidget(BuiltInWidgets.kBooleanBox);

        entryTurretActualDeg = preMatch.add("Turret Angle (deg)", 0.0).withWidget(BuiltInWidgets.kDial)
                .withPosition(4, 0).withSize(2, 2).withProperties(Map.of("Min", -90.0, "Max", 90.0, "Show value", true));

        entryShooterActualRPM = preMatch.add("Shooter RPM", 0.0).withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(4, 2).withSize(2, 1).withProperties(Map.of("Min", 0.0, "Max", 6000.0));

        entryHoodAngleDeg = preMatch.add("Hood Angle (deg)", 0.0).withPosition(4, 3).withSize(2, 1);


        // --- TAB 2: SHOOTING / VISION ---
        ShuffleboardTab shootingTab = Shuffleboard.getTab("Shooting");

        ShuffleboardLayout llLayout = shootingTab
                .getLayout("Limelight", BuiltInLayouts.kGrid)
                .withPosition(0, 0).withSize(3, 4)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));

        entryLLHasTarget = llLayout.add("Has Target", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryLLTargetCount = llLayout.add("Target Count", 0);
        entryLLTX = llLayout.add("TX (horiz °)", 0.0);
        entryLLTY = llLayout.add("TY (vert °)", 0.0);
        entryLLDistance = llLayout.add("Distance (m)", 0.0);
        entryLLTagID = llLayout.add("AprilTag ID", -1);

        // New Diagnostics Section (Tidying up the layout)
        ShuffleboardLayout turretDiag = shootingTab
                .getLayout("Turret Hardware", BuiltInLayouts.kList)
                .withPosition(6, 0).withSize(2, 4);

        entryTurretRawRotations = turretDiag.add("Raw Encoder Rot", 0.0);
        entryTurretYAMSAngle    = turretDiag.add("YAMS Deg", 0.0);

        ShuffleboardLayout shotLayout = shootingTab
                .getLayout("Shot Solution", BuiltInLayouts.kGrid)
                .withPosition(3, 0).withSize(3, 4)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));

        entryShotValid = shotLayout.add("Solution Valid", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryReadyToShoot = shotLayout.add("Ready to Shoot", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryShotTurretDeg = shotLayout.add("Turret Cmd (°)", 0.0);
        entryShotLaunchDeg = shotLayout.add("Launch Angle (°)", 0.0);
        entryShotDistM = shotLayout.add("Calc Distance (m)", 0.0);
        entryShotRPM = shotLayout.add("Target RPM", 0.0);

        entryShotMessage = shootingTab.add("Solver Message", "No solution yet")
                .withPosition(0, 4).withSize(6, 1);


        // --- TAB 3: DRIVE / ODOMETRY ---
        ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

        driveTab.add("Field", field).withWidget(BuiltInWidgets.kField).withPosition(0, 0).withSize(5, 3);

        ShuffleboardLayout poseLayout = driveTab
                .getLayout("Robot Pose", BuiltInLayouts.kList)
                .withPosition(5, 0).withSize(2, 3)
                .withProperties(Map.of("Label position", "LEFT"));

        entryRobotX = poseLayout.add("X (m)", 0.0);
        entryRobotY = poseLayout.add("Y (m)", 0.0);
        entryRobotHeading = poseLayout.add("Heading (°)", 0.0);

        ShuffleboardLayout velLayout = driveTab
                .getLayout("Velocity", BuiltInLayouts.kList)
                .withPosition(5, 3)
                .withSize(2, 3)
                .withProperties(Map.of("Label position", "LEFT"));

        entryVelocityX = velLayout.add("Vx (m/s)", 0.0);
        entryVelocityY = velLayout.add("Vy (m/s)", 0.0);
        entryTotalSpeed = velLayout.add("Speed (m/s)", 0.0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", 0.0, "Max", 5.0));
    }

    public void update(ShotSolution activeSolution) {
        // Pre-Match Logic
        entryAlliance.getEntry().setString(DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red ? "RED" : "BLUE").orElse("Unknown"));
        entryDriverStation.getEntry().setInteger(DriverStation.getLocation().orElse(-1));
        entryFMSConnected.getEntry().setBoolean(DriverStation.isFMSAttached());

        double shooterRPM  = superstructure.getShooterSpeed().in(RPM);
        double turretDeg   = superstructure.getTurretAngle().in(Degrees);
        double hoodDeg     = superstructure.getHoodAngle().in(Degrees);

        // Hardware Diagnostics Update
        entryTurretRawRotations.getEntry().setDouble(turret.getRawEncoderRotations());
        entryTurretYAMSAngle.getEntry().setDouble(round1(turret.getRawAngle().in(Degrees)));

        entryShooterOk.getEntry().setBoolean(shooterRPM >= 0);
        entryTurretOk.getEntry().setBoolean(!Double.isNaN(turretDeg));
        entryLimelightConnected.getEntry().setBoolean(true); 

        Pose2d pose = drivebase.getPose();
        entryOdometryOk.getEntry().setBoolean(pose.getTranslation().getNorm() > 0.01);

        entryTurretActualDeg.getEntry().setDouble(round1(turretDeg));
        entryShooterActualRPM.getEntry().setDouble(round1(shooterRPM));
        entryHoodAngleDeg.getEntry().setDouble(round1(hoodDeg));

        // Shooting Logic
        boolean hasTarget = limelight.hasTarget();
        entryLLHasTarget.getEntry().setBoolean(hasTarget);
        entryLLTargetCount.getEntry().setInteger(hasTarget ? 1 : 0);
        entryLLTX.getEntry().setDouble(round2(limelight.getTX()));
        entryLLTY.getEntry().setDouble(round2(limelight.getTY()));
        entryLLTagID.getEntry().setInteger(limelight.getTagID());
        entryLLDistance.getEntry().setDouble(round2(computeLLDistance()));

        if (activeSolution != null) {
            entryShotValid.getEntry().setBoolean(activeSolution.valid);
            entryShotMessage.getEntry().setString(activeSolution.message);
            entryShotTurretDeg.getEntry().setDouble(round2(activeSolution.turretAngleDegrees));
            entryShotLaunchDeg.getEntry().setDouble(round2(activeSolution.launchAngleDegrees));
            entryShotDistM.getEntry().setDouble(round2(activeSolution.horizontalDistanceMeters));
            entryShotRPM.getEntry().setDouble(round1(activeSolution.estimatedFlywheelRPM));
        } else {
            entryShotValid.getEntry().setBoolean(false);
            entryShotMessage.getEntry().setString("No shooting mode active");
        }

        double targetRPM    = superstructure.getTargetShooterSpeed().in(RPM);
        double targetTurret = superstructure.getTargetTurretAngle().in(Degrees);
        double targetHood   = superstructure.getTargetHoodAngle().in(Degrees);
        boolean shooterReady = Math.abs(shooterRPM - targetRPM) < 100;
        boolean turretReady  = Math.abs(turretDeg - targetTurret) < 1.0;
        boolean hoodReady    = Math.abs(hoodDeg - targetHood) < 2.0;
        entryReadyToShoot.getEntry().setBoolean(shooterReady && turretReady && hoodReady && hasTarget);

        // Drive Logic
        field.setRobotPose(pose);
        entryRobotX.getEntry().setDouble(round2(pose.getX()));
        entryRobotY.getEntry().setDouble(round2(pose.getY()));
        entryRobotHeading.getEntry().setDouble(round1(pose.getRotation().getDegrees()));

        ChassisSpeeds speeds = drivebase.getFieldVelocity();
        entryVelocityX.getEntry().setDouble(round2(speeds.vxMetersPerSecond));
        entryVelocityY.getEntry().setDouble(round2(speeds.vyMetersPerSecond));
        entryTotalSpeed.getEntry().setDouble(round2(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));
    }

    public void update() { update(null); }

    private double computeLLDistance() {
        if (!limelight.hasTarget()) return 0.0;
        double totalAngleRad = Math.toRadians(ProjectileMotion.LIMELIGHT_MOUNT_ANGLE_DEGREES + limelight.getTY());
        if (Math.abs(totalAngleRad) < 1e-6) return 0.0;
        return Math.max(0.0, (ProjectileMotion.HUB_APRILTAG_HEIGHT_METERS - ProjectileMotion.LIMELIGHT_MOUNT_HEIGHT_METERS) / Math.tan(totalAngleRad));
    }

    private double round1(double v) { return Math.round(v * 10.0) / 10.0; }
    private double round2(double v) { return Math.round(v * 100.0) / 100.0; }
}