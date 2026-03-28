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

//ShuffleboardManager — centralized pre-match and in-match telemetry dashboard.
public class ShuffleboardManager {

    // References

    private final LimeLight      limelight;
    private final Superstructure superstructure;
    private final SwerveSubsystem drivebase;

    // Field widget (shared across tabs)

    private final Field2d field = new Field2d();

    // Cached network table entries — write once per update() call.
    // Using GenericEntry avoids repeated tab/layout lookups every loop.

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

    // Drive tab
    private final SimpleWidget entryRobotX;
    private final SimpleWidget entryRobotY;
    private final SimpleWidget entryRobotHeading;
    private final SimpleWidget entryVelocityX;
    private final SimpleWidget entryVelocityY;
    private final SimpleWidget entryTotalSpeed;

    // Constructor — builds all tabs and widgets once at startup

    public ShuffleboardManager(
            LimeLight limelight,
            Superstructure superstructure,
            SwerveSubsystem drivebase) {

        this.limelight      = limelight;
        this.superstructure = superstructure;
        this.drivebase      = drivebase;


        //  Pre-Match Safety Checklist
        ShuffleboardTab preMatch = Shuffleboard.getTab("Pre-Match");

        // Status column (left) 
        ShuffleboardLayout statusLayout = preMatch
                .getLayout("Station & Alliance", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 4)
                .withProperties(Map.of("Label position", "LEFT"));

        entryAlliance = statusLayout
                .add("Alliance", "Unknown")
                .withWidget(BuiltInWidgets.kTextView);

        entryDriverStation = statusLayout
                .add("DS Number", -1)
                .withWidget(BuiltInWidgets.kTextView);

        entryFMSConnected = statusLayout
                .add("FMS Connected", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "green", "Color when false", "gray"));

        // Subsystem readiness column (middle)
        ShuffleboardLayout subsysLayout = preMatch
                .getLayout("Subsystem Readiness", BuiltInLayouts.kList)
                .withPosition(2, 0)
                .withSize(2, 5)
                .withProperties(Map.of("Label position", "LEFT"));

        entryShooterOk = subsysLayout
                .add("Shooter Motor OK", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "green", "Color when false", "red"));

        entryTurretOk = subsysLayout
                .add("Turret Motor OK", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "green", "Color when false", "red"));

        entryLimelightConnected = subsysLayout
                .add("Limelight Connected", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "green", "Color when false", "red"));

        entryOdometryOk = subsysLayout
                .add("Odometry Valid", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "green", "Color when false", "yellow"));

        // Turret angle dial (right — visual sanity check)
        entryTurretActualDeg = preMatch
                .add("Turret Angle (deg)", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withPosition(4, 0)
                .withSize(2, 2)
                .withProperties(Map.of("Min", -90.0, "Max", 90.0, "Show value", true));

        // Shooter RPM bar
        entryShooterActualRPM = preMatch
                .add("Shooter RPM", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(4, 2)
                .withSize(2, 1)
                .withProperties(Map.of("Min", 0.0, "Max", 6000.0, "Num tick marks", 7));

        // Hood angle
        entryHoodAngleDeg = preMatch
                .add("Hood Angle (deg)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(4, 3)
                .withSize(2, 1);

        // Shooting / Vision
        ShuffleboardTab shootingTab = Shuffleboard.getTab("Shooting");

        // Limelight block
        ShuffleboardLayout llLayout = shootingTab
                .getLayout("Limelight", BuiltInLayouts.kGrid)
                .withPosition(0, 0)
                .withSize(3, 4)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));

        entryLLHasTarget = llLayout
                .add("Has Target", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "lime", "Color when false", "red"));

        entryLLTargetCount = llLayout
                .add("Target Count", 0)
                .withWidget(BuiltInWidgets.kTextView);

        entryLLTX = llLayout
                .add("TX (horiz °)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        entryLLTY = llLayout
                .add("TY (vert °)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        entryLLDistance = llLayout
                .add("Distance (m)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        entryLLTagID = llLayout
                .add("AprilTag ID", -1)
                .withWidget(BuiltInWidgets.kTextView);

        // Shot solution block
        ShuffleboardLayout shotLayout = shootingTab
                .getLayout("Shot Solution", BuiltInLayouts.kGrid)
                .withPosition(3, 0)
                .withSize(3, 4)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));

        entryShotValid = shotLayout
                .add("Solution Valid", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "green", "Color when false", "red"));

        entryReadyToShoot = shotLayout
                .add("Ready to Shoot", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when true", "lime", "Color when false", "gray"));

        entryShotTurretDeg = shotLayout
                .add("Turret Cmd (°)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        entryShotLaunchDeg = shotLayout
                .add("Launch Angle (°)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        entryShotDistM = shotLayout
                .add("Calc Distance (m)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        entryShotRPM = shotLayout
                .add("Target RPM", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        // Message bar (full width)
        entryShotMessage = shootingTab
                .add("Solver Message", "No solution yet")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 4)
                .withSize(6, 1);

        /*
        // Live RPM number bar
        entryShooterActualRPM = shootingTab
                .add("Actual RPM", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(6, 0)
                .withSize(2, 1)
                .withProperties(Map.of("Min", 0.0, "Max", 6000.0));

        // Turret dial
        entryTurretActualDeg = shootingTab
                .add("Actual Turret °", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withPosition(6, 1)
                .withSize(2, 2)
                .withProperties(Map.of("Min", -90.0, "Max", 90.0, "Show value", true));
        */
        
        //  TAB 3 — Drive / Odometry
        ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

        // Field2d map widget
        driveTab.add("Field", field)
                .withWidget(BuiltInWidgets.kField)
                .withPosition(0, 0)
                .withSize(5, 3);

        // Pose numbers
        ShuffleboardLayout poseLayout = driveTab
                .getLayout("Robot Pose", BuiltInLayouts.kList)
                .withPosition(5, 0)
                .withSize(2, 3)
                .withProperties(Map.of("Label position", "LEFT"));

        entryRobotX = poseLayout
                .add("X (m)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        entryRobotY = poseLayout
                .add("Y (m)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        entryRobotHeading = poseLayout
                .add("Heading (°)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        // Velocity
        ShuffleboardLayout velLayout = driveTab
                .getLayout("Velocity", BuiltInLayouts.kList)
                .withPosition(5, 3)
                .withSize(2, 3)
                .withProperties(Map.of("Label position", "LEFT"));

        entryVelocityX = velLayout
                .add("Vx (m/s)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        entryVelocityY = velLayout
                .add("Vy (m/s)", 0.0)
                .withWidget(BuiltInWidgets.kTextView);

        entryTotalSpeed = velLayout
                .add("Speed (m/s)", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("Min", 0.0, "Max", 5.0));
    }

    //  update() — call this every robot periodic (50 Hz)

    /**
     * Refreshes all Shuffleboard widgets with current sensor and solver data.
     * Call this once per robot periodic loop.
     */
    public void update(ShotSolution activeSolution) {

        //  Pre-Match tab

        // Alliance
        String allianceName = DriverStation.getAlliance()
                .map(a -> a == DriverStation.Alliance.Red ? "RED" : "BLUE")
                .orElse("Unknown");
        entryAlliance.getEntry().setString(allianceName);

        // DS station number (1–3)
        entryDriverStation.getEntry().setInteger(DriverStation.getLocation().orElse(-1));

        // FMS
        entryFMSConnected.getEntry().setBoolean(DriverStation.isFMSAttached());

        double shooterRPM  = superstructure.getShooterSpeed().in(RPM);
        double turretDeg   = superstructure.getTurretAngle().in(Degrees);
        double hoodDeg     = superstructure.getHoodAngle().in(Degrees);

        entryShooterOk.getEntry().setBoolean(shooterRPM >= 0); // always true if CAN comms live
        entryTurretOk.getEntry().setBoolean(!Double.isNaN(turretDeg));

        // Limelight connected = NetworkTables entry is fresh (tv == 0 or 1, not -1)
        boolean llConnected = limelight.getTX() != 0.0 || limelight.hasTarget()
                || limelight.getTagID() >= 0;
                
        entryLimelightConnected.getEntry().setBoolean(true); // NT entry exists = connected

        // Odometry valid = pose is not at the field origin (0,0) which would indicate a reset or no localisation yet.
        Pose2d pose = drivebase.getPose();
        boolean odometryOk = pose.getTranslation().getNorm() > 0.01;
        entryOdometryOk.getEntry().setBoolean(odometryOk);

        entryTurretActualDeg.getEntry().setDouble(round1(turretDeg));
        entryShooterActualRPM.getEntry().setDouble(round1(shooterRPM));
        entryHoodAngleDeg.getEntry().setDouble(round1(hoodDeg));

                //  Shooting

        // Limelight live data
        boolean hasTarget = limelight.hasTarget();
        entryLLHasTarget.getEntry().setBoolean(hasTarget);

        entryLLTargetCount.getEntry().setInteger(hasTarget ? 1 : 0);

        entryLLTX.getEntry().setDouble(round2(limelight.getTX()));
        entryLLTY.getEntry().setDouble(round2(limelight.getTY()));
        entryLLTagID.getEntry().setInteger(limelight.getTagID());

        // Compute current limelight distance using the height formula
        double llDist = computeLLDistance();
        entryLLDistance.getEntry().setDouble(round2(llDist));

        // Shot solution
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

        entryShooterActualRPM.getEntry().setDouble(round1(shooterRPM));
        entryTurretActualDeg.getEntry().setDouble(round1(turretDeg));

        //  Drive tab

        field.setRobotPose(pose);

        entryRobotX.getEntry().setDouble(round2(pose.getX()));
        entryRobotY.getEntry().setDouble(round2(pose.getY()));
        entryRobotHeading.getEntry().setDouble(round1(pose.getRotation().getDegrees()));

        ChassisSpeeds speeds = drivebase.getFieldVelocity();
        double vx    = speeds.vxMetersPerSecond;
        double vy    = speeds.vyMetersPerSecond;
        double total = Math.hypot(vx, vy);

        entryVelocityX.getEntry().setDouble(round2(vx));
        entryVelocityY.getEntry().setDouble(round2(vy));
        entryTotalSpeed.getEntry().setDouble(round2(total));
    }

    //Convenience overload — call with no solution when no shooting mode is active.

    public void update() {
        update(null);
    }

    //  Utility

    /**
     * Computes limelight-to-hub distance using the height formula.
     * Returns 0.0 if no target is visible or angle is degenerate.
     */
    private double computeLLDistance() {
        if (!limelight.hasTarget()) return 0.0;
        double totalAngleRad = Math.toRadians(
                ProjectileMotion.LIMELIGHT_MOUNT_ANGLE_DEGREES + limelight.getTY());
        if (Math.abs(totalAngleRad) < 1e-6) return 0.0;
        double dist = (ProjectileMotion.HUB_APRILTAG_HEIGHT_METERS
                - ProjectileMotion.LIMELIGHT_MOUNT_HEIGHT_METERS)
                / Math.tan(totalAngleRad);
        return Math.max(0.0, dist);
    }

    private double round1(double v) { return Math.round(v * 10.0) / 10.0; }
    private double round2(double v) { return Math.round(v * 100.0) / 100.0; }
}