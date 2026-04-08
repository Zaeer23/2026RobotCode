package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ProjectileMotion.ShotSolution;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShuffleboardManager {

    public enum ShooterControlProfile {
        CURRENT("Current Controls"),
        ALT_TWO("Trigger Feed Layout");

        private final String label;

        ShooterControlProfile(String label) {
            this.label = label;
        }

        @Override
        public String toString() {
            return label;
        }
    }

    public enum StartPosition {
        AMP_SIDE("Amp Side", new Pose2d(1.60, 6.70, Rotation2d.fromDegrees(0.0))),
        CENTER("Center", new Pose2d(1.45, 4.10, Rotation2d.fromDegrees(0.0))),
        SOURCE_SIDE("Source Side", new Pose2d(1.60, 1.50, Rotation2d.fromDegrees(0.0)));

        private final String label;
        private final Pose2d pose;

        StartPosition(String label, Pose2d pose) {
            this.label = label;
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }

        @Override
        public String toString() {
            return label;
        }
    }

    private static final String PREMATCH_TAB = "2026 Pre-Match";
    private static final String SHOOTING_TAB = "2026 Shooting";
    private static final String DRIVE_TAB = "2026 Drive";
    private static final String MATCH_TAB = "2026 Match";
    private static final String DASHBOARD_BUILD = "2026_MATCH_V3";

    private final LimeLight limelight;
    private final Superstructure superstructure;
    private final SwerveSubsystem drivebase;
    private final TurretSubsystem turret;
    private final Field2d field = new Field2d();
    private final SendableChooser<ShooterControlProfile> controlProfileChooser = new SendableChooser<>();
    private final SendableChooser<Double> driveSpeedChooser = new SendableChooser<>();
    private final SendableChooser<StartPosition> startPositionChooser = new SendableChooser<>();

    private final SimpleWidget entryAlliance;
    private final SimpleWidget entryDriverStation;
    private final SimpleWidget entryFMSConnected;
    private final SimpleWidget entryShooterOk;
    private final SimpleWidget entryTurretOk;
    private final SimpleWidget entryLimelightConnected;
    private final SimpleWidget entryOdometryOk;
    private final SimpleWidget entryTurretActualDeg;
    private final SimpleWidget entryShooterActualRPM;
    private final SimpleWidget entryHoodAngleDeg;

    private final SimpleWidget entryLLHasTarget;
    private final SimpleWidget entryLLTargetCount;
    private final SimpleWidget entryLLTX;
    private final SimpleWidget entryLLTY;
    private final SimpleWidget entryLLDistance;
    private final SimpleWidget entryLLTagID;
    private final SimpleWidget entryLLLatencyMs;
    private final SimpleWidget entryLLStreamState;
    private final SimpleWidget entryShotValid;
    private final SimpleWidget entryShotMessage;
    private final SimpleWidget entryShotTurretDeg;
    private final SimpleWidget entryShotLaunchDeg;
    private final SimpleWidget entryShotDistM;
    private final SimpleWidget entryShotRPM;
    private final SimpleWidget entryReadyToShoot;

    private final SimpleWidget entryTurretRawRotations;
    private final SimpleWidget entryTurretYAMSAngle;
    private final SimpleWidget entryTurretLastOutput;
    private final SimpleWidget entryTurretHealth;
    private final SimpleWidget entryTurretTrackTag;
    private final SimpleWidget entryTurretTrackError;
    private final SimpleWidget entryTurretTrackAge;

    private final SimpleWidget entryRobotX;
    private final SimpleWidget entryRobotY;
    private final SimpleWidget entryRobotHeading;
    private final SimpleWidget entryVelocityX;
    private final SimpleWidget entryVelocityY;
    private final SimpleWidget entryTotalSpeed;

    private final SimpleWidget entryMatchHasTarget;
    private final SimpleWidget entryMatchTagId;
    private final SimpleWidget entryMatchTx;
    private final SimpleWidget entryMatchTy;
    private final SimpleWidget entryMatchDistance;
    private final SimpleWidget entryMatchLatency;
    private final SimpleWidget entryMatchTurret;
    private final SimpleWidget entryMatchShooter;
    private final SimpleWidget entryFieldZone;
    private final SimpleWidget entryMatchReady;
    private final SimpleWidget entryMatchStream;
    private final SimpleWidget entryMatchSpeed;
    private final SimpleWidget entryMatchTurretHealth;
    private final SimpleWidget entryActiveControlSet;
    private final SimpleWidget entrySelectedDriveScale;
    private final SimpleWidget entrySelectedStartPose;

    public ShuffleboardManager(
            LimeLight limelight,
            Superstructure superstructure,
            SwerveSubsystem drivebase,
            TurretSubsystem turret) {

        this.limelight = limelight;
        this.superstructure = superstructure;
        this.drivebase = drivebase;
        this.turret = turret;

        SmartDashboard.putString("Dashboard/Build", DASHBOARD_BUILD);
        System.out.println("[SHUFFLEBOARD] Initializing " + DASHBOARD_BUILD);

        ShuffleboardTab preMatch = Shuffleboard.getTab(PREMATCH_TAB);
        ShuffleboardLayout statusLayout = preMatch
                .getLayout("Station and Alliance", BuiltInLayouts.kList)
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

        entryTurretActualDeg = preMatch.add("Turret Angle (deg)", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withPosition(4, 0)
                .withSize(2, 2)
                .withProperties(Map.of("Min", -90.0, "Max", 90.0, "Show value", true));

        entryShooterActualRPM = preMatch.add("Shooter RPM", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(4, 2)
                .withSize(2, 1)
                .withProperties(Map.of("Min", 0.0, "Max", 6000.0));

        entryHoodAngleDeg = preMatch.add("Hood Angle (deg)", 0.0)
                .withPosition(4, 3)
                .withSize(2, 1);

        configureChoosers();
        preMatch.add("Shooter Controls", controlProfileChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(6, 0)
                .withSize(3, 1);
        preMatch.add("Drive Speed", driveSpeedChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(6, 1)
                .withSize(3, 1);
        preMatch.add("Start Position", startPositionChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(6, 2)
                .withSize(3, 1);

        ShuffleboardTab shootingTab = Shuffleboard.getTab(SHOOTING_TAB);
        shootingTab.add("Vision Status", "Live Limelight diagnostics")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 0)
                .withSize(4, 1);

        ShuffleboardLayout llLayout = shootingTab
                .getLayout("Limelight Feed", BuiltInLayouts.kGrid)
                .withPosition(0, 1)
                .withSize(4, 4)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));

        entryLLHasTarget = llLayout.add("Has Target", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryLLTargetCount = llLayout.add("Target Count", 0);
        entryLLTX = llLayout.add("TX (deg)", 0.0);
        entryLLTY = llLayout.add("TY (deg)", 0.0);
        entryLLDistance = llLayout.add("Distance (m)", 0.0);
        entryLLTagID = llLayout.add("AprilTag ID", -1);
        entryLLLatencyMs = llLayout.add("Latency (ms)", 0.0);
        entryLLStreamState = llLayout.add("Stream", "SEARCHING");

        ShuffleboardLayout shotLayout = shootingTab
                .getLayout("Shot Solution", BuiltInLayouts.kGrid)
                .withPosition(4, 1)
                .withSize(4, 4)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));

        entryShotValid = shotLayout.add("Solution Valid", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryReadyToShoot = shotLayout.add("Ready to Shoot", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryShotTurretDeg = shotLayout.add("Turret Cmd (deg)", 0.0);
        entryShotLaunchDeg = shotLayout.add("Launch Angle (deg)", 0.0);
        entryShotDistM = shotLayout.add("Calc Distance (m)", 0.0);
        entryShotRPM = shotLayout.add("Target RPM", 0.0);

        entryShotMessage = shootingTab.add("Solver Message", "No solution yet")
                .withPosition(0, 5)
                .withSize(8, 1);

        ShuffleboardLayout turretDiag = shootingTab
                .getLayout("Turret Hardware", BuiltInLayouts.kList)
                .withPosition(8, 1)
                .withSize(2, 6);

        entryTurretRawRotations = turretDiag.add("Raw Encoder Rot", 0.0);
        entryTurretYAMSAngle = turretDiag.add("YAMS Deg", 0.0);
        entryTurretLastOutput = turretDiag.add("Last Output", 0.0);
        entryTurretHealth = turretDiag.add("Tracking Healthy", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryTurretTrackTag = turretDiag.add("Track Tag", -1);
        entryTurretTrackError = turretDiag.add("Track Error", 0.0);
        entryTurretTrackAge = turretDiag.add("Track Age (s)", 0.0);

        ShuffleboardTab driveTab = Shuffleboard.getTab(DRIVE_TAB);
        driveTab.add("Field", field)
                .withWidget(BuiltInWidgets.kField)
                .withPosition(0, 0)
                .withSize(5, 3);

        ShuffleboardLayout poseLayout = driveTab
                .getLayout("Robot Pose", BuiltInLayouts.kList)
                .withPosition(5, 0)
                .withSize(2, 3)
                .withProperties(Map.of("Label position", "LEFT"));

        entryRobotX = poseLayout.add("X (m)", 0.0);
        entryRobotY = poseLayout.add("Y (m)", 0.0);
        entryRobotHeading = poseLayout.add("Heading (deg)", 0.0);

        ShuffleboardLayout velLayout = driveTab
                .getLayout("Velocity", BuiltInLayouts.kList)
                .withPosition(5, 3)
                .withSize(2, 3)
                .withProperties(Map.of("Label position", "LEFT"));

        entryVelocityX = velLayout.add("Vx (m/s)", 0.0);
        entryVelocityY = velLayout.add("Vy (m/s)", 0.0);
        entryTotalSpeed = velLayout.add("Speed (m/s)", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("Min", 0.0, "Max", 5.0));

        ShuffleboardTab matchTab = Shuffleboard.getTab(MATCH_TAB);
        matchTab.add("2026 Field", field)
                .withWidget(BuiltInWidgets.kField)
                .withPosition(0, 0)
                .withSize(8, 5);

        ShuffleboardLayout matchVision = matchTab
                .getLayout("Driver Overlay", BuiltInLayouts.kGrid)
                .withPosition(8, 0)
                .withSize(4, 5)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 6));

        entryMatchHasTarget = matchVision.add("Target Lock", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryMatchReady = matchVision.add("Ready", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryMatchTurretHealth = matchVision.add("Turret Health", false).withWidget(BuiltInWidgets.kBooleanBox);
        entryMatchTagId = matchVision.add("Target Tag", -1);
        entryMatchStream = matchVision.add("Stream", "SEARCHING");
        entryMatchTx = matchVision.add("TX", 0.0);
        entryMatchTy = matchVision.add("TY", 0.0);
        entryMatchDistance = matchVision.add("Distance", 0.0);
        entryMatchLatency = matchVision.add("Latency", 0.0);
        entryMatchTurret = matchVision.add("Turret Deg", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("Min", -90.0, "Max", 90.0, "Show value", true));
        entryMatchShooter = matchVision.add("Shooter RPM", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("Min", 0.0, "Max", 6000.0));
        entryMatchSpeed = matchVision.add("Speed", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("Min", 0.0, "Max", 5.0));
        entryFieldZone = matchVision.add("Field Zone", "Center");
        entryActiveControlSet = matchVision.add("Controls", ShooterControlProfile.CURRENT.toString());
        entrySelectedDriveScale = matchVision.add("Drive Scale", 0.65);
        entrySelectedStartPose = matchVision.add("Start Pose", StartPosition.CENTER.toString());

        initializeFieldDecor();
        Shuffleboard.selectTab(MATCH_TAB);
    }

    public void update(ShotSolution activeSolution) {
        entryAlliance.getEntry().setString(
                DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red ? "RED" : "BLUE").orElse("Unknown"));
        entryDriverStation.getEntry().setInteger(DriverStation.getLocation().orElse(-1));
        entryFMSConnected.getEntry().setBoolean(DriverStation.isFMSAttached());

        double shooterRPM = superstructure.getShooterSpeed().in(RPM);
        double turretDeg = superstructure.getTurretAngle().in(Degrees);
        double hoodDeg = superstructure.getHoodAngle().in(Degrees);
        Pose2d pose = drivebase.getPose();

        entryTurretRawRotations.getEntry().setDouble(turret.getRawEncoderRotations());
        entryTurretYAMSAngle.getEntry().setDouble(round1(turret.getRawAngle().in(Degrees)));
        entryTurretLastOutput.getEntry().setDouble(round2(turret.getLastTrackingOutput()));
        entryTurretHealth.getEntry().setBoolean(turret.isTrackingHealthy());
        entryTurretTrackTag.getEntry().setInteger(turret.getLastTrackingTagId());
        entryTurretTrackError.getEntry().setDouble(round2(turret.getFilteredTrackingTxDegrees()));
        entryTurretTrackAge.getEntry().setDouble(round2(turret.getLastValidTrackingAgeSeconds()));

        entryShooterOk.getEntry().setBoolean(shooterRPM >= 0);
        entryTurretOk.getEntry().setBoolean(!Double.isNaN(turretDeg));
        entryLimelightConnected.getEntry().setBoolean(true);
        entryOdometryOk.getEntry().setBoolean(pose.getTranslation().getNorm() > 0.01);

        entryTurretActualDeg.getEntry().setDouble(round1(turretDeg));
        entryShooterActualRPM.getEntry().setDouble(round1(shooterRPM));
        entryHoodAngleDeg.getEntry().setDouble(round1(hoodDeg));

        LimeLight.AprilTagScan directScan = limelight.scanDirect();
        boolean hasTarget = directScan.hasTarget;
        entryLLHasTarget.getEntry().setBoolean(hasTarget);
        entryLLTargetCount.getEntry().setInteger(hasTarget ? 1 : 0);
        entryLLTX.getEntry().setDouble(round2(directScan.tx));
        entryLLTY.getEntry().setDouble(round2(directScan.ty));
        entryLLTagID.getEntry().setInteger(directScan.tagID);
        entryLLDistance.getEntry().setDouble(round2(directScan.distance));
        entryLLLatencyMs.getEntry().setDouble(round1(directScan.latencyMs));
        entryLLStreamState.getEntry().setString(hasTarget ? "TRACKING" : "SEARCHING");

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
            entryShotTurretDeg.getEntry().setDouble(0.0);
            entryShotLaunchDeg.getEntry().setDouble(0.0);
            entryShotDistM.getEntry().setDouble(0.0);
            entryShotRPM.getEntry().setDouble(0.0);
        }

        double targetRPM = superstructure.getTargetShooterSpeed().in(RPM);
        double targetTurret = superstructure.getTargetTurretAngle().in(Degrees);
        double targetHood = superstructure.getTargetHoodAngle().in(Degrees);
        boolean shooterReady = Math.abs(shooterRPM - targetRPM) < 100;
        boolean turretReady = Math.abs(turretDeg - targetTurret) < 1.0;
        boolean hoodReady = Math.abs(hoodDeg - targetHood) < 2.0;
        entryReadyToShoot.getEntry().setBoolean(shooterReady && turretReady && hoodReady && hasTarget);

        field.setRobotPose(pose);
        updateFieldObjects(directScan, pose);
        entryRobotX.getEntry().setDouble(round2(pose.getX()));
        entryRobotY.getEntry().setDouble(round2(pose.getY()));
        entryRobotHeading.getEntry().setDouble(round1(pose.getRotation().getDegrees()));

        ChassisSpeeds speeds = drivebase.getFieldVelocity();
        entryVelocityX.getEntry().setDouble(round2(speeds.vxMetersPerSecond));
        entryVelocityY.getEntry().setDouble(round2(speeds.vyMetersPerSecond));
        entryTotalSpeed.getEntry().setDouble(round2(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));

        entryMatchHasTarget.getEntry().setBoolean(hasTarget);
        entryMatchReady.getEntry().setBoolean(shooterReady && turretReady && hoodReady && hasTarget);
        entryMatchTurretHealth.getEntry().setBoolean(turret.isTrackingHealthy());
        entryMatchTagId.getEntry().setInteger(directScan.tagID);
        entryMatchStream.getEntry().setString(hasTarget ? "TRACKING" : "SEARCHING");
        entryMatchTx.getEntry().setDouble(round2(directScan.tx));
        entryMatchTy.getEntry().setDouble(round2(directScan.ty));
        entryMatchDistance.getEntry().setDouble(round2(directScan.distance));
        entryMatchLatency.getEntry().setDouble(round1(directScan.latencyMs));
        entryMatchTurret.getEntry().setDouble(round1(turretDeg));
        entryMatchShooter.getEntry().setDouble(round1(shooterRPM));
        entryMatchSpeed.getEntry().setDouble(round2(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));
        entryFieldZone.getEntry().setString(describeFieldZone(pose));
        entryActiveControlSet.getEntry().setString(getSelectedControlProfile().toString());
        entrySelectedDriveScale.getEntry().setDouble(getSelectedDriveSpeedScale());
        entrySelectedStartPose.getEntry().setString(getSelectedStartPosition().toString());
        SmartDashboard.putString("Dashboard/ActiveTab", MATCH_TAB);
    }

    public void update() {
        update(null);
    }

    private void updateFieldObjects(LimeLight.AprilTagScan scan, Pose2d robotPose) {
        field.getObject("Robot Ghost").setPose(robotPose);
        if (!scan.hasTarget || scan.distance <= 0.0) {
            field.getObject("Limelight Target").setPoses();
            field.getObject("Aim Line").setPoses();
            field.getObject("Selected Start").setPose(getSelectedStartPosition().getPose());
            return;
        }

        double targetHeadingRadians = robotPose.getRotation().getRadians() + Math.toRadians(scan.tx);
        double targetX = robotPose.getX() + (scan.distance * Math.cos(targetHeadingRadians));
        double targetY = robotPose.getY() + (scan.distance * Math.sin(targetHeadingRadians));
        Pose2d targetPose = new Pose2d(targetX, targetY, robotPose.getRotation());
        field.getObject("Limelight Target").setPose(targetPose);
        field.getObject("Aim Line").setPoses(robotPose, targetPose);
        field.getObject("Selected Start").setPose(getSelectedStartPosition().getPose());
    }

    private void initializeFieldDecor() {
        field.getObject("Home Marker").setPose(new Pose2d(1.5, 4.1, new edu.wpi.first.math.geometry.Rotation2d()));
        field.getObject("Mid Marker").setPose(new Pose2d(8.3, 4.1, new edu.wpi.first.math.geometry.Rotation2d()));
        field.getObject("Opponent Marker").setPose(new Pose2d(15.1, 4.1, new edu.wpi.first.math.geometry.Rotation2d()));
        field.getObject("Selected Start").setPose(getSelectedStartPosition().getPose());
    }

    private void configureChoosers() {
        controlProfileChooser.setDefaultOption(
                ShooterControlProfile.CURRENT.toString(),
                ShooterControlProfile.CURRENT);
        controlProfileChooser.addOption(
                ShooterControlProfile.ALT_TWO.toString(),
                ShooterControlProfile.ALT_TWO);

        driveSpeedChooser.setDefaultOption("65%", 0.65);
        driveSpeedChooser.addOption("35%", 0.35);
        driveSpeedChooser.addOption("100%", 1.00);

        startPositionChooser.setDefaultOption(StartPosition.CENTER.toString(), StartPosition.CENTER);
        startPositionChooser.addOption(StartPosition.AMP_SIDE.toString(), StartPosition.AMP_SIDE);
        startPositionChooser.addOption(StartPosition.SOURCE_SIDE.toString(), StartPosition.SOURCE_SIDE);
    }

    public ShooterControlProfile getSelectedControlProfile() {
        ShooterControlProfile selected = controlProfileChooser.getSelected();
        return selected != null ? selected : ShooterControlProfile.CURRENT;
    }

    public double getSelectedDriveSpeedScale() {
        Double selected = driveSpeedChooser.getSelected();
        return selected != null ? selected.doubleValue() : 0.65;
    }

    public StartPosition getSelectedStartPosition() {
        StartPosition selected = startPositionChooser.getSelected();
        return selected != null ? selected : StartPosition.CENTER;
    }

    private String describeFieldZone(Pose2d pose) {
        if (pose.getX() < 5.5) {
            return "Home Wing";
        }
        if (pose.getX() < 11.0) {
            return "Midfield";
        }
        return "Opponent Wing";
    }

    private double round1(double v) {
        return Math.round(v * 10.0) / 10.0;
    }

    private double round2(double v) {
        return Math.round(v * 100.0) / 100.0;
    }
}
