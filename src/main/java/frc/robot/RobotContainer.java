// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ShootingCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.vision.AprilTagCamera;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShuffleboardManager;
import frc.robot.subsystems.ShuffleboardManager.ShooterControlProfile;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.KickerSubsystem;

import static edu.wpi.first.units.Units.Degrees;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private static final double ALT_TURRET_MANUAL_SPEED = 0.18;

  /** Driver USB camera. TEMPORARILY DISABLED — set true to bring the stream back. */
  private static final boolean DRIVER_CAMERA_ENABLED = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController shooterXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> (driverXbox.getLeftY() * -1),
                                                                () -> (driverXbox.getLeftX() * -1))
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -(driverXbox.getLeftY()),
                                                                        () -> -(driverXbox.getLeftX()))
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  // Declare the Limelight subsystem
private final AprilTagCamera m_camera = new AprilTagCamera(Constants.LimeLightConstants.NAME);
   // Define the components (You may need to add imports for these)
private final ShooterSubsystem m_shooter = new ShooterSubsystem();
private final TurretSubsystem m_turret   = new TurretSubsystem();
private final HoodSubsystem m_hood       = new HoodSubsystem();
private final KickerSubsystem m_kicker   = new KickerSubsystem();
private final IntakeSubsystem m_intake   = new IntakeSubsystem();
private final HopperSubsystem m_hopper   = new HopperSubsystem();
private final ClimberSubsystem m_climber = new ClimberSubsystem();
private final UsbCamera m_driverCamera = startDriverCamera();


// Create the Superstructure instance
private final Superstructure m_superstructure = new Superstructure(
    m_shooter, m_turret, m_hood, m_intake, m_hopper, m_kicker, m_camera
);
private final ShuffleboardManager m_shuffleboard = new ShuffleboardManager(
    m_camera, m_superstructure, drivebase, m_turret, m_driverCamera
);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    configureSubSystemKeys();
    configureLimeLightKeys();
    configureAutoChooser();
  }

  private void configureAutoChooser()
  {
    // Vision-driven autos are gone along with the old targeting stack. They come back once tag
    // detection is proven on the real robot; shipping an auto that steers off a camera we cannot
    // yet trust would just move the failure somewhere harder to debug.
    autoChooser.setDefaultOption(
        "Blind timed shoot (no vision)",
        new ShootingCommand(m_shooter, m_kicker, m_hopper));
    autoChooser.addOption("Do nothing", Commands.none());
    SmartDashboard.putData("Auto Routine", autoChooser);
  }

  public void configureSubSystemKeys()
  {
    new Trigger(() -> isCurrentShooterProfile() && shooterXbox.a().getAsBoolean())
        .whileTrue(m_superstructure.feedAllCommand());
    // Three deliberately separate vision actions, on three separate buttons:
    //   driver RB     -> LINEUP ONLY   : moves the chassis, never shoots
    //   operator Y    -> TRACK + SHOOT : shoots, never moves the chassis
    //   operator LB   -> LINEUP + SHOOT: does both
    // Keeping them apart means the driver can reposition without firing, and the operator can fire
    // without the robot driving itself out from under them.

    // Automatic aiming and lineup were removed with the old targeting stack and come back once tag
    // detection is verified on the robot. Only the range-based flywheel speed survives, below.

    // SPIN FOR RANGE — sets flywheel speed from the measured hub distance. The operator still aims
    // the turret and still picks the moment to fire with the manual feed on A.
    new Trigger(() -> isCurrentShooterProfile() && shooterXbox.rightBumper().getAsBoolean())
        .whileTrue(m_superstructure.shootAtMeasuredRangeCommand());
    new Trigger(() -> isCurrentShooterProfile() && shooterXbox.povLeft().getAsBoolean())
        .onTrue(m_shooter.decreaseLimelightPowerForLastShotCommand());
    new Trigger(() -> isCurrentShooterProfile() && shooterXbox.povRight().getAsBoolean())
        .onTrue(m_shooter.increaseLimelightPowerForLastShotCommand());
    new Trigger(() -> isCurrentShooterProfile() && shooterXbox.povDown().getAsBoolean())
        .onTrue(m_shooter.confirmMadeShotCommand());
    new Trigger(m_shooter::isFeedbackWindowOpen)
        .whileTrue(Commands.startEnd(
            () -> shooterXbox.getHID().setRumble(RumbleType.kBothRumble, 1.0),
            () -> shooterXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0))
            .ignoringDisable(true));
    
    m_turret.setDefaultCommand(
        m_superstructure.manualTurretControl(() -> {
          if (isAltShooterProfile()) {
            boolean left = shooterXbox.leftBumper().getAsBoolean();
            boolean right = shooterXbox.rightBumper().getAsBoolean();
            if (left == right) {
              return 0.0;
            }
            return left ? -ALT_TURRET_MANUAL_SPEED : ALT_TURRET_MANUAL_SPEED;
          }
          return shooterXbox.getRightX() * 0.35;
        }));

    configureIntakeKeys();

    new Trigger(() -> isCurrentShooterProfile() && shooterXbox.x().getAsBoolean())
        .whileTrue(m_hopper.backFeedCommand());

    //shooterXbox.leftBumper().whileTrue(m_climber.climbDownCommand());
    //shooterXbox.leftTrigger(0.1).whileTrue(m_climber.climbUpCommand());

//got rid of pov buttons because they like to break too often, might fix later for more precise aiming if we dont get limelight in ti
    Trigger shooterSpinupTrigger = new Trigger(() -> isCurrentShooterProfile() && shooterXbox.getRightTriggerAxis() > 0.10);
    shooterSpinupTrigger.whileTrue(m_shooter.spinUp());
    shooterSpinupTrigger.onFalse(m_shooter.stop());
    new Trigger(() -> isAltShooterProfile() && shooterXbox.getRightTriggerAxis() > 0.10)
        .whileTrue(m_shooter.spinUp());
    new Trigger(() -> isAltShooterProfile() && shooterXbox.getLeftTriggerAxis() > 0.10)
        .whileTrue(Commands.parallel(
            m_hopper.feedCommand().asProxy(),
            m_kicker.feedCommand().asProxy()));
    new Trigger(() -> isAltShooterProfile() && shooterXbox.y().getAsBoolean())
        .whileTrue(m_hopper.backFeedCommand());

    }

  /**
   * Every control that commands the intake, behind one switch.
   *
   * <p>While {@link Constants.IntakeConstants#ENABLED} is false the bindings below are never
   * registered at all — the buttons simply do nothing for the intake — and the subsystem is parked
   * on a default command that actively drives both motors to zero. Set that constant back to true
   * to restore all of this exactly as it was.
   */
  private void configureIntakeKeys()
  {
    if (!Constants.IntakeConstants.ENABLED) {
      m_intake.setDefaultCommand(m_intake.disabledCommand());
      DriverStation.reportWarning(
          "[INTAKE] Intake is DISABLED (Constants.IntakeConstants.ENABLED = false). "
              + "Pivot and roller are held at zero and all intake buttons are inactive.",
          false);
      return;
    }

    m_intake.setDefaultCommand(
        m_intake.manualPivot(() -> shooterXbox.getLeftY()));

    new Trigger(() -> isCurrentShooterProfile() && shooterXbox.b().getAsBoolean())
        .whileTrue(m_superstructure.intakeCommand());

    new Trigger(() -> isAltShooterProfile() && shooterXbox.a().getAsBoolean())
        .whileTrue(m_superstructure.intakeCommand());

    new Trigger(() -> isAltShooterProfile() && shooterXbox.b().getAsBoolean())
        .whileTrue(m_intake.backFeedAndRollCommand());
  }
  
  /**
   * Vision wiring.
   *
   * <p>Currently nothing drives a mechanism from the camera. {@link AprilTagCamera} is registered
   * as a subsystem, so its {@code periodic()} reads tags and publishes telemetry every loop with no
   * button press needed — open the {@code AprilTag/} keys on the dashboard to see what it sees.
   *
   * <p>Note what is NOT here: no pipeline selection and no tag ID filter pushed to the camera.
   * Both of those are hardware-side state that persists across reboots, and both previously made a
   * healthy camera look blind. Camera configuration belongs in the Limelight web UI.
   */
  public void configureLimeLightKeys()
  {
    // Driver POV-RIGHT dumps every key the camera publishes to the console. Works while disabled,
    // which is when you actually want it: park the robot in front of a tag, press it, read the
    // output. POV-right is used because every face button on this controller is already taken.
    driverXbox.povRight().onTrue(m_camera.dumpDiagnosticsCommand());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    new Trigger(DriverStation::isEnabled)
        .onTrue(drivebase.centerModulesCommand().withTimeout(0.25));

    Command driveRobotRelativeAngularVelocity = Commands.run(
        () -> drivebase.drive(getScaledRobotRelativeDrive()),
        drivebase);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveRobotRelativeAngularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveRobotRelativeAngularVelocity);

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.rightStick().whileTrue(driveRobotRelativeAngularVelocity);
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.back().onTrue(
          Commands.runOnce(drivebase::printAbsoluteEncoderCalibration).ignoringDisable(true));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      
    }

    // Vision drivebase bindings live in configureLimeLightKeys(), called from the constructor.
    // configureLimelightBindings() used to bind driverXbox.rightBumper() and .start() as well,
    // which double-bound both buttons: two whileTrue commands, both requiring the drivebase, would
    // be scheduled together and immediately interrupt each other. Removed in favour of the single
    // set in configureLimeLightKeys().
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void updateShuffleboard()
  {
    m_shuffleboard.update();
  }

  /**
   * Brings up the driver camera, or returns null while it is switched off.
   *
   * <p>TEMPORARILY DISABLED via {@link #DRIVER_CAMERA_ENABLED}. Starting an automatic capture
   * claims a USB device, spawns an encoder thread and streams continuously over the radio, so
   * turning it off is worth real CPU and bandwidth while debugging something else. Flip the flag
   * back to true to restore it; the Shuffleboard widget reappears with it.
   */
  private UsbCamera startDriverCamera()
  {
    if (!DRIVER_CAMERA_ENABLED) {
      System.out.println("[CAMERA] Driver camera DISABLED (RobotContainer.DRIVER_CAMERA_ENABLED).");
      return null;
    }

    UsbCamera camera = CameraServer.startAutomaticCapture("DriverCam", 0);
    camera.setResolution(320, 240);
    camera.setFPS(20);
    return camera;
  }

  private ChassisSpeeds getScaledRobotRelativeDrive()
  {
    double maxLinearSpeed = drivebase.getSwerveDrive().getMaximumChassisVelocity();
    double maxAngularSpeed = drivebase.getSwerveDrive().getMaximumChassisAngularVelocity();
    double selectedLinearSpeed = Math.min(m_shuffleboard.getSelectedDriveSpeedMetersPerSecond(), maxLinearSpeed);
    double driveScale = maxLinearSpeed > 0.0 ? selectedLinearSpeed / maxLinearSpeed : 0.0;

    double forward = -driverXbox.getLeftY() * driveScale * maxLinearSpeed;
    double strafeLeft = -driverXbox.getLeftX() * driveScale * maxLinearSpeed;
    double rotateCounterClockwise = -driverXbox.getRightX() * driveScale * maxAngularSpeed;

    return new ChassisSpeeds(
        forward,
        strafeLeft,
        rotateCounterClockwise);
  }

  public void applySelectedStartPose()
  {
    drivebase.resetOdometry(m_shuffleboard.getSelectedStartPosition().getPose());
  }

  private boolean isCurrentShooterProfile()
  {
    return m_shuffleboard.getSelectedControlProfile() == ShooterControlProfile.CURRENT;
  }

  private boolean isAltShooterProfile()
  {
    return m_shuffleboard.getSelectedControlProfile() == ShooterControlProfile.ALT_TWO;
  }
}
