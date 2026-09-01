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
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.HubLineupCommand;
import frc.robot.commands.LimeLightRunner;
import frc.robot.commands.ShootingCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLight;
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
private final LimeLight m_limelight = new LimeLight("limelight");
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
    m_shooter, m_turret, m_hood, m_intake, m_hopper, m_kicker, m_limelight
);
private final ShuffleboardManager m_shuffleboard = new ShuffleboardManager(
    m_limelight, m_superstructure, drivebase, m_turret, m_driverCamera
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
    autoChooser.setDefaultOption(
        "Shoot + Collect + Shoot",
        AutoRoutines.shootCollectShoot(m_superstructure, m_turret, m_shooter, m_limelight, drivebase));
    autoChooser.addOption(
        "Shoot preload + leave",
        AutoRoutines.shootAndLeave(m_superstructure, m_turret, m_shooter, m_limelight, drivebase));
    autoChooser.addOption(
        "Shoot preload only",
        AutoRoutines.shootPreloadOnly(m_superstructure, m_turret, m_shooter, m_limelight, drivebase));
    autoChooser.addOption(
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

    // TRACK AND SHOOT — aim, spin, and feed when ready. Chassis stays with the driver.
    new Trigger(() -> isCurrentShooterProfile() && shooterXbox.y().getAsBoolean())
        .whileTrue(m_superstructure.trackAndShootCommand(drivebase));

    // LINEUP AND SHOOT — drive to range first, then the same. Borrows the drivebase only until the
    // lineup settles, then releases it back to the driver.
    new Trigger(() -> isCurrentShooterProfile() && shooterXbox.leftBumper().getAsBoolean())
        .whileTrue(m_superstructure.autoShootCommand(drivebase));

    // AIM ONLY — turret tracks and flywheel spins, but never feeds. For the operator who wants to
    // pick the exact moment to fire with the manual feed on A.
    new Trigger(() -> isCurrentShooterProfile() && shooterXbox.rightBumper().getAsBoolean())
        .whileTrue(m_superstructure.limelightShootCommand(drivebase));
    driverXbox.y().whileTrue(m_limelight.alignCommand(drivebase));
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
   * Vision-assisted drivebase moves.
   *
   * <p>Both are {@code whileTrue} on a HELD button, and neither is a default command. The drivebase
   * default stays plain driver control, so releasing the button always hands the robot straight
   * back to the driver — and the lineup ends by itself once it settles.
   *
   * <p>This method previously constructed a {@link LimeLightRunner} and called {@code execute()} on
   * it directly, exactly once, at startup. A Command has to be scheduled to run: calling execute()
   * by hand fires a single loop iteration during init, ignores the drivebase requirement it
   * declared, and then never runs again. {@link HubLineupCommand} supersedes it.
   */
  public void configureLimeLightKeys()
  {
    m_limelight.configureForAprilTags();

    // Auto line up on the hub: squares up and drives to the calibrated shooting range.
    driverXbox.rightBumper().whileTrue(m_superstructure.hubLineupCommand(drivebase));

    // Line up on the TOWER tags for the climb.
    driverXbox.start().whileTrue(m_superstructure.climbBarAlignCommand(drivebase));
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

  private UsbCamera startDriverCamera()
  {
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
