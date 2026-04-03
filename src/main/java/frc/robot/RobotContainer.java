// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LimeLightRunner;
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
  private static final double[] DRIVE_SPEED_PRESETS = {0.35, 0.65, 1.0};

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController shooterXbox = new CommandXboxController(1);
  private int currentDriveSpeedIndex = 1;
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
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
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
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


// Create the Superstructure instance
private final Superstructure m_superstructure = new Superstructure(
    m_shooter, m_turret, m_hood, m_intake, m_hopper, m_kicker, m_limelight
);

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    configureSubSystemKeys();
    configureLimeLightKeys();
  }

  public void configureSubSystemKeys()
  {
    shooterXbox.a().whileTrue(m_superstructure.hopperFeedCommand());
    shooterXbox.y().whileTrue(m_turret.trackTarget(m_limelight, drivebase));
    driverXbox.y().whileTrue(m_limelight.alignCommand(drivebase));
    shooterXbox.povDown().whileTrue(m_superstructure.kickerFeedCommand());
    
    m_turret.setDefaultCommand(
    m_superstructure.manualTurretControl(() -> {
        double input = shooterXbox.getRightX();
        if (Math.abs(input) < 0.1) return 0.0;
        return Math.copySign(input * input, input);
    })
);

//  -------------------- AUTONOMOUS COMMANDS --------------------
   NamedCommands.registerCommand("HungryHungryIntake",
    Commands.parallel(
        m_superstructure.intakeCommand()
    )
);
NamedCommands.registerCommand("DeployIntake",
    Commands.parallel(
        m_superstructure.setIntakePivotAngle(Degrees.of(115))
    )
);
NamedCommands.registerCommand("shoot",
Commands.parallel(
  
m_shooter.spinUp()
    ) 
);
NamedCommands.registerCommand("feed",
Commands.parallel(
  m_kicker.feedCommand(),
m_hopper.feedCommand()
));

NamedCommands.registerCommand("STOP",
Commands.parallel(
      m_shooter.stop(),
      m_kicker.stopCommand(),
      m_hopper.stopCommand()
    )
);
NamedCommands.registerCommand("LimelightTurret", 
Commands.parallel(
      m_turret.trackTarget(m_limelight)
    ) 
);  
NamedCommands.registerCommand("ClimbBarAlign",
    m_superstructure.climbBarAlignCommand(drivebase).withTimeout(2.5)
);
NamedCommands.registerCommand("ClimbBarAlignAndClimb",
    Commands.sequence(
        m_superstructure.climbBarAlignCommand(drivebase).withTimeout(2.5),
        m_climber.climbUpCommand().withTimeout(2.5)
    )
);
NamedCommands.registerCommand("ClimbDown",
 Commands.sequence(
  m_climber.climbDownCommand()
 ));
 NamedCommands.registerCommand("ClimbUp",
 Commands.sequence(
  m_climber.climbUpCommand()
 ));

// ------------------ END OF AUTONOMOUS COMMANDS --------------------

m_intake.setDefaultCommand(
    m_intake.manualPivot(() -> 
    shooterXbox.getLeftY())
);
    shooterXbox.x().whileTrue(m_hopper.backFeedCommand());

    shooterXbox.rightBumper().whileTrue(
    m_shooter.setSpeedFromLimelight(m_limelight, 45.0)
);
    //shooterXbox.leftBumper().whileTrue(m_climber.climbDownCommand());
    //shooterXbox.leftTrigger(0.1).whileTrue(m_climber.climbUpCommand());

//got rid of pov buttons because they like to break too often, might fix later for more precise aiming if we dont get limelight in ti
    new Trigger(() -> shooterXbox.getRightTriggerAxis() > 0.10)
    .whileTrue(m_shooter.setPercentAsRPM(
        () -> shooterXbox.getRightTriggerAxis()
    ));
shooterXbox.b().whileTrue(m_intake.intakeCommand());

    }
  
  public void configureLimeLightKeys()
  {
    LimeLightRunner runner = new LimeLightRunner(m_limelight, drivebase, () -> driverXbox.getLeftY());
    runner.execute();
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
    Command driveRobotRelativeAngularVelocity = Commands.run(
        () -> drivebase.drive(getScaledRobotRelativeDrive()),
        drivebase);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      // Without reliable absolute module references, robot-oriented driving is the safer default.
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
      drivebase.setDefaultCommand(driveRobotRelativeAngularVelocity); // Keep driver controls robot-relative in test too.

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
      driverXbox.rightStick().whileTrue(driveRobotRelativeAngularVelocity);
      driverXbox.povUp().onTrue(Commands.runOnce(this::increaseDriveSpeedPreset));
      driverXbox.povDown().onTrue(Commands.runOnce(this::decreaseDriveSpeedPreset));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      
    }

    configureLimelightBindings();
      
  }

  // right now it's manual but plan to automate it
  public void configureLimelightBindings()
  {
    driverXbox.rightBumper().whileTrue(
      m_superstructure.limelightAlignCommand(drivebase)
    );
    driverXbox.start().whileTrue(
      m_superstructure.climbBarAlignCommand(drivebase)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  private ChassisSpeeds getScaledRobotRelativeDrive()
  {
    ChassisSpeeds requestedSpeeds = driveRobotOriented.get();
    double driveScale = DRIVE_SPEED_PRESETS[currentDriveSpeedIndex];
    return new ChassisSpeeds(
        requestedSpeeds.vxMetersPerSecond * driveScale,
        requestedSpeeds.vyMetersPerSecond * driveScale,
        requestedSpeeds.omegaRadiansPerSecond * driveScale);
  }

  private void increaseDriveSpeedPreset()
  {
    currentDriveSpeedIndex = Math.min(currentDriveSpeedIndex + 1, DRIVE_SPEED_PRESETS.length - 1);
  }

  private void decreaseDriveSpeedPreset()
  {
    currentDriveSpeedIndex = Math.max(currentDriveSpeedIndex - 1, 0);
  }
}
