package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class LimeLightRunner extends Command {

    private final LimeLight limelight;
    private final SwerveSubsystem swerve;

    private final DoubleSupplier forwardJoystick;

    // class variables useful to tweak/tune
    private final double TURN_kP = 0.03;
    private final double DRIVE_kP = 0.4;
    
    // adjust for distance
    private final double DESIRED_AREA = 2.0;

    public LimeLightRunner(LimeLight limelight, SwerveSubsystem swerve, DoubleSupplier forwardJoystick)
    {
        this.limelight = limelight;
        this.swerve = swerve;
        this.forwardJoystick = forwardJoystick;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        limelight.lightMode(4);
        if (!limelight.hasTarget()) {
            swerve.drive(new Translation2d(0,0),0.0,false);
            return;
        }

        double turn = limelight.getTurnCorrection(TURN_kP);

        double autoForward = limelight.getForwardCorrection(DESIRED_AREA, DRIVE_kP);

        double joystickForward = forwardJoystick.getAsDouble();

        double finalForward = autoForward + joystickForward * 0.5;

        // may need to adjust depending on SwerveSubsystem drive method
        swerve.drive(new Translation2d(finalForward, 0),   turn,   true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
