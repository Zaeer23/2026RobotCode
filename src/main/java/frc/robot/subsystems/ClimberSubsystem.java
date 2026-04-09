package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private static final double CLIMBER_FULL_TRAVEL_SECONDS = 1.0;

    private final SparkMax climberSparkMax = new SparkMax(
        Constants.ClimberConstants.kClimberMotorId, MotorType.kBrushless
    );

    public ClimberSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake); // Brake when idle — won't slip on bar
        config.smartCurrentLimit(40);
        config.openLoopRampRate(0.35);
        config.closedLoopRampRate(0.35);
        climberSparkMax.configure(config,
        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        
    }

    /** Runs the climber upward at full speed. */
    public void climbUp() {
        climberSparkMax.set(Constants.ClimberConstants.kClimberSpeed);
    }

    /** Runs the climber downward. */
    public void climbDown() {
        climberSparkMax.set(-Constants.ClimberConstants.kClimberSpeed);
    }

    /** Stops the climber (motor will brake due to IdleMode.kBrake). */
    public void stop() {
        climberSparkMax.set(0);
    }

    // --- Command factories ---

    public Command climbUpCommand() {
        return this.startEnd(this::climbUp, this::stop).withName("ClimbUp");
    }

    public Command climbDownCommand() {
        return this.startEnd(this::climbDown, this::stop).withName("ClimbDown");
    }

    public Command climbUpToTopCommand() {
        return this.run(this::climbUp)
            .withTimeout(CLIMBER_FULL_TRAVEL_SECONDS)
            .finallyDo(() -> stop())
            .withName("ClimbUpToTop");
    }

    public Command climbDownToBottomCommand() {
        return this.run(this::climbDown)
            .withTimeout(CLIMBER_FULL_TRAVEL_SECONDS)
            .finallyDo(() -> stop())
            .withName("ClimbDownToBottom");
    }
}
