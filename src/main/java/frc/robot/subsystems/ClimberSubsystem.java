package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber on CAN ID 36.
 *
 * <p><b>There is no climber on this robot and nothing is wired to ID 36.</b> The whole class is
 * gated behind {@link Constants.ClimberConstants#PRESENT}, which is false, so no SparkMax is ever
 * constructed and ID 36 never appears on the CAN bus at all.
 *
 * <p>This matters because the warnings are not something the robot can be told to ignore. A
 * SparkMax object is what puts the ID on the bus: constructing one makes REVLib start polling that
 * ID, and {@code configure()} on an ID with no device behind it retries and then reports a
 * timeout, every loop, forever. There is no "mark this device optional" switch in REVLib. The only
 * way to silence it is not to open it, which is what the flag does.
 *
 * <p>Every method still exists and is safe to call — they just do nothing while the flag is false.
 * That is deliberate: a caller that binds a climber command gets a no-op rather than a
 * NullPointerException in the middle of a match.
 *
 * <p>To bring a real climber online: wire it, set {@code PRESENT = true}, and re-add the subsystem
 * to RobotContainer.
 */
public class ClimberSubsystem extends SubsystemBase {
    private static final double CLIMBER_FULL_TRAVEL_SECONDS = 1.0;

    /** Null whenever the climber is not present, which is the entire point. */
    private final SparkMax climberSparkMax;

    public ClimberSubsystem() {
        if (!Constants.ClimberConstants.PRESENT) {
            climberSparkMax = null;
            System.out.println(
                "[CLIMBER] Not present. CAN ID " + Constants.ClimberConstants.kClimberMotorId
                    + " is deliberately not opened, so it produces no CAN traffic and no faults.");
            return;
        }

        climberSparkMax = new SparkMax(
            Constants.ClimberConstants.kClimberMotorId, MotorType.kBrushless
        );
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake); // Brake when idle — won't slip on bar
        config.smartCurrentLimit(40);
        config.openLoopRampRate(0.35);
        config.closedLoopRampRate(0.35);
        climberSparkMax.configure(config,
        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
    }

    /** True only when there is real hardware behind CAN ID 36. */
    public boolean isPresent() {
        return climberSparkMax != null;
    }

    /** Runs the climber upward at full speed. Does nothing when the climber is not present. */
    public void climbUp() {
        if (climberSparkMax != null) {
            climberSparkMax.set(Constants.ClimberConstants.kClimberSpeed);
        }
    }

    /** Runs the climber downward. Does nothing when the climber is not present. */
    public void climbDown() {
        if (climberSparkMax != null) {
            climberSparkMax.set(-Constants.ClimberConstants.kClimberSpeed);
        }
    }

    /** Stops the climber (motor will brake due to IdleMode.kBrake). */
    public void stop() {
        if (climberSparkMax != null) {
            climberSparkMax.set(0);
        }
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
