package frc.robot.cyberpsycho;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.DashboardTelemetry;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.vision.Limelight;
import java.io.OutputStream;
import java.io.PrintStream;
import java.lang.reflect.Field;
import swervelib.SwerveModule;
import org.junit.jupiter.api.Test;

/**
 * Enters the code on a simulated D-pad against the real {@link RobotContainer} and checks what
 * CyberPsycho promises: the mechanisms are locked out and stay locked out through everything that
 * normally clears commands, the drive is unleashed, the camera is camera-only, and nothing turns
 * it off again.
 *
 * <p>Runs the native HAL and the actual scheduler, the same way {@code AlignmentSoakTest} does,
 * but for a few simulated seconds rather than minutes, so it is part of the normal test run.
 */
public class CyberPsychoIntegrationTest {

  private static final int DRIVER = 0;
  private static final int OPERATOR = 1;

  private static final int UP = 0;
  private static final int RIGHT = 90;
  private static final int DOWN = 180;
  private static final int LEFT = 270;
  private static final int CENTRE = -1;

  private static final int AXIS_RIGHT_TRIGGER = 3;

  /** One tag in view the whole time, the same frame AlignmentSoakTest feeds. */
  private static final double[] TAG_IN_VIEW = {10, 8, 10, 0.5, 4, 4, 0.1};

  private RobotContainer container;
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private long heartbeat = 0;

  @SuppressWarnings("unchecked")
  private <T> T field(String name) throws ReflectiveOperationException {
    Field field = RobotContainer.class.getDeclaredField(name);
    field.setAccessible(true);
    return (T) field.get(container);
  }

  /** One robot loop, in the order Robot.robotPeriodic runs them, with the camera publishing. */
  private void loops(int count) {
    for (int i = 0; i < count; i++) {
      limelight.getEntry("tv").setDouble(1.0);
      limelight.getEntry("hb").setDouble(heartbeat++);
      limelight.getEntry("rawfiducials").setDoubleArray(TAG_IN_VIEW);
      DriverStationSim.notifyNewData();
      SimHooks.stepTiming(0.02);
      container.runCyberPsycho();
      scheduler.run();
    }
  }

  private void enterCode(int port) {
    for (int pov : new int[] {UP, UP, DOWN, DOWN, LEFT, RIGHT, LEFT, RIGHT}) {
      DriverStationSim.setJoystickPOV(port, 0, pov);
      loops(4);
      DriverStationSim.setJoystickPOV(port, 0, CENTRE);
      loops(4);
    }
  }

  private String holder(Subsystem subsystem) {
    Command command = scheduler.requiring(subsystem);
    return command == null ? "nothing" : command.getName();
  }

  private static boolean dashboardSaysActive() {
    return NetworkTableInstance.getDefault().getTable(DashboardTelemetry.TABLE)
        .getEntry("CyberPsycho/Active").getBoolean(false);
  }

  @Test
  void theCodeLocksEverythingButTheWheelsAndNeverLetsGo() throws ReflectiveOperationException {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    for (int port = 0; port < 2; port++) {
      DriverStationSim.setJoystickAxisCount(port, 6);
      DriverStationSim.setJoystickButtonCount(port, 10);
      DriverStationSim.setJoystickPOVCount(port, 1);
      DriverStationSim.setJoystickPOV(port, 0, CENTRE);
      DriverStationSim.setJoystickIsXbox(port, true);
    }
    DriverStationSim.notifyNewData();
    DriverStation.silenceJoystickConnectionWarning(true);
    SimHooks.pauseTiming();

    PrintStream original = System.out;
    SwerveSubsystem drivebase = null;
    try (PrintStream quiet = new PrintStream(OutputStream.nullOutputStream())) {
      System.setOut(quiet);
      container = new RobotContainer();
      CyberPsycho cyber = field("m_cyberPsycho");
      drivebase = field("drivebase");
      ShooterSubsystem shooter = field("m_shooter");
      TurretSubsystem turret = field("m_turret");
      Limelight camera = field("m_camera");

      double normalTop = drivebase.getSwerveDrive().getMaximumChassisVelocity();
      // The NEO's free speed is rated at 12 V; the cap is set for a full battery.
      double motorTop = drivebase.getSwerveDrive().getMaximumModuleDriveVelocity()
          * CyberPsycho.FULL_BATTERY_VOLTS / 12.0;
      assertTrue(motorTop > normalTop,
          "the motors' free speed should be above the normal cap, or there is nothing to unleash");

      loops(10);
      assertFalse(cyber.isActive());
      assertFalse(dashboardSaysActive());
      assertTrue(camera.isConnected());
      assertFalse(camera.getVisibleTargets().isEmpty(), "camera reads the tag before the mode");
      assertEquals("Superstructure.aimTurret", holder(turret), "turret tracks by default");

      // Half the code, then a pause, does nothing.
      DriverStationSim.setJoystickPOV(OPERATOR, 0, UP);
      loops(4);
      DriverStationSim.setJoystickPOV(OPERATOR, 0, CENTRE);
      loops(200);
      assertFalse(cyber.isActive());

      enterCode(OPERATOR);
      assertTrue(cyber.isActive());
      assertTrue(dashboardSaysActive());
      assertEquals("CyberPsycho.Lockout", holder(shooter));
      assertEquals("CyberPsycho.Lockout", holder(turret));
      assertEquals(motorTop, drivebase.getSwerveDrive().getMaximumChassisVelocity(), 1e-9);
      for (SwerveModule module : drivebase.getSwerveDrive().getModules()) {
        SparkMax drive = (SparkMax) module.getDriveMotor().getMotor();
        assertEquals(CyberPsycho.DRIVE_CURRENT_LIMIT_AMPS,
            drive.configAccessor.getSmartCurrentLimit(), "drive current limit raised");
        assertFalse(drive.configAccessor.getVoltageCompensationEnabled(),
            "voltage compensation off, so a full battery reaches the wheels");
        assertEquals(0.0, drive.configAccessor.getClosedLoopRampRate(), 1e-9);
        assertEquals(0.0, drive.configAccessor.getOpenLoopRampRate(), 1e-9);
      }
      assertTrue(camera.isCameraOnly());
      loops(1);
      assertEquals("CAMERA ONLY", camera.getStatus());
      assertTrue(camera.getVisibleTargets().isEmpty(), "tag still in view, but no longer read");
      assertFalse(camera.hasTarget());
      assertEquals(LimeLightConstants.DRIVER_CAMERA_PIPELINE,
          limelight.getEntry("pipeline").getInteger(-1), "camera switched to the driver pipeline");

      // Mechanism buttons are refused: operator R2 (shoot), driver POV-left (turret test).
      DriverStationSim.setJoystickAxis(OPERATOR, AXIS_RIGHT_TRIGGER, 1.0);
      DriverStationSim.setJoystickPOV(DRIVER, 0, LEFT);
      loops(10);
      assertEquals("CyberPsycho.Lockout", holder(shooter));
      assertEquals("CyberPsycho.Lockout", holder(turret));
      DriverStationSim.setJoystickAxis(OPERATOR, AXIS_RIGHT_TRIGGER, 0.0);
      DriverStationSim.setJoystickPOV(DRIVER, 0, CENTRE);
      loops(2);

      // teleopInit cancels everything when no auto ran. The lockout comes straight back.
      scheduler.cancelAll();
      loops(1);
      assertEquals("CyberPsycho.Lockout", holder(shooter));

      // Disabling does not end it either.
      DriverStationSim.setEnabled(false);
      loops(50);
      DriverStationSim.setEnabled(true);
      loops(5);
      assertTrue(cyber.isActive());
      assertEquals("CyberPsycho.Lockout", holder(turret));
      assertEquals(motorTop, drivebase.getSwerveDrive().getMaximumChassisVelocity(), 1e-9);

      // Entering the code again is not a way out.
      enterCode(DRIVER);
      assertTrue(cyber.isActive());
      assertTrue(dashboardSaysActive());

      // AUTO says plainly that it is sitting out rather than having its routine refused.
      assertEquals("CyberPsycho (no auto)", container.getAutonomousCommand().getName());
      assertNotEquals("nothing", holder(shooter));
    } finally {
      System.setOut(original);
      scheduler.cancelAll();
      scheduler.unregisterAllSubsystems();
      scheduler.getDefaultButtonLoop().clear();
      if (drivebase != null) {
        drivebase.getSwerveDrive().stopOdometryThread();
      }
      for (int port = 0; port < 2; port++) {
        DriverStationSim.setJoystickPOV(port, 0, CENTRE);
        DriverStationSim.setJoystickAxis(port, AXIS_RIGHT_TRIGGER, 0.0);
      }
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      SimHooks.resumeTiming();
    }
  }
}
