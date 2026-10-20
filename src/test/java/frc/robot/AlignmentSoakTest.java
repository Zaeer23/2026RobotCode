package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.OutputStream;
import java.io.PrintStream;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;

/**
 * Opt-in integration diagnostic: 200 seconds idle, then ten minutes with L2 held and a moving tag.
 * Runs the actual container, native HAL, scheduler, and both dashboard update paths. It deliberately
 * does not start Robot.robotInit(), so camera TCP forwarding is not part of this alignment check.
 *
 * <p>Run alone from the repository root in PowerShell (JAVA_HOME must select the WPILib JDK):
 * <pre>
 * $env:RUN_ALIGNMENT_SOAK = 'true'
 * .\gradlew.bat test --tests frc.robot.AlignmentSoakTest --rerun-tasks
 * Remove-Item Env:RUN_ALIGNMENT_SOAK
 * </pre>
 *
 * <p>To repeat the constrained-heap diagnostic, save this Gradle init script anywhere and append
 * {@code --init-script <path-to-that-file>} to the command above:
 * <pre>
 * allprojects {
 *     tasks.withType(Test).configureEach {
 *         maxHeapSize = '64m'
 *         jvmArgs '-XX:+UseSerialGC'
 *     }
 * }
 * </pre>
 * The output reports retained heap after GC and topic counts every 100 simulated seconds. This
 * does not measure native memory or reproduce roboRIO network bandwidth and hardware behavior.
 */
class AlignmentSoakTest {
  @Test
  @EnabledIfEnvironmentVariable(named = "RUN_ALIGNMENT_SOAK", matches = "true")
  void sustainedL2WithValidLimelightTarget() throws ReflectiveOperationException {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    for (int port = 0; port < 2; port++) {
      DriverStationSim.setJoystickAxisCount(port, 6);
      DriverStationSim.setJoystickButtonCount(port, 10);
      DriverStationSim.setJoystickPOVCount(port, 1);
      DriverStationSim.setJoystickPOV(port, 0, -1);
      DriverStationSim.setJoystickIsXbox(port, true);
    }
    DriverStationSim.notifyNewData();
    DriverStation.silenceJoystickConnectionWarning(true);
    RobotContainer container = new RobotContainer();
    Field drivebaseField = RobotContainer.class.getDeclaredField("drivebase");
    drivebaseField.setAccessible(true);
    SwerveSubsystem drivebase = (SwerveSubsystem) drivebaseField.get(container);
    NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight");
    camera.getEntry("tv").setDouble(1.0);
    camera.getEntry("getpipe").setDouble(0.0);
    camera.getEntry("rawfiducials").setDoubleArray(new double[] {10, 8, 10, 0.5, 4, 4, 0.1});
    CommandScheduler scheduler = CommandScheduler.getInstance();
    PrintStream original = System.out;
    long initialRetainedBytes = 0;
    int alignmentTopics = 0;
    try (PrintStream quiet = new PrintStream(OutputStream.nullOutputStream())) {
      System.setOut(quiet);
      SimHooks.pauseTiming();
      for (int loop = 0; loop < 40_000; loop++) {
        DriverStationSim.setJoystickAxis(0, 2, loop >= 10_000 ? 1.0 : 0.0);
        camera.getEntry("hb").setDouble(loop);
        camera.getEntry("rawfiducials").setDoubleArray(new double[] {
            10, 8.0 + 4.0 * Math.sin(loop * 0.03), 10.0 + Math.sin(loop * 0.02),
            0.5, 4.0 + Math.sin(loop * 0.01), 4.0, 0.1});
        DriverStationSim.notifyNewData();
        SimHooks.stepTiming(0.02);
        scheduler.run();
        container.updateShuffleboard();
        SmartDashboard.updateValues();
        Shuffleboard.update();
        if (loop % 5_000 == 0) {
          System.gc();
          Runtime rt = Runtime.getRuntime();
          long retainedBytes = rt.totalMemory() - rt.freeMemory();
          int topicCount = NetworkTableInstance.getDefault().getTopics().length;
          if (loop == 5_000) {
            initialRetainedBytes = retainedBytes;
          } else if (loop >= 15_000) {
            assertTrue(retainedBytes - initialRetainedBytes < 16 * 1024 * 1024,
                "Retained Java heap must stay bounded while L2 remains held");
            if (alignmentTopics == 0) {
              alignmentTopics = topicCount;
            }
            assertEquals(alignmentTopics, topicCount,
                "Holding L2 must not create an increasing number of telemetry topics");
          }
          original.printf("SOAK cycle=%d phase=%s retainedHeapMiB=%.1f heapMaxMiB=%.1f topics=%d%n", loop,
              loop >= 10_000 ? "L2" : "idle", retainedBytes / 1048576.0,
              rt.maxMemory() / 1048576.0, topicCount);
        }
        if (loop == 10_010) {
          assertEquals("TURNING", NetworkTableInstance.getDefault()
              .getTable("SmartDashboard").getEntry("DriveAlign/State").getString(""));
        }
      }
    } finally {
      System.setOut(original);
      scheduler.cancelAll();
      scheduler.unregisterAllSubsystems();
      scheduler.getDefaultButtonLoop().clear();
      drivebase.getSwerveDrive().stopOdometryThread();
      DriverStationSim.setJoystickAxis(0, 2, 0.0);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      SimHooks.resumeTiming();
    }
  }
}
