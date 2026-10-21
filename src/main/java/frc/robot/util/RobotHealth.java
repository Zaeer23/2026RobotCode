package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/** Local evidence for distinguishing DS link loss, supply brownouts and delayed robot loops. */
public final class RobotHealth {
  private final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("Dashboard/Health");
  private final Throttle publishRate = new Throttle(Throttle.SLOW_HZ);
  private final DoubleLogEntry batteryLog;
  private final DoubleLogEntry loopGapLog;
  private final BooleanLogEntry brownoutLog;
  private boolean lastDsAttached;
  private boolean lastBrownout;
  private long dsDisconnects;
  private long brownouts;
  private long delayedLoops;
  private double lastLoopSeconds = -1;
  private double minimumBatteryVolts = Double.POSITIVE_INFINITY;
  private double maximumLoopGapMs;

  public RobotHealth() {
    // Keep diagnostics on the roboRIO, including while the radio link is unavailable.
    // File writes run on WPILib's logging thread. Avoid recording every NT topic.
    DataLogManager.logNetworkTables(false);
    DataLogManager.start();
    var log = DataLogManager.getLog();
    DriverStation.startDataLog(log, false);
    batteryLog = new DoubleLogEntry(log, "/Health/BatteryVolts");
    loopGapLog = new DoubleLogEntry(log, "/Health/LoopGapMs");
    brownoutLog = new BooleanLogEntry(log, "/Health/Brownout");
  }

  /** Sample at the beginning of every robot loop so stalls appear in the following sample. */
  public void update() {
    double now = Timer.getFPGATimestamp();
    double voltage = RobotController.getBatteryVoltage();
    boolean dsAttached = DriverStation.isDSAttached();
    boolean brownout = RobotController.isBrownedOut();
    double loopGapMs = lastLoopSeconds < 0 ? 0 : (now - lastLoopSeconds) * 1000;
    lastLoopSeconds = now;

    if (lastDsAttached && !dsAttached) {
      dsDisconnects++;
    }
    if (brownout && !lastBrownout) {
      brownouts++;
    }
    if (loopGapMs > 40) {
      delayedLoops++;
    }
    lastDsAttached = dsAttached;
    lastBrownout = brownout;
    minimumBatteryVolts = Math.min(minimumBatteryVolts, voltage);
    maximumLoopGapMs = Math.max(maximumLoopGapMs, loopGapMs);
    batteryLog.append(voltage);
    loopGapLog.append(loopGapMs);
    brownoutLog.update(brownout);

    if (publishRate.ready()) {
      table.getEntry("DSAttached").setBoolean(dsAttached);
      table.getEntry("DSDisconnects").setInteger(dsDisconnects);
      table.getEntry("Brownouts").setInteger(brownouts);
      table.getEntry("DelayedLoops").setInteger(delayedLoops);
      table.getEntry("MinimumBatteryVolts").setDouble(minimumBatteryVolts);
      table.getEntry("MaximumLoopGapMs").setDouble(maximumLoopGapMs);
      table.getEntry("UptimeSeconds").setDouble(now);
    }
  }
}
