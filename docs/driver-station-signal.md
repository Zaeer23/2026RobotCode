# Driver Station connection checks

The robot now shares one video budget across all Limelight forwarding routes: at most two
viewers and 1.5 Mbit/s of video payload in total. This limits a possible source of radio
congestion; it does not establish why the Driver Station disconnected. Direct connections to
the camera bypass the roboRIO budget. Reduce the Limelight stream resolution/frame rate in its
web interface if the picture lags, and close extra camera browser tabs.
Forwarded connections with no video write progress for five seconds are closed so an interrupted
viewer cannot occupy a connection slot indefinitely after radio loss.

After deploying, reproduce the issue with the robot supported safely, then inspect these
NetworkTables values under `Dashboard/Health` (updated twice per second):

- `DSDisconnects`: increments when the robot sees an attached Driver Station disappear.
- `Brownouts` and `MinimumBatteryVolts`: show supply interruptions during motor load.
- `DelayedLoops` and `MaximumLoopGapMs`: count gaps over 40 ms between robot loops.
- `UptimeSeconds`: drops after a roboRIO reboot; the counters also reset when code restarts.

The roboRIO also records `/Health/BatteryVolts`, `/Health/LoopGapMs`, `/Health/Brownout`, DS
control state, and console output to a local `.wpilog`. Download it with WPILib DataLog Tool
from the USB drive's `logs` directory, or `/home/lvuser/logs` when no drive is attached. The
samples remain available across radio loss; abrupt power loss can still lose the final buffered
samples. Compare its timestamps with the Driver Station log to separate a radio/ethernet
outage from a brownout or robot-code restart. A repeated dropout with healthy voltage and no
loop delay still needs inspection of radio power, Ethernet connections, and DS network settings.

References: [WPILib data logs](https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html)
and [camera bandwidth considerations](https://docs.wpilib.org/en/stable/docs/software/vision-processing/introduction/strategies-for-vision-programming.html).
