package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/*

  File Destinatoin:    frc/robot/subsystems/vision/LimeLight.java

  Command Execution:  frc/robot/commands/vision/LimeLightRunner.java

*/
public class LimeLight {

    private final NetworkTable table;

    public LimeLight(String name) {
        table = NetworkTableInstance.getDefault().getTable(name);
    }

    //some bic data access thingymabobies

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public double getTX() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getTY() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getTA() {
        return table.getEntry("ta").getDouble(0);
    }

    public int getTagID() {
        return (int) table.getEntry("tid").getDouble(-1);
    }

    public void lightMode(int number) {
        table.getEntry("ledMode").setNumber(number);
    }
    // this is just optional helpers

    // Positive tx means target is to the right
    public double getTurnCorrection(double kP) {
        return -getTX() * kP;
    }

    // Example forward correction using area
    public double getForwardCorrection(double desiredArea, double kP) {
        return (desiredArea - getTA()) * kP;
    }
}