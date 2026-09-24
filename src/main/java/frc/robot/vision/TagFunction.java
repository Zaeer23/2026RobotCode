package frc.robot.vision;

import edu.wpi.first.math.util.Units;

/**
 * What a 2026 REBUILT AprilTag is attached to.
 *
 * <p>The four structures sit at three different heights, which is the reason this enum exists at
 * all. Range to a tag is found by walking the camera ray out to the tag's height plane, so reading
 * a TRENCH tag while assuming HUB height scales the whole distance measurement. Knowing what you
 * are looking at is a prerequisite for knowing how far away it is.
 */
public enum TagFunction {

  /** Scoring goal. This is what the shooter aims at. Two tags on each of four faces. */
  HUB(44.25),

  /** Trench structure spanning the field. */
  TRENCH(35.0),

  /** Climb structure at the alliance wall. */
  TOWER(21.75),

  /** Game-piece loading station at the alliance wall. */
  OUTPOST(21.75),

  /** Reported ID is not part of the 2026 field. */
  UNKNOWN(0.0);

  private final double heightInches;

  TagFunction(double heightInches) {
    this.heightInches = heightInches;
  }

  /** Height of the tag's centre above the carpet, in inches. */
  public double heightInches() {
    return heightInches;
  }

  /** Height of the tag's centre above the carpet, in meters. */
  public double heightMeters() {
    return Units.inchesToMeters(heightInches);
  }

  /** True for a structure this robot can actually score in. */
  public boolean isScoringTarget() {
    return this == HUB;
  }
}
