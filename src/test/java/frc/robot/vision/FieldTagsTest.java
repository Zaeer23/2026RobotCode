package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

/**
 * Locks the 2026 tag map against the official field layout.
 *
 * <p>Getting a tag into the wrong group is silent at runtime: detection still works and a distance
 * still comes out, it is just wrong by the height difference between two structures. These tests
 * are what turn that into a build failure.
 */
class FieldTagsTest {

  @Test
  void classLoadsWhichMeansItSelfVerifiedAgainstTheOfficialLayout() {
    // FieldTags cross-checks every group against the real layout in a static block, so simply
    // touching it here proves that check passed.
    assertEquals(32, FieldTags.LAYOUT.getTags().size(), "2026 REBUILT has 32 tags");
  }

  @Test
  void everyFieldTagIsGroupedExactlyOnce() {
    Set<Integer> seen = new HashSet<>();
    for (Set<Integer> group : Set.of(
        FieldTags.ALL_HUB, FieldTags.ALL_TRENCH, FieldTags.ALL_TOWER, FieldTags.ALL_OUTPOST)) {
      for (int id : group) {
        assertTrue(seen.add(id), "tag " + id + " appears in more than one group");
      }
    }
    assertEquals(32, seen.size(), "all 32 tags must be grouped");

    for (int id = 1; id <= 32; id++) {
      assertTrue(FieldTags.isFieldTag(id), "tag " + id + " is missing from the map");
    }
  }

  @Test
  void groupSizesMatchTheFieldDrawing() {
    assertEquals(8, FieldTags.RED_HUB.size());
    assertEquals(8, FieldTags.BLUE_HUB.size());
    assertEquals(4, FieldTags.RED_TRENCH.size());
    assertEquals(4, FieldTags.BLUE_TRENCH.size());
    assertEquals(2, FieldTags.RED_TOWER.size());
    assertEquals(2, FieldTags.BLUE_TOWER.size());
    assertEquals(2, FieldTags.RED_OUTPOST.size());
    assertEquals(2, FieldTags.BLUE_OUTPOST.size());
  }

  @Test
  void heightsMatchTheOfficialLayoutForEveryTag() {
    for (int id = 1; id <= 32; id++) {
      double official = FieldTags.poseOf(id).orElseThrow().getZ();
      assertEquals(official, FieldTags.heightMeters(id), 1e-3,
          "tag " + id + " height disagrees with the field layout");
    }
  }

  @Test
  void theThreeMountingHeightsAreWhatTheManualSays() {
    assertEquals(Units.inchesToMeters(44.25), TagFunction.HUB.heightMeters(), 1e-6);
    assertEquals(Units.inchesToMeters(35.0), TagFunction.TRENCH.heightMeters(), 1e-6);
    assertEquals(Units.inchesToMeters(21.75), TagFunction.TOWER.heightMeters(), 1e-6);
    assertEquals(Units.inchesToMeters(21.75), TagFunction.OUTPOST.heightMeters(), 1e-6);
  }

  /** The specific confusion that broke ranging before: tag 1 is a TRENCH tag, not a HUB tag. */
  @Test
  void tagOneIsTrenchNotHub() {
    assertEquals(TagFunction.TRENCH, FieldTags.functionOf(1));
    assertFalse(FieldTags.ALL_HUB.contains(1));
  }

  @Test
  void hubTagsAreSplitCorrectlyBetweenAlliances() {
    for (int id : FieldTags.RED_HUB) {
      assertEquals(Alliance.Red, FieldTags.allianceOf(id).orElseThrow(), "tag " + id);
      assertEquals(TagFunction.HUB, FieldTags.functionOf(id));
    }
    for (int id : FieldTags.BLUE_HUB) {
      assertEquals(Alliance.Blue, FieldTags.allianceOf(id).orElseThrow(), "tag " + id);
      assertEquals(TagFunction.HUB, FieldTags.functionOf(id));
    }
  }

  /** Red hub tags must sit in the red half and blue in the blue half, per the official poses. */
  @Test
  void eachAllianceHubIsInItsOwnHalfOfTheField() {
    double midfield = FieldTags.FIELD_LENGTH_METERS / 2.0;
    for (int id : FieldTags.RED_HUB) {
      assertTrue(FieldTags.poseOf(id).orElseThrow().getX() > midfield, "red tag " + id);
    }
    for (int id : FieldTags.BLUE_HUB) {
      assertTrue(FieldTags.poseOf(id).orElseThrow().getX() < midfield, "blue tag " + id);
    }
    assertTrue(FieldTags.RED_HUB_CENTER.getX() > midfield);
    assertTrue(FieldTags.BLUE_HUB_CENTER.getX() < midfield);
  }

  @Test
  void towerAndOutpostSitAtTheAllianceWalls() {
    double length = FieldTags.FIELD_LENGTH_METERS;
    for (int id : FieldTags.BLUE_TOWER) {
      assertTrue(FieldTags.poseOf(id).orElseThrow().getX() < 0.5, "blue tower " + id);
    }
    for (int id : FieldTags.RED_TOWER) {
      assertTrue(FieldTags.poseOf(id).orElseThrow().getX() > length - 0.5, "red tower " + id);
    }
  }

  @Test
  void unknownIdsAreRejectedRatherThanGuessed() {
    for (int id : new int[] {0, 33, 99, -1}) {
      assertFalse(FieldTags.isFieldTag(id));
      assertEquals(TagFunction.UNKNOWN, FieldTags.functionOf(id));
      assertTrue(FieldTags.allianceOf(id).isEmpty());
      assertTrue(FieldTags.describe(id).contains("NOT A 2026 FIELD TAG"));
    }
  }

  @Test
  void describeReadsUsefullyInALog() {
    assertEquals("20 (BLUE HUB)", FieldTags.describe(20));
    assertEquals("9 (RED HUB)", FieldTags.describe(9));
    assertEquals("1 (RED TRENCH)", FieldTags.describe(1));
    assertEquals("31 (BLUE TOWER)", FieldTags.describe(31));
    assertEquals("13 (RED OUTPOST)", FieldTags.describe(13));
  }

  /**
   * With no driver station attached, hub filtering must return BOTH hubs. Narrowing to one alliance
   * before the alliance is known would reject every tag on a bench.
   */
  @Test
  void hubTagsFallBackToBothAlliancesWhenNoAllianceIsReported() {
    assertEquals(FieldTags.ALL_HUB, FieldTags.hubTags());
    assertEquals(FieldTags.ALL_TOWER, FieldTags.towerTags());
    assertEquals(16, FieldTags.ALL_HUB.size());
  }

  @Test
  void onlyHubCountsAsAScoringTarget() {
    assertTrue(TagFunction.HUB.isScoringTarget());
    assertFalse(TagFunction.TRENCH.isScoringTarget());
    assertFalse(TagFunction.TOWER.isScoringTarget());
    assertFalse(TagFunction.OUTPOST.isScoringTarget());
  }
}
