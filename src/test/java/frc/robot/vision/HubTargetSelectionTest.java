package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The trench is covered in AprilTags and every auto parks beside it to shoot.
 *
 * <p>These pin down which tags may be used to aim and to range. The failure they guard against is
 * silent and expensive: a TRENCH tag is nearer and squarer to the camera from a neutral-zone
 * shooting position than the hub is, so anything that takes "the nearest tag" ends up aiming the
 * turret at the tunnel and ranging the shot to a wall two metres away.
 */
public class HubTargetSelectionTest {

  @Test
  public void trenchTagsAreNotScoringTargets() {
    for (int id : FieldTags.ALL_TRENCH) {
      assertFalse(
          FieldTags.functionOf(id).isScoringTarget(),
          "tag " + id + " is on the trench and must never be treated as something to shoot at");
    }
  }

  @Test
  public void towerAndOutpostTagsAreNotScoringTargets() {
    for (int id : FieldTags.ALL_TOWER) {
      assertFalse(FieldTags.functionOf(id).isScoringTarget(), "tower tag " + id);
    }
    for (int id : FieldTags.ALL_OUTPOST) {
      assertFalse(FieldTags.functionOf(id).isScoringTarget(), "outpost tag " + id);
    }
  }

  @Test
  public void everyHubTagIsAScoringTarget() {
    for (int id : FieldTags.ALL_HUB) {
      assertTrue(
          FieldTags.functionOf(id).isScoringTarget(),
          "tag " + id + " is on a hub and must be shootable");
    }
  }

  @Test
  public void bothAlliancesHubsCount() {
    // getHubTarget() deliberately does not filter by alliance. Filtering by alliance is what made
    // the turret ignore a tag held up in front of it on the bench, and either hub sits at the same
    // HUB mounting height, so the ranging is correct for both.
    assertTrue(FieldTags.functionOf(8).isScoringTarget(), "red hub tag 8");
    assertTrue(FieldTags.functionOf(20).isScoringTarget(), "blue hub tag 20");
  }

  @Test
  public void aTrenchTagAndAHubTagDoNotShareAMountingHeight() {
    // If they did, ranging off the wrong one would still produce a plausible number and the
    // mistake would never show up as an obviously silly distance.
    assertEquals(TagFunction.HUB, FieldTags.functionOf(20));
    assertEquals(TagFunction.TRENCH, FieldTags.functionOf(22));
    assertTrue(
        FieldTags.heightMeters(20) > FieldTags.heightMeters(22),
        "a hub tag sits higher than a trench tag, which is what makes ty-ranging distinguish them");
  }

  @Test
  public void theTrenchTagsFlankingOurShootingLanesAreRejected() {
    // Blue bottom and top trench openings carry 17/28 and 22/23. These are the exact tags in the
    // camera's view from the neutral-zone shooting spots.
    for (int id : new int[] {17, 28, 22, 23}) {
      assertEquals(
          TagFunction.TRENCH,
          FieldTags.functionOf(id),
          "tag " + id + " flanks a blue trench opening");
      assertFalse(FieldTags.functionOf(id).isScoringTarget());
    }
  }
}
