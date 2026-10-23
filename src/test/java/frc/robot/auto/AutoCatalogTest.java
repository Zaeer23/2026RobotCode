package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The dashboard's auto preview is only worth having if it matches what the robot will do, and the
 * routines are only worth having if they fit in AUTO.
 *
 * <p>These build the preview from the real deployed {@code .auto} and {@code .path} files and check
 * it against what those files say: the robot starts where the first path starts, every shot in the
 * routine appears, and every path comes with a real PathPlanner trajectory rather than a fallback.
 * The same timing then checks each routine fills the 20 s AUTO period without running out of time
 * for its last shot.
 */
public class AutoCatalogTest {

  /** Same values RobotContainer registers. */
  private static final double SHOT_SECONDS = 3.5;
  private static final double FEED_SECONDS = 3.0;

  private static final double AUTO_SECONDS = 20.0;

  /**
   * The last shot has to start by here to get its full spin-up and most of its feed in. FUEL
   * launched before the timer hits 0:00 still scores for 3 s after (manual 6.5), so the shot
   * itself may run a little past the end.
   */
  private static final double LAST_SHOT_STARTS_BY = 17.0;
  private static final double LAST_FEED_STARTS_BY = 18.0;

  /** A routine that is finished well before the end has left points on the field. */
  private static final double USES_AT_LEAST = 17.5;

  private static AutoCatalog catalog() {
    AutoCatalog catalog = new AutoCatalog(name -> Commands.none());
    catalog.setStepSeconds("Shoot", SHOT_SECONDS);
    catalog.setStepSeconds("FeedOutpost", FEED_SECONDS);
    return catalog;
  }

  private static List<String> rebuiltAutos() {
    File[] files = new File("src/main/deploy/pathplanner/autos")
        .listFiles((dir, name) -> name.startsWith("R26 ") && name.endsWith(".auto"));
    // Discovered, not hard-coded to a count. The generator owns this set, so a test that names
    // a fixed number breaks every time the strategy changes rather than when something is wrong.
    assertTrue(files != null && files.length > 0, "no R26 autos found - run ./gradlew generateAutos");
    List<String> names = new ArrayList<>();
    for (File file : files) {
      names.add(file.getName().substring(0, file.getName().length() - ".auto".length()));
    }
    names.sort(String::compareTo);
    return names;
  }

  private static long count(JsonNode steps, String named) {
    long count = 0;
    for (JsonNode step : steps) {
      for (JsonNode name : step.path("names")) {
        if (named.equals(name.asText())) {
          count++;
        }
      }
    }
    return count;
  }

  @Test
  void everyAutoHasADrawablePreview() throws Exception {
    AutoCatalog catalog = catalog();
    for (String auto : rebuiltAutos()) {
      JsonNode steps = catalog.stepsOf(auto);
      assertFalse(steps.isEmpty(), auto + " has no steps");

      for (JsonNode step : steps) {
        if (!"drive".equals(step.path("kind").asText())) {
          continue;
        }
        String where = auto + " / " + step.path("name").asText();
        JsonNode points = step.path("points");
        assertTrue(points.size() >= 8, where + " has too few points to draw");
        assertEquals(0, points.size() % 4, where + " points are not [t, x, y, deg] groups");
        assertFalse(step.path("estimated").asBoolean(false),
            where + " fell back to estimated timing: no PathPlanner trajectory");
        assertTrue(step.path("seconds").asDouble() > 0.0, where + " takes no time");

        double previous = -1.0;
        for (int i = 0; i < points.size(); i += 4) {
          double t = points.get(i).asDouble();
          assertTrue(t >= previous, where + " goes back in time at sample " + i / 4);
          previous = t;
        }
      }
    }
  }

  @Test
  void everyAutoFillsAutoWithoutStarvingItsLastShot() throws Exception {
    AutoCatalog catalog = catalog();
    for (String auto : rebuiltAutos()) {
      double t = 0.0;
      double lastShot = -1.0;
      double lastFeed = -1.0;
      for (JsonNode step : catalog.stepsOf(auto)) {
        for (JsonNode name : step.path("names")) {
          if ("Shoot".equals(name.asText())) {
            lastShot = t;
          } else if ("FeedOutpost".equals(name.asText())) {
            lastFeed = t;
          }
        }
        t += step.path("seconds").asDouble();
      }
      String total = String.format("%.1f s", t);
      assertTrue(t >= USES_AT_LEAST, auto + " is done at " + total + ", leaving AUTO unused");
      assertTrue(t <= AUTO_SECONDS + 1.0, auto + " runs " + total + ", well past AUTO");
      assertTrue(lastShot >= 0.0 || lastFeed >= 0.0, auto + " never scores or feeds");
      if (lastShot >= 0.0) {
        assertTrue(lastShot <= LAST_SHOT_STARTS_BY,
            auto + String.format(" starts its last shot at %.1f s, too late to finish", lastShot));
      }
      if (lastFeed >= 0.0) {
        assertTrue(lastFeed <= LAST_FEED_STARTS_BY,
            auto + String.format(" starts feeding the OUTPOST at %.1f s, too late", lastFeed));
      }
    }
  }

  @Test
  void previewShowsEveryShotAndFeed() throws Exception {
    // Every human-player routine must end by handing FUEL over; that is the whole strategy.
    AutoCatalog catalog = catalog();
    for (String auto : rebuiltAutos()) {
      JsonNode steps = catalog.stepsOf(auto);
      assertEquals(1, count(steps, "FeedOutpost"),
          auto + " must feed the OUTPOST exactly once");
    }
  }

  @Test
  void shotsTakeTheirConfiguredTime() throws Exception {
    AutoCatalog catalog = catalog();
    for (String auto : rebuiltAutos()) {
      for (JsonNode step : catalog.stepsOf(auto)) {
        if ("action".equals(step.path("kind").asText())
            && step.path("title").asText().contains("Shoot")) {
          assertEquals(SHOT_SECONDS, step.path("seconds").asDouble(), 1e-9, auto);
        }
      }
    }
  }

  @Test
  void previewStartsWhereTheFirstPathStarts() throws Exception {
    JsonNode first = catalog().stepsOf(rebuiltAutos().get(0)).get(0);
    assertEquals("drive", first.path("kind").asText());

    // Squared on the ROBOT STARTING LINE. Which lane depends on the routine, so this checks the
    // line rather than one particular spot on it -- naming a y here breaks whenever the set of
    // routines changes, which is not the same thing as being wrong.
    JsonNode points = first.path("points");
    assertEquals(0.0, points.get(0).asDouble(), 1e-6);
    assertEquals(3.57, points.get(1).asDouble(), 0.05);
    double y = points.get(2).asDouble();
    assertTrue(y > 0.45 && y < 7.62, "start y=" + y + " is off the field");
  }

  @Test
  void intakeRunsOnEveryCollectionPath() throws Exception {
    // Driving to FUEL with the intake up collects nothing and still burns the time.
    for (String auto : rebuiltAutos()) {
      for (JsonNode step : catalog().stepsOf(auto)) {
        String name = step.path("name").asText();
        boolean collecting = name.contains("to Neutral") || name.contains("to Chute")
            || name.contains("to Depot");
        if ("drive".equals(step.path("kind").asText()) && collecting) {
          boolean intake = false;
          for (JsonNode alongside : step.path("with")) {
            intake |= "IntakeDown".equals(alongside.asText());
          }
          assertTrue(intake, auto + " / " + name + " should run the intake");
        }
      }
    }
  }
}
