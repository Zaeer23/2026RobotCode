package frc.dashboard;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * One autonomous routine as the robot described it, laid out on a timeline.
 *
 * <p>The robot publishes each routine as an ordered list of steps (drive a path, shoot, wait),
 * with every path as timed samples of the trajectory PathPlanner will follow. This turns that into
 * something the dashboard can draw and animate: when each step starts and ends, where the robot is
 * at any moment, and where each shot happens.
 *
 * <p>Everything here is in BLUE-side field coordinates, exactly as authored. Mirroring for the red
 * alliance happens at draw time, the same way PathPlanner flips the paths when it runs them.
 */
final class AutoRoutine {

  enum Kind {
    DRIVE, SHOT, ACTION, WAIT
  }

  /** One step of a routine, positioned on the timeline and, where known, on the field. */
  static final class Step {
    final Kind kind;
    final String title;
    final String detail;
    final double start;
    final double end;
    final boolean intake;

    /** Timed samples, {@code [t, x, y, deg]} with t relative to this step. Drive steps only. */
    final double[] points;

    /** Where the step happens. NaN when the routine never drives, so the position is unknown. */
    final double x;
    final double y;
    final double heading;

    Step(Kind kind, String title, String detail, double start, double end, boolean intake,
        double[] points, double x, double y, double heading) {
      this.kind = kind;
      this.title = title;
      this.detail = detail;
      this.start = start;
      this.end = end;
      this.intake = intake;
      this.points = points;
      this.x = x;
      this.y = y;
      this.heading = heading;
    }

    double seconds() {
      return end - start;
    }

    boolean located() {
      return !Double.isNaN(x);
    }
  }

  final String label;
  final boolean isDefault;
  final String error;
  final String note;
  final List<Step> steps;
  final double totalSeconds;
  final int shots;
  final double drivenMeters;

  /** {@code [x, y, deg]} where the robot starts, or null if it starts wherever it was put. */
  final double[] start;

  private AutoRoutine(String label, boolean isDefault, String error, String note, List<Step> steps,
      double[] start, double drivenMeters) {
    this.label = label;
    this.isDefault = isDefault;
    this.error = error;
    this.note = note;
    this.steps = Collections.unmodifiableList(steps);
    this.start = start;
    this.drivenMeters = drivenMeters;
    this.totalSeconds = steps.isEmpty() ? 0.0 : steps.get(steps.size() - 1).end;
    this.shots = (int) steps.stream().filter(s -> s.kind == Kind.SHOT).count();
  }

  /** A routine the robot offers but sent no preview for (older robot code). */
  static AutoRoutine placeholder(String label, boolean isDefault) {
    return new AutoRoutine(label, isDefault, null,
        "No preview from the robot. Deploy the latest robot code to see this one drawn.",
        new ArrayList<>(), null, 0.0);
  }

  boolean drives() {
    return start != null;
  }

  /** One line for a list: "3 shots · 14.6 s · 9.2 m". */
  String summary() {
    if (error != null) {
      return "Preview unavailable";
    }
    if (steps.isEmpty()) {
      return note != null && note.startsWith("No preview") ? "No preview" : "Stays still";
    }
    List<String> parts = new ArrayList<>();
    parts.add(shots == 1 ? "1 shot" : shots + " shots");
    parts.add(String.format(Locale.ROOT, "%.1f s", totalSeconds));
    if (drives()) {
      parts.add(String.format(Locale.ROOT, "%.1f m", drivenMeters));
    } else {
      parts.add("no driving");
    }
    return String.join("  ·  ", parts);
  }

  /** Index of the step running at time t, clamped to the routine. -1 if there are no steps. */
  int stepIndexAt(double t) {
    if (steps.isEmpty()) {
      return -1;
    }
    for (int i = 0; i < steps.size(); i++) {
      Step step = steps.get(i);
      // Instant steps never "contain" a time, so they are never the active one mid-playback.
      if (t >= step.start && t < step.end) {
        return i;
      }
    }
    return t < steps.get(0).start ? 0 : steps.size() - 1;
  }

  /** {@code [x, y, deg]} of the robot at time t, or null if the routine never drives. */
  double[] poseAt(double t) {
    if (start == null) {
      return null;
    }
    double[] pose = start.clone();
    for (Step step : steps) {
      if (step.kind == Kind.DRIVE && step.points != null && step.points.length >= 4) {
        if (t < step.start) {
          return pose;
        }
        if (t <= step.end) {
          return sample(step.points, t - step.start);
        }
        int last = step.points.length - 4;
        pose = new double[] {step.points[last + 1], step.points[last + 2], step.points[last + 3]};
      }
    }
    return pose;
  }

  private static double[] sample(double[] points, double t) {
    int n = points.length / 4;
    if (t <= points[0]) {
      return new double[] {points[1], points[2], points[3]};
    }
    for (int i = 1; i < n; i++) {
      double t1 = points[i * 4];
      if (t <= t1) {
        double t0 = points[(i - 1) * 4];
        double u = t1 > t0 ? (t - t0) / (t1 - t0) : 1.0;
        return new double[] {
          lerp(points[(i - 1) * 4 + 1], points[i * 4 + 1], u),
          lerp(points[(i - 1) * 4 + 2], points[i * 4 + 2], u),
          lerpAngle(points[(i - 1) * 4 + 3], points[i * 4 + 3], u)
        };
      }
    }
    int last = (n - 1) * 4;
    return new double[] {points[last + 1], points[last + 2], points[last + 3]};
  }

  private static double lerp(double a, double b, double u) {
    return a + (b - a) * u;
  }

  private static double lerpAngle(double a, double b, double u) {
    double delta = ((b - a + 540.0) % 360.0) - 180.0;
    return a + delta * u;
  }

  // ---- Parsing ----------------------------------------------------------------

  private static final ObjectMapper JSON = new ObjectMapper();

  /** Parses the robot's {@code Dashboard/Auto/Catalog} JSON. */
  static List<AutoRoutine> parseCatalog(String json) throws Exception {
    List<AutoRoutine> routines = new ArrayList<>();
    JsonNode root = JSON.readTree(json);
    for (JsonNode node : root.path("routines")) {
      routines.add(parse(node));
    }
    return routines;
  }

  private static AutoRoutine parse(JsonNode node) {
    String label = node.path("label").asText("?");
    boolean isDefault = node.path("default").asBoolean(false);
    String error = node.hasNonNull("error") ? node.path("error").asText() : null;
    String note = node.hasNonNull("note") ? node.path("note").asText() : null;

    // The start is where the first path starts; anything before it happens there too.
    double[] start = null;
    for (JsonNode step : node.path("steps")) {
      JsonNode points = step.path("points");
      if ("drive".equals(step.path("kind").asText()) && points.size() >= 4) {
        start = new double[] {points.get(1).asDouble(), points.get(2).asDouble(),
          points.get(3).asDouble()};
        break;
      }
    }

    List<Step> steps = new ArrayList<>();
    double t = 0.0;
    double meters = 0.0;
    double[] here = start == null ? null : start.clone();

    for (JsonNode step : node.path("steps")) {
      String kindName = step.path("kind").asText("action");
      double seconds = Math.max(0.0, step.path("seconds").asDouble(0.0));

      if ("drive".equals(kindName)) {
        JsonNode raw = step.path("points");
        double[] points = new double[raw.size() - raw.size() % 4];
        for (int i = 0; i < points.length; i++) {
          points[i] = raw.get(i).asDouble();
        }
        List<String> with = textList(step.path("with"));
        boolean intake = with.stream().anyMatch(n -> n.toLowerCase(Locale.ROOT).contains("intake"));

        double length = 0.0;
        for (int i = 4; i < points.length; i += 4) {
          length += Math.hypot(points[i + 1] - points[i - 3], points[i + 2] - points[i - 2]);
        }
        meters += length;

        StringBuilder detail = new StringBuilder(
            String.format(Locale.ROOT, "%.1f s  ·  %.1f m", seconds, length));
        // The intake is already in the step's kind ("Drive + intake"); name anything else.
        List<String> others = with.stream()
            .filter(n -> !n.toLowerCase(Locale.ROOT).contains("intake"))
            .collect(java.util.stream.Collectors.toList());
        if (!others.isEmpty()) {
          detail.append("  ·  with ").append(humanize(others).toLowerCase(Locale.ROOT));
        }
        if (step.path("estimated").asBoolean(false)) {
          detail.append("  ·  timing estimated");
        }

        double x = points.length >= 4 ? points[1] : Double.NaN;
        double y = points.length >= 4 ? points[2] : Double.NaN;
        double heading = points.length >= 4 ? points[3] : Double.NaN;
        steps.add(new Step(Kind.DRIVE, step.path("name").asText("Path"), detail.toString(),
            t, t + seconds, intake, points, x, y, heading));
        if (points.length >= 4) {
          int last = points.length - 4;
          here = new double[] {points[last + 1], points[last + 2], points[last + 3]};
        }
      } else if ("wait".equals(kindName)) {
        steps.add(new Step(Kind.WAIT, "Wait", String.format(Locale.ROOT, "%.1f s", seconds),
            t, t + seconds, false, null, at(here, 0), at(here, 1), at(here, 2)));
      } else {
        List<String> names = textList(step.path("names"));
        String title = step.hasNonNull("title") ? step.path("title").asText() : humanize(names);
        boolean shot = names.stream().anyMatch(n -> n.equalsIgnoreCase("Shoot"))
            || title.toLowerCase(Locale.ROOT).contains("shot")
            || title.toLowerCase(Locale.ROOT).contains("shoot");
        String detail = seconds <= 0.0
            ? "instant"
            : String.format(Locale.ROOT, shot ? "up to %.1f s" : "%.1f s", seconds);
        steps.add(new Step(shot ? Kind.SHOT : Kind.ACTION, title, detail,
            t, t + seconds, false, null, at(here, 0), at(here, 1), at(here, 2)));
      }
      t += seconds;
    }

    return new AutoRoutine(label, isDefault, error, note, steps, start, meters);
  }

  private static double at(double[] pose, int index) {
    return pose == null ? Double.NaN : pose[index];
  }

  private static List<String> textList(JsonNode array) {
    List<String> out = new ArrayList<>();
    for (JsonNode item : array) {
      String text = item.asText("");
      if (!text.isBlank()) {
        out.add(text);
      }
    }
    return out;
  }

  /** {@code ["SpinUp", "IntakeDown"]} becomes {@code "Spin up + Intake down"}. */
  static String humanize(List<String> names) {
    if (names.isEmpty()) {
      return "Action";
    }
    List<String> words = new ArrayList<>();
    for (String name : names) {
      String spaced = name.replaceAll("([a-z0-9])([A-Z])", "$1 $2");
      words.add(spaced.isEmpty()
          ? spaced
          : spaced.substring(0, 1).toUpperCase(Locale.ROOT)
              + spaced.substring(1).toLowerCase(Locale.ROOT));
    }
    return String.join(" + ", words);
  }
}
