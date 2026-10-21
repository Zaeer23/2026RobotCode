package frc.dashboard;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import javax.swing.JPanel;

/**
 * Overhead view of the field, used twice: live on the Match tab, and as the autonomous preview.
 *
 * <p><b>Live</b>, it draws the robot where the camera puts it. Every tag's field position is known
 * exactly, and the camera reports where a tag sits relative to the robot, so one seen tag fixes
 * the robot on the field with no odometry and no drift. Next to the full tag layout that answers
 * the question that matters from the driver station: not so much "where am I" as <em>"am I about
 * to lose sight of the target"</em>. Tags in view are lit; when the estimate goes stale the robot
 * is drawn hollow rather than removed, so you can see where it last had a fix.
 *
 * <p><b>As a preview</b>, it draws an autonomous routine: where the robot starts, the path it
 * drives, a numbered badge for every step, and where each shot is taken from. A ghost robot can be
 * placed at any moment of the routine, which is how the Autonomous tab plays it back.
 *
 * <p>The field itself is drawn to the 2026 manual (section 5): each half's barrier of TRENCH, BUMP,
 * HUB, BUMP, TRENCH that every path has to get through, the ROBOT STARTING LINES, and along each
 * ALLIANCE WALL the OUTPOST with its opening, the TOWER and the DEPOT with its 24 FUEL. The
 * neutral-zone FUEL is drawn where it is staged at the start of a match. Red is blue rotated 180
 * degrees about the field centre, which is how the real field is built.
 */
public class FieldView extends JPanel {

  // ---- Field geometry, 2026 REBUILT ------------------------------------------------------------

  // All BLUE side, from the manual and the official AprilTag layout; red is rotated 180 degrees.

  /** The ALLIANCE ZONE is 158.6 in deep; the ROBOT STARTING LINE is the 2 in tape at its edge. */
  private static final double ALLIANCE_ZONE_DEPTH = 4.028;
  private static final double LINE_WIDTH = 0.051;

  /** TRENCHES are 47.0 in deep, centred on the HUB, and 65.65 in wide from the guardrail. */
  private static final double BARRIER_NEAR_X = 4.029;
  private static final double BARRIER_FAR_X = 5.223;
  private static final double TRENCH_WIDTH = 1.668;

  /** BUMPS are 44.4 in deep, centred on the HUB, between each TRENCH and the HUB. */
  private static final double BUMP_NEAR_X = 4.062;
  private static final double BUMP_FAR_X = 5.190;

  /** The HUB is a 47 in square with a 41.7 in hexagonal opening. */
  private static final double HUB_SIZE = 1.194;
  private static final double HUB_OPENING = 1.059;
  private static final double[] BLUE_HUB_FALLBACK = {4.626, 4.035};

  /** DEPOT: 42.0 in along the ALLIANCE WALL, 27.0 in deep. */
  private static final double DEPOT_DEPTH = 0.686;
  private static final double DEPOT_LOW_Y = 5.40;
  private static final double DEPOT_HIGH_Y = 6.467;

  /** TOWER BASE: 39.0 in wide, 45.18 in deep, centred on tag 31; UPRIGHTS 32.25 in apart. */
  private static final double TOWER_DEPTH = 1.148;
  private static final double TOWER_CENTRE_Y = 3.746;
  private static final double TOWER_HALF_WIDTH = 0.495;
  private static final double UPRIGHT_HALF_SPACING = 0.410;

  /** OUTPOST opening at the base of the OUTPOST, 32.0 in wide, centred on tag 29. */
  private static final double OUTPOST_OPENING_CENTRE_Y = 0.666;
  private static final double OUTPOST_OPENING_HALF_WIDTH = 0.406;

  /** OUTPOST AREA, 71.0 in wide, from the guardrail along the ALLIANCE WALL. */
  private static final double OUTPOST_WIDTH = 1.80;

  /** Neutral-zone FUEL is staged in a 206 in by 72 in box centred on the field. */
  private static final double FUEL_BOX_HALF_LENGTH = 0.915;
  private static final double FUEL_BOX_HALF_WIDTH = 2.615;
  private static final double FUEL_RADIUS = 0.075;

  /** Robot with bumpers, matching PathPlanner's settings. */
  private static final double ROBOT_SIZE = 0.9;


  // ---- Colours ---------------------------------------------------------------------------------

  private static final Color CARPET = new Color(0x1A1F28);
  private static final Color GRID = new Color(255, 255, 255, 9);
  private static final Color FIELD_EDGE = new Color(0x3E4655);
  private static final Color BUMP = new Color(0x272E3A);
  private static final Color FUEL = new Color(0xF2C94C);
  private static final Color STEEL = new Color(0x8A93A3);
  private static final Color TAG_DIM = new Color(0x4A5262);
  private static final Color ROBOT_LIVE = Theme.GOOD;
  private static final Color ROBOT_STALE = new Color(0x7A8292);

  // ---- State -----------------------------------------------------------------------------------

  private double fieldLength = 16.541;
  private double fieldWidth = 8.069;

  /** Flat [id, x, y, z, yaw] per tag, as published by the robot. */
  private double[] tagLayout = new double[0];
  private final Set<Integer> visibleTagIds = new HashSet<>();

  private boolean showLive = true;
  private boolean poseValid = false;
  private boolean everHadPose = false;
  private double robotX = 0.0;
  private double robotY = 0.0;
  private double robotHeadingDegrees = 0.0;

  private AutoRoutine routine;
  private boolean mirrored = false;
  private boolean routineFaint = false;
  private double playhead = Double.NaN;
  private String message;

  /** Visible x range in field metres. NaN shows the whole field. */
  private double viewFromX = Double.NaN;
  private double viewToX = Double.NaN;

  // Transform for the current paint.
  private double visibleWidth;
  private double scale;
  private double originX;
  private double originY;

  public FieldView() {
    setPreferredSize(new Dimension(640, 340));
    setOpaque(false);
  }

  // ---- Inputs ----------------------------------------------------------------------------------

  public void setFieldSize(double lengthMeters, double widthMeters) {
    if (lengthMeters > 1 && widthMeters > 1) {
      this.fieldLength = lengthMeters;
      this.fieldWidth = widthMeters;
    }
  }

  public void setTagLayout(double[] layout) {
    this.tagLayout = layout == null ? new double[0] : layout;
  }

  public void setVisibleTags(double[] ids) {
    visibleTagIds.clear();
    if (ids != null) {
      for (double id : ids) {
        visibleTagIds.add((int) id);
      }
    }
  }

  public void setRobotPose(boolean valid, double x, double y, double headingDegrees) {
    this.poseValid = valid;
    if (valid) {
      this.everHadPose = true;
      this.robotX = x;
      this.robotY = y;
      this.robotHeadingDegrees = headingDegrees;
    }
  }

  /** Whether to draw the live robot and its sight lines. Off for the auto preview. */
  public void setShowLive(boolean show) {
    this.showLive = show;
  }

  /**
   * The routine to draw, or null for none.
   *
   * @param red mirror it onto the red half, as PathPlanner does when it runs it
   * @param faint draw it quietly, as a reminder behind live data rather than the main subject
   */
  public void setRoutine(AutoRoutine routine, boolean red, boolean faint) {
    this.routine = routine;
    this.mirrored = red;
    this.routineFaint = faint;
  }

  /** Seconds into the routine to place the ghost robot at, or NaN for no ghost. */
  public void setPlayhead(double seconds) {
    this.playhead = seconds;
  }

  /**
   * Shows only part of the field's length, or all of it with NaN.
   *
   * <p>Autonomous never leaves its own half, so the preview zooms to that half and the route is
   * drawn nearly twice the size it would be on the whole field.
   */
  public void setViewX(double fromX, double toX) {
    this.viewFromX = fromX;
    this.viewToX = toX;
  }

  /** A note drawn over the middle of the field, or null. */
  public void setMessage(String message) {
    this.message = message;
  }

  // ---- Painting --------------------------------------------------------------------------------

  @Override
  protected void paintComponent(Graphics g) {
    super.paintComponent(g);
    Graphics2D g2 = (Graphics2D) g.create();
    Theme.smooth(g2);

    int margin = 14;
    double fromX = Double.isNaN(viewFromX) ? 0.0 : Math.max(0.0, viewFromX);
    double toX = Double.isNaN(viewToX) ? fieldLength : Math.min(fieldLength, viewToX);
    double usableW = getWidth() - 2.0 * margin;
    double usableH = getHeight() - 2.0 * margin;
    if (usableW <= 0 || usableH <= 0 || toX <= fromX) {
      g2.dispose();
      return;
    }
    scale = Math.min(usableW / (toX - fromX), usableH / fieldWidth);
    originX = (getWidth() - (toX - fromX) * scale) / 2.0 - fromX * scale;
    originY = (getHeight() - fieldWidth * scale) / 2.0;
    visibleWidth = (toX - fromX) * scale;

    RoundRectangle2D visible = new RoundRectangle2D.Double(
        px(fromX), originY, (toX - fromX) * scale, fieldWidth * scale, 10, 10);
    Shape outerClip = g2.getClip();
    g2.clip(visible);

    drawCarpet(g2);
    drawNeutralFuel(g2);
    for (boolean red : new boolean[] {false, true}) {
      drawBarrier(g2, red);
      drawWall(g2, red);
    }
    drawTags(g2);

    if (routine != null && routine.drives()) {
      drawRoutine(g2);
    }
    if (showLive) {
      drawSightLines(g2);
      drawLiveRobot(g2);
      drawPoseReadout(g2);
    }
    g2.setClip(outerClip);

    // A cut edge fades out, so a half-field view reads as part of a bigger field.
    if (fromX > 0.0) {
      fadeEdge(g2, px(fromX), true);
    }
    if (toX < fieldLength) {
      fadeEdge(g2, px(toX), false);
    }
    g2.setStroke(new BasicStroke(1.5f));
    g2.setColor(FIELD_EDGE);
    g2.draw(visible);

    if (message != null) {
      drawMessage(g2, message);
    }
    g2.dispose();
  }

  private void fadeEdge(Graphics2D g2, double x, boolean leftEdge) {
    double width = 36;
    double from = leftEdge ? x : x - width;
    g2.setPaint(new java.awt.GradientPaint(
        (float) (leftEdge ? x + width : x - width), 0f, Theme.alpha(Theme.SURFACE, 0),
        (float) x, 0f, Theme.alpha(Theme.SURFACE, 235)));
    g2.fill(new Rectangle2D.Double(from, originY - 1, width, fieldWidth * scale + 2));
  }

  private double px(double x) {
    return originX + x * scale;
  }

  private double py(double y) {
    return originY + (fieldWidth - y) * scale;
  }

  /** Routine coordinates are authored on blue; red runs them rotated about the field centre. */
  private double rx(double x) {
    return mirrored ? fieldLength - x : x;
  }

  private double ry(double y) {
    return mirrored ? fieldWidth - y : y;
  }

  private double rh(double headingDegrees) {
    return mirrored ? headingDegrees + 180.0 : headingDegrees;
  }

  private Rectangle2D fieldRect(double x0, double y0, double x1, double y1) {
    double left = px(Math.min(x0, x1));
    double top = py(Math.max(y0, y1));
    return new Rectangle2D.Double(left, top, Math.abs(x1 - x0) * scale, Math.abs(y1 - y0) * scale);
  }

  /** A blue-side rectangle, or the same rectangle rotated onto the red side. */
  private Rectangle2D sideRect(boolean red, double x0, double y0, double x1, double y1) {
    return red
        ? fieldRect(fieldLength - x1, fieldWidth - y1, fieldLength - x0, fieldWidth - y0)
        : fieldRect(x0, y0, x1, y1);
  }

  private double sideX(boolean red, double x) {
    return red ? fieldLength - x : x;
  }

  private double sideY(boolean red, double y) {
    return red ? fieldWidth - y : y;
  }

  /** +1 toward the field from the blue wall, -1 from the red one. */
  private static int sideSign(boolean red) {
    return red ? -1 : 1;
  }

  /** Labels only go on when the field is drawn big enough to read them without clutter. */
  private boolean roomForLabels() {
    return scale > 55;
  }

  private void label(Graphics2D g2, String text, double cx, double cy, Color colour) {
    if (!roomForLabels()) {
      return;
    }
    g2.setFont(Theme.caption(9.5f));
    FontMetrics fm = g2.getFontMetrics();
    g2.setColor(colour);
    g2.drawString(text, (float) (cx - fm.stringWidth(text) / 2.0),
        (float) (cy + (fm.getAscent() - fm.getDescent()) / 2.0));
  }

  private void drawCarpet(Graphics2D g2) {
    RoundRectangle2D field = new RoundRectangle2D.Double(
        originX, originY, fieldLength * scale, fieldWidth * scale, 10, 10);
    g2.setColor(CARPET);
    g2.fill(field);

    // One-metre grid: enough to judge distance by eye without reading as clutter.
    Shape savedClip = g2.getClip();
    g2.clip(field);
    g2.setColor(GRID);
    g2.setStroke(new BasicStroke(1f));
    for (int x = 1; x < fieldLength; x++) {
      g2.draw(new Line2D.Double(px(x), py(0), px(x), py(fieldWidth)));
    }
    for (int y = 1; y < fieldWidth; y++) {
      g2.draw(new Line2D.Double(px(0), py(y), px(fieldLength), py(y)));
    }

    for (boolean red : new boolean[] {false, true}) {
      Color alliance = red ? Theme.ALLIANCE_RED : Theme.ALLIANCE_BLUE;
      // ALLIANCE ZONE, wall to ROBOT STARTING LINE.
      g2.setColor(Theme.alpha(alliance, 26));
      g2.fill(sideRect(red, 0, 0, ALLIANCE_ZONE_DEPTH, fieldWidth));
      // ROBOT STARTING LINE: alliance-coloured tape at the zone's edge.
      Rectangle2D line = sideRect(red, ALLIANCE_ZONE_DEPTH - LINE_WIDTH, 0, ALLIANCE_ZONE_DEPTH,
          fieldWidth);
      double lineWidth = Math.max(2.0, line.getWidth());
      g2.setColor(Theme.alpha(alliance, 210));
      g2.fill(new Rectangle2D.Double(line.getCenterX() - lineWidth / 2, line.getY(), lineWidth,
          line.getHeight()));
      // ALLIANCE WALL.
      g2.setStroke(new BasicStroke((float) Math.max(3.0, 0.12 * scale)));
      g2.setColor(Theme.alpha(alliance, 200));
      double wallX = px(sideX(red, 0));
      g2.draw(new Line2D.Double(wallX, py(0), wallX, py(fieldWidth)));
    }
    g2.setClip(savedClip);

    // CENTER LINE: white tape across the middle of the NEUTRAL ZONE.
    g2.setColor(Theme.alpha(Color.WHITE, 120));
    g2.setStroke(new BasicStroke((float) Math.max(1.5, LINE_WIDTH * scale)));
    g2.draw(new Line2D.Double(px(fieldLength / 2), py(0), px(fieldLength / 2), py(fieldWidth)));
  }

  /** The 360 to 408 neutral-zone FUEL in the box they are staged in, drawn faintly. */
  private void drawNeutralFuel(Graphics2D g2) {
    double cx = fieldLength / 2;
    double cy = fieldWidth / 2;
    double spacing = FUEL_RADIUS * 2.05;
    double r = Math.max(1.0, FUEL_RADIUS * scale * 0.85);
    g2.setColor(Theme.alpha(FUEL, 46));
    for (double x = cx - FUEL_BOX_HALF_LENGTH + FUEL_RADIUS; x < cx + FUEL_BOX_HALF_LENGTH;
        x += spacing) {
      for (double y = cy - FUEL_BOX_HALF_WIDTH + FUEL_RADIUS; y < cy + FUEL_BOX_HALF_WIDTH;
          y += spacing) {
        g2.fill(new Ellipse2D.Double(px(x) - r, py(y) - r, r * 2, r * 2));
      }
    }
  }

  /** TRENCH, BUMP, HUB, BUMP, TRENCH: the line every path has to get through. */
  private void drawBarrier(Graphics2D g2, boolean red) {
    Color alliance = red ? Theme.ALLIANCE_RED : Theme.ALLIANCE_BLUE;
    double[] hub = hubCentre(red);
    double hubX = red ? fieldLength - hub[0] : hub[0];
    double hubY = red ? fieldWidth - hub[1] : hub[1];

    // BUMPS: ramps the robot can drive over, between each TRENCH and the HUB. Laid out in blue
    // coordinates and rotated, like everything else on the red side.
    double blueHubLow = hub[1] - HUB_SIZE / 2;
    double blueHubHigh = hub[1] + HUB_SIZE / 2;
    g2.setStroke(new BasicStroke(1f));
    for (double[] span : new double[][] {{TRENCH_WIDTH, blueHubLow},
        {blueHubHigh, fieldWidth - TRENCH_WIDTH}}) {
      Rectangle2D r = sideRect(red, BUMP_NEAR_X, span[0], BUMP_FAR_X, span[1]);
      RoundRectangle2D bump = new RoundRectangle2D.Double(r.getX(), r.getY(), r.getWidth(),
          r.getHeight(), 6, 6);
      g2.setColor(Theme.mix(BUMP, alliance, 0.12));
      g2.fill(bump);
      g2.setColor(Theme.alpha(alliance, 90));
      g2.draw(bump);
      // Ridge along the crest.
      g2.setColor(Theme.alpha(Color.WHITE, 22));
      g2.draw(new Line2D.Double(r.getCenterX(), r.getY() + 3, r.getCenterX(), r.getMaxY() - 3));
      label(g2, "BUMP", r.getCenterX(), r.getCenterY(), Theme.alpha(Theme.TEXT, 90));
    }

    // TRENCHES: open structures the robot drives under, so hatched rather than solid.
    for (double[] span : new double[][] {{0, TRENCH_WIDTH}, {fieldWidth - TRENCH_WIDTH, fieldWidth}}) {
      Rectangle2D r = sideRect(red, BARRIER_NEAR_X, span[0], BARRIER_FAR_X, span[1]);
      g2.setColor(Theme.alpha(alliance, 18));
      g2.fill(r);
      Shape savedClip = g2.getClip();
      g2.clip(r);
      g2.setColor(Theme.alpha(alliance, 55));
      double step = Math.max(6, 0.22 * scale);
      for (double d = -r.getHeight(); d < r.getWidth(); d += step) {
        g2.draw(new Line2D.Double(r.getX() + d, r.getMaxY(), r.getX() + d + r.getHeight(), r.getY()));
      }
      g2.setClip(savedClip);
      g2.setColor(Theme.alpha(alliance, 120));
      g2.draw(r);
      label(g2, "TRENCH", r.getCenterX(), r.getCenterY(), Theme.alpha(Theme.TEXT, 110));
    }

    // HUB, with its hexagonal opening.
    Rectangle2D hubRect = fieldRect(hubX - HUB_SIZE / 2, hubY - HUB_SIZE / 2, hubX + HUB_SIZE / 2,
        hubY + HUB_SIZE / 2);
    RoundRectangle2D hubShape = new RoundRectangle2D.Double(hubRect.getX(), hubRect.getY(),
        hubRect.getWidth(), hubRect.getHeight(), 8, 8);
    g2.setColor(Theme.alpha(alliance, 95));
    g2.fill(hubShape);
    g2.setColor(Theme.alpha(alliance, 230));
    g2.setStroke(new BasicStroke(1.6f));
    g2.draw(hubShape);
    Path2D hexagon = new Path2D.Double();
    double radius = HUB_OPENING / 2 * scale;
    for (int k = 0; k < 6; k++) {
      double a = Math.PI / 6 + k * Math.PI / 3;
      double hx = px(hubX) + radius * Math.cos(a);
      double hy = py(hubY) + radius * Math.sin(a);
      if (k == 0) {
        hexagon.moveTo(hx, hy);
      } else {
        hexagon.lineTo(hx, hy);
      }
    }
    hexagon.closePath();
    g2.setColor(Theme.alpha(CARPET, 215));
    g2.fill(hexagon);
    g2.setColor(Theme.alpha(alliance, 160));
    g2.draw(hexagon);
    label(g2, "HUB", px(hubX), py(hubY), Theme.alpha(Theme.TEXT, 150));
  }

  /** What stands along each ALLIANCE WALL: the OUTPOST and its opening, the TOWER, and the DEPOT. */
  private void drawWall(Graphics2D g2, boolean red) {
    Color alliance = red ? Theme.ALLIANCE_RED : Theme.ALLIANCE_BLUE;
    g2.setStroke(new BasicStroke(1.4f));

    // DEPOT: a low steel frame against the wall holding 24 FUEL.
    Rectangle2D depot = sideRect(red, 0, DEPOT_LOW_Y, DEPOT_DEPTH, DEPOT_HIGH_Y);
    g2.setColor(Theme.alpha(STEEL, 40));
    g2.fill(depot);
    g2.setColor(Theme.alpha(STEEL, 200));
    g2.draw(depot);
    double fr = Math.max(1.2, FUEL_RADIUS * scale * 0.9);
    g2.setColor(Theme.alpha(FUEL, 170));
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 6; j++) {
        double fx = sideX(red, DEPOT_DEPTH * (i + 0.5) / 4);
        double fy = sideY(red, DEPOT_LOW_Y + (DEPOT_HIGH_Y - DEPOT_LOW_Y) * (j + 0.5) / 6);
        g2.fill(new Ellipse2D.Double(px(fx) - fr, py(fy) - fr, fr * 2, fr * 2));
      }
    }
    label(g2, "DEPOT", depot.getCenterX() + sideSign(red) * (depot.getWidth() / 2 + 22),
        depot.getCenterY(), Theme.alpha(Theme.TEXT, 120));

    // TOWER: base plate out from the wall, with the two UPRIGHTS the RUNGS span.
    Rectangle2D tower = sideRect(red, 0, TOWER_CENTRE_Y - TOWER_HALF_WIDTH, TOWER_DEPTH,
        TOWER_CENTRE_Y + TOWER_HALF_WIDTH);
    g2.setColor(Theme.alpha(alliance, 40));
    g2.fill(tower);
    g2.setColor(Theme.alpha(alliance, 170));
    g2.draw(tower);
    double upright = Math.max(3.0, 0.09 * scale);
    double rungX = px(sideX(red, 0.10));
    double lowUpright = py(sideY(red, TOWER_CENTRE_Y - UPRIGHT_HALF_SPACING));
    double highUpright = py(sideY(red, TOWER_CENTRE_Y + UPRIGHT_HALF_SPACING));
    g2.setColor(Theme.alpha(alliance, 200));
    g2.draw(new Line2D.Double(rungX, lowUpright, rungX, highUpright));
    g2.setColor(alliance);
    g2.fill(new Rectangle2D.Double(rungX - upright / 2, lowUpright - upright / 2, upright, upright));
    g2.fill(new Rectangle2D.Double(rungX - upright / 2, highUpright - upright / 2, upright,
        upright));
    label(g2, "TOWER", tower.getCenterX() + sideSign(red) * 6, tower.getCenterY(),
        Theme.alpha(Theme.TEXT, 140));

    // OUTPOST: where the HUMAN PLAYER works. Its OUTPOST opening is where robots feed them FUEL.
    Rectangle2D outpost = sideRect(red, 0, 0, 0.10, OUTPOST_WIDTH);
    g2.setColor(Theme.alpha(alliance, 120));
    g2.fill(outpost);
    Rectangle2D opening = sideRect(red, 0, OUTPOST_OPENING_CENTRE_Y - OUTPOST_OPENING_HALF_WIDTH, 0.16,
        OUTPOST_OPENING_CENTRE_Y + OUTPOST_OPENING_HALF_WIDTH);
    g2.setColor(Theme.alpha(FUEL, 150));
    g2.fill(opening);
    label(g2, "OUTPOST", opening.getCenterX() + sideSign(red) * 34, opening.getCenterY(),
        Theme.alpha(Theme.TEXT, 140));
  }

  /**
   * HUB centre from its own tags: midway between opposite faces, in BLUE coordinates. For red it
   * is the red HUB rotated onto the blue side, so callers rotate it like everything else.
   *
   * <p>Not the mean of all eight HUB tags: each face has one centred tag and one offset one, and
   * the offsets do not cancel, so the mean sits 9 cm toward the ALLIANCE WALL.
   */
  private double[] hubCentre(boolean red) {
    double[] wallFace = tagXY(red ? 10 : 26);
    double[] fieldFace = tagXY(red ? 4 : 20);
    double[] lowFace = tagXY(red ? 5 : 18);
    double[] highFace = tagXY(red ? 2 : 21);
    if (wallFace == null || fieldFace == null || lowFace == null || highFace == null) {
      return BLUE_HUB_FALLBACK.clone();
    }
    double x = (wallFace[0] + fieldFace[0]) / 2.0;
    double y = (lowFace[1] + highFace[1]) / 2.0;
    return red ? new double[] {fieldLength - x, fieldWidth - y} : new double[] {x, y};
  }

  private double[] tagXY(int id) {
    for (int i = 0; i + 4 < tagLayout.length; i += 5) {
      if ((int) tagLayout[i] == id) {
        return new double[] {tagLayout[i + 1], tagLayout[i + 2]};
      }
    }
    return null;
  }

  private void drawTags(Graphics2D g2) {
    g2.setFont(Theme.ui(Font.BOLD, 10f));
    for (int i = 0; i + 4 < tagLayout.length; i += 5) {
      int id = (int) tagLayout[i];
      double x = px(tagLayout[i + 1]);
      double y = py(tagLayout[i + 2]);
      boolean lit = showLive && visibleTagIds.contains(id);
      if (lit) {
        g2.setColor(Theme.alpha(Theme.ACCENT, 60));
        g2.fill(new Ellipse2D.Double(x - 8, y - 8, 16, 16));
        g2.setColor(Theme.ACCENT);
        g2.fill(new Ellipse2D.Double(x - 4, y - 4, 8, 8));
        g2.drawString(String.valueOf(id), (float) x + 8, (float) y - 7);
      } else {
        g2.setColor(TAG_DIM);
        g2.fill(new Ellipse2D.Double(x - 2.5, y - 2.5, 5, 5));
      }
    }
  }

  // ---- Autonomous route ------------------------------------------------------------------------

  private void drawRoutine(Graphics2D g2) {
    double fade = routineFaint ? 0.45 : 1.0;
    boolean playing = !Double.isNaN(playhead);
    int activeStep = playing ? routine.stepIndexAt(playhead) : -1;

    // Shot aim lines first, so the route draws over them.
    double[] hubBlue = hubCentre(false);
    for (AutoRoutine.Step step : routine.steps) {
      if (step.kind == AutoRoutine.Kind.SHOT && step.located()) {
        drawAimLine(g2, step, hubBlue, fade);
      }
    }

    // Each path: a dark underlay so it reads over any structure, then the route colour. While
    // playing, the part already driven is bright and the rest is dimmed.
    for (AutoRoutine.Step step : routine.steps) {
      if (step.kind != AutoRoutine.Kind.DRIVE || step.points == null || step.points.length < 8) {
        continue;
      }
      Color colour = step.intake ? Theme.ROUTE_INTAKE : Theme.ROUTE_DRIVE;
      Path2D full = pathOf(step.points, Double.POSITIVE_INFINITY);

      g2.setStroke(new BasicStroke(8f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g2.setColor(Theme.alpha(Theme.BG, (int) (170 * fade)));
      g2.draw(full);

      g2.setStroke(new BasicStroke(3.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      if (playing) {
        g2.setColor(Theme.alpha(colour, (int) (140 * fade)));
        g2.draw(full);
        double into = playhead - step.start;
        if (into > 0) {
          g2.setColor(Theme.alpha(colour, (int) (255 * fade)));
          g2.draw(pathOf(step.points, into));
        }
      } else {
        g2.setColor(Theme.alpha(colour, (int) (255 * fade)));
        g2.draw(full);
      }

      if (!routineFaint) {
        drawChevrons(g2, step.points, Theme.alpha(Theme.mix(colour, Theme.TEXT, 0.5), 220));
      }
    }

    // Where it starts.
    double[] start = routine.start;
    drawRobot(g2, rx(start[0]), ry(start[1]), rh(start[2]),
        Theme.alpha(Theme.TEXT, (int) (18 * fade)), Theme.alpha(Theme.TEXT, (int) (200 * fade)),
        true);
    if (!routineFaint) {
      drawTag(g2, "START", px(rx(start[0])), py(ry(start[1])) - ROBOT_SIZE * scale * 0.5 - 12,
          Theme.TEXT, Theme.SURFACE_RAISED);
      drawBadges(g2, activeStep);
    }

    // The ghost: where the robot is at this moment of the routine.
    if (playing) {
      double[] pose = routine.poseAt(playhead);
      if (pose != null) {
        drawRobot(g2, rx(pose[0]), ry(pose[1]), rh(pose[2]), Theme.alpha(Theme.ACCENT, 80),
            Theme.ACCENT, false);
      }
    }
  }

  /** Screen-space path through the samples, up to {@code untilSeconds} into the step. */
  private Path2D pathOf(double[] points, double untilSeconds) {
    Path2D path = new Path2D.Double();
    path.moveTo(px(rx(points[1])), py(ry(points[2])));
    for (int i = 4; i + 3 < points.length; i += 4) {
      double t = points[i];
      if (t > untilSeconds) {
        double t0 = points[i - 4];
        double u = t > t0 ? (untilSeconds - t0) / (t - t0) : 1.0;
        double x = points[i - 3] + (points[i + 1] - points[i - 3]) * u;
        double y = points[i - 2] + (points[i + 2] - points[i - 2]) * u;
        path.lineTo(px(rx(x)), py(ry(y)));
        break;
      }
      path.lineTo(px(rx(points[i + 1])), py(ry(points[i + 2])));
    }
    return path;
  }

  /** Small arrowheads along a path, so its direction reads without playing it. */
  private void drawChevrons(Graphics2D g2, double[] points, Color colour) {
    double spacing = Math.max(1.0, 70.0 / scale);
    double nextAt = spacing * 0.5;
    double distance = 0.0;
    g2.setColor(colour);
    g2.setStroke(new BasicStroke(1.8f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
    for (int i = 4; i + 3 < points.length; i += 4) {
      double x0 = rx(points[i - 3]);
      double y0 = ry(points[i - 2]);
      double x1 = rx(points[i + 1]);
      double y1 = ry(points[i + 2]);
      double seg = Math.hypot(x1 - x0, y1 - y0);
      if (seg < 1e-6) {
        continue;
      }
      while (distance + seg >= nextAt) {
        double u = (nextAt - distance) / seg;
        double cx = px(x0 + (x1 - x0) * u);
        double cy = py(y0 + (y1 - y0) * u);
        // Screen direction: y flips.
        double angle = Math.atan2(-(y1 - y0), x1 - x0);
        AffineTransform saved = g2.getTransform();
        g2.translate(cx, cy);
        g2.rotate(angle);
        Path2D chevron = new Path2D.Double();
        chevron.moveTo(-3.5, -4.5);
        chevron.lineTo(1.5, 0);
        chevron.lineTo(-3.5, 4.5);
        g2.draw(chevron);
        g2.setTransform(saved);
        nextAt += spacing;
      }
      distance += seg;
    }
  }

  private void drawAimLine(Graphics2D g2, AutoRoutine.Step step, double[] hubBlue, double fade) {
    double x0 = px(rx(step.x));
    double y0 = py(ry(step.y));
    double x1 = px(rx(hubBlue[0]));
    double y1 = py(ry(hubBlue[1]));
    g2.setStroke(new BasicStroke(1.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10f,
        new float[] {2f, 5f}, 0f));
    g2.setColor(Theme.alpha(Theme.ROUTE_SHOT, (int) (200 * fade)));
    g2.draw(new Line2D.Double(x0, y0, x1, y1));
  }

  /**
   * Numbered badges matching the step list. Steps that happen at the same spot (a shot straight
   * after a path, say) sit side by side rather than on top of each other.
   */
  private void drawBadges(Graphics2D g2, int activeStep) {
    List<double[]> anchors = new ArrayList<>();
    List<List<Integer>> groups = new ArrayList<>();

    for (int i = 0; i < routine.steps.size(); i++) {
      AutoRoutine.Step step = routine.steps.get(i);
      double sx;
      double sy;
      if (step.kind == AutoRoutine.Kind.DRIVE && step.points != null && step.points.length >= 8) {
        double[] mid = pointAtFraction(step.points, 0.5);
        sx = px(rx(mid[0]));
        sy = py(ry(mid[1]));
      } else if (step.located()) {
        sx = px(rx(step.x));
        sy = py(ry(step.y));
      } else {
        continue;
      }

      int group = -1;
      for (int a = 0; a < anchors.size(); a++) {
        if (Math.hypot(anchors.get(a)[0] - sx, anchors.get(a)[1] - sy) < 20) {
          group = a;
          break;
        }
      }
      if (group < 0) {
        anchors.add(new double[] {sx, sy});
        groups.add(new ArrayList<>());
        group = anchors.size() - 1;
      }
      groups.get(group).add(i);
    }

    int radius = 11;
    for (int a = 0; a < anchors.size(); a++) {
      List<Integer> members = groups.get(a);
      double width = members.size() * (radius * 2 + 4) - 4;
      double x = anchors.get(a)[0] - width / 2 + radius;
      double y = anchors.get(a)[1];
      for (int index : members) {
        drawBadge(g2, x, y, radius, index + 1, colourOf(routine.steps.get(index)),
            index == activeStep);
        x += radius * 2 + 4;
      }
    }
  }

  private void drawBadge(Graphics2D g2, double x, double y, int radius, int number, Color colour,
      boolean active) {
    double r = active ? radius + 2 : radius;
    if (active) {
      g2.setColor(Theme.alpha(colour, 70));
      g2.fill(new Ellipse2D.Double(x - r - 5, y - r - 5, (r + 5) * 2, (r + 5) * 2));
    }
    g2.setColor(Theme.BG);
    g2.fill(new Ellipse2D.Double(x - r - 2, y - r - 2, (r + 2) * 2, (r + 2) * 2));
    g2.setColor(colour);
    g2.fill(new Ellipse2D.Double(x - r, y - r, r * 2, r * 2));
    if (active) {
      g2.setColor(Theme.TEXT);
      g2.setStroke(new BasicStroke(2f));
      g2.draw(new Ellipse2D.Double(x - r, y - r, r * 2, r * 2));
    }
    g2.setFont(Theme.numeric(Font.BOLD, 12f));
    FontMetrics fm = g2.getFontMetrics();
    String text = String.valueOf(number);
    g2.setColor(Theme.TEXT_ON_ACCENT);
    g2.drawString(text, (float) (x - fm.stringWidth(text) / 2.0),
        (float) (y + (fm.getAscent() - fm.getDescent()) / 2.0));
  }

  static Color colourOf(AutoRoutine.Step step) {
    switch (step.kind) {
      case DRIVE:
        return step.intake ? Theme.ROUTE_INTAKE : Theme.ROUTE_DRIVE;
      case SHOT:
        return Theme.ROUTE_SHOT;
      default:
        return Theme.ROUTE_OTHER;
    }
  }

  private static double[] pointAtFraction(double[] points, double fraction) {
    double total = 0.0;
    for (int i = 4; i + 3 < points.length; i += 4) {
      total += Math.hypot(points[i + 1] - points[i - 3], points[i + 2] - points[i - 2]);
    }
    double target = total * fraction;
    double run = 0.0;
    for (int i = 4; i + 3 < points.length; i += 4) {
      double seg = Math.hypot(points[i + 1] - points[i - 3], points[i + 2] - points[i - 2]);
      if (run + seg >= target && seg > 0) {
        double u = (target - run) / seg;
        return new double[] {
          points[i - 3] + (points[i + 1] - points[i - 3]) * u,
          points[i - 2] + (points[i + 2] - points[i - 2]) * u
        };
      }
      run += seg;
    }
    return new double[] {points[1], points[2]};
  }

  // ---- Live robot ------------------------------------------------------------------------------

  /** Lines from the robot to each visible tag, which makes losing sight of one visible early. */
  private void drawSightLines(Graphics2D g2) {
    if (!poseValid || visibleTagIds.isEmpty()) {
      return;
    }
    double rxp = px(robotX);
    double ryp = py(robotY);
    g2.setStroke(new BasicStroke(1.2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10f,
        new float[] {4f, 4f}, 0f));
    g2.setColor(Theme.alpha(Theme.ACCENT, 140));
    for (int i = 0; i + 4 < tagLayout.length; i += 5) {
      if (visibleTagIds.contains((int) tagLayout[i])) {
        g2.draw(new Line2D.Double(rxp, ryp, px(tagLayout[i + 1]), py(tagLayout[i + 2])));
      }
    }
  }

  private void drawLiveRobot(Graphics2D g2) {
    if (!everHadPose) {
      return;
    }
    if (poseValid) {
      drawRobot(g2, robotX, robotY, robotHeadingDegrees, ROBOT_LIVE,
          Theme.mix(ROBOT_LIVE, Color.WHITE, 0.3), false);
    } else {
      drawRobot(g2, robotX, robotY, robotHeadingDegrees, Theme.alpha(ROBOT_STALE, 30),
          ROBOT_STALE, true);
    }
  }

  /**
   * A robot footprint with a notch on its front.
   *
   * <p>Screen Y is flipped relative to the field, so a CCW-positive heading rotates clockwise here.
   */
  private void drawRobot(Graphics2D g2, double x, double y, double headingDegrees, Color fill,
      Color outline, boolean dashed) {
    double size = Math.max(12.0, ROBOT_SIZE * scale);
    AffineTransform saved = g2.getTransform();
    g2.translate(px(x), py(y));
    g2.rotate(-Math.toRadians(headingDegrees));

    RoundRectangle2D body = new RoundRectangle2D.Double(-size / 2, -size / 2, size, size,
        size * 0.22, size * 0.22);
    g2.setColor(fill);
    g2.fill(body);
    Stroke stroke = dashed
        ? new BasicStroke(1.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10f,
            new float[] {4f, 3f}, 0f)
        : new BasicStroke(2f);
    g2.setStroke(stroke);
    g2.setColor(outline);
    g2.draw(body);

    // Front: a filled arrowhead on the leading edge.
    Path2D nose = new Path2D.Double();
    nose.moveTo(size / 2 - size * 0.30, -size * 0.20);
    nose.lineTo(size / 2 - size * 0.06, 0);
    nose.lineTo(size / 2 - size * 0.30, size * 0.20);
    nose.closePath();
    g2.fill(nose);

    g2.setTransform(saved);
  }

  private void drawPoseReadout(Graphics2D g2) {
    String text = poseValid
        ? String.format("x %.2f m   y %.2f m   %.0f°", robotX, robotY, robotHeadingDegrees)
        : (everHadPose ? "No tag in view · last fix shown" : "No tag in view · position unknown");
    Color ink = poseValid ? Theme.TEXT : Theme.TEXT_DIM;
    g2.setFont(Theme.numeric(Font.PLAIN, 12f));
    FontMetrics fm = g2.getFontMetrics();
    double w = fm.stringWidth(text) + 20;
    double h = 24;
    double x = Math.max(originX, 0) + 10;
    double y = originY + fieldWidth * scale - h - 10;
    g2.setColor(Theme.alpha(Theme.BG, 210));
    g2.fill(new RoundRectangle2D.Double(x, y, w, h, h, h));
    g2.setColor(ink);
    g2.drawString(text, (float) (x + 10), (float) (y + (h + fm.getAscent() - fm.getDescent()) / 2));
  }

  private void drawTag(Graphics2D g2, String text, double cx, double cy, Color ink, Color fill) {
    g2.setFont(Theme.caption(10f));
    FontMetrics fm = g2.getFontMetrics();
    double w = fm.stringWidth(text) + 14;
    double h = 18;
    g2.setColor(Theme.alpha(fill, 235));
    g2.fill(new RoundRectangle2D.Double(cx - w / 2, cy - h / 2, w, h, h, h));
    g2.setColor(ink);
    g2.drawString(text, (float) (cx - w / 2 + 7), (float) (cy + (fm.getAscent() - fm.getDescent()) / 2.0));
  }

  private void drawMessage(Graphics2D g2, String text) {
    g2.setFont(Theme.ui(Font.BOLD, 14f));
    FontMetrics fm = g2.getFontMetrics();
    int maxWidth = (int) Math.max(120, Math.min(visibleWidth - 60, 520));
    List<String> lines = wrap(fm, text, maxWidth);
    double lineHeight = fm.getHeight();
    double w = 0;
    for (String line : lines) {
      w = Math.max(w, fm.stringWidth(line));
    }
    w += 36;
    double h = lines.size() * lineHeight + 28;
    double x = getWidth() / 2.0 - w / 2;
    double y = getHeight() / 2.0 - h / 2;
    g2.setColor(Theme.alpha(Theme.BG, 225));
    g2.fill(new RoundRectangle2D.Double(x, y, w, h, 16, 16));
    g2.setColor(Theme.BORDER_STRONG);
    g2.setStroke(new BasicStroke(1f));
    g2.draw(new RoundRectangle2D.Double(x, y, w, h, 16, 16));
    g2.setColor(Theme.TEXT);
    double baseline = y + 14 + fm.getAscent();
    for (String line : lines) {
      g2.drawString(line, (float) (getWidth() / 2.0 - fm.stringWidth(line) / 2.0), (float) baseline);
      baseline += lineHeight;
    }
  }

  private static List<String> wrap(FontMetrics fm, String text, int width) {
    List<String> lines = new ArrayList<>();
    StringBuilder line = new StringBuilder();
    for (String word : text.split("\\s+")) {
      String candidate = line.length() == 0 ? word : line + " " + word;
      if (fm.stringWidth(candidate) > width && line.length() > 0) {
        lines.add(line.toString());
        line = new StringBuilder(word);
      } else {
        line = new StringBuilder(candidate);
      }
    }
    if (line.length() > 0) {
      lines.add(line.toString());
    }
    return lines;
  }
}
