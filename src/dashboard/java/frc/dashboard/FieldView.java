package frc.dashboard;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.util.HashSet;
import java.util.Set;

import javax.swing.JPanel;

/**
 * Overhead field view built from what the camera can see.
 *
 * <p>Every tag's field position is known exactly, and the camera reports where a tag sits relative
 * to the robot. One seen tag therefore fixes the robot on the field with no odometry and no
 * accumulated drift. Drawing that next to the full tag layout answers the question that matters
 * from the driver station: not "where am I" so much as <em>"am I about to lose sight of the
 * target"</em>.
 *
 * <p>Tags currently in view are drawn lit; the rest are dim. When the estimate goes stale the
 * robot is drawn hollow rather than removed, so you can see where it last had a fix.
 */
public class FieldView extends JPanel {

  private static final Color BACKGROUND = new Color(0x1E, 0x20, 0x24);
  private static final Color FIELD_FILL = new Color(0x2A, 0x2D, 0x33);
  private static final Color FIELD_EDGE = new Color(0x55, 0x5A, 0x63);
  private static final Color TAG_DIM = new Color(0x4C, 0x52, 0x5C);
  private static final Color TAG_LIT = new Color(0x4C, 0xC2, 0xFF);
  private static final Color ROBOT_FILL = new Color(0x3D, 0xDC, 0x84);
  private static final Color ROBOT_STALE = new Color(0x7A, 0x7F, 0x88);
  private static final Color TEXT = new Color(0xD8, 0xDC, 0xE2);
  private static final Color HUB_RED = new Color(0xC2, 0x3B, 0x3B, 0x70);
  private static final Color HUB_BLUE = new Color(0x3B, 0x6F, 0xC2, 0x70);
  private static final Color SIGHT_LINE = new Color(0x4C, 0xC2, 0xFF, 0x80);

  /** Hub tag IDs per alliance, from the official 2026 layout. */
  private static final Set<Integer> RED_HUB_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
  private static final Set<Integer> BLUE_HUB_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

  private double fieldLength = 16.541;
  private double fieldWidth = 8.069;

  /** Flat [id, x, y, z, yaw] per tag, as published by the robot. */
  private double[] tagLayout = new double[0];

  private final Set<Integer> visibleTagIds = new HashSet<>();

  private boolean poseValid = false;
  private double robotX = 0.0;
  private double robotY = 0.0;
  private double robotHeadingDegrees = 0.0;

  public FieldView() {
    setPreferredSize(new Dimension(640, 340));
    setBackground(BACKGROUND);
  }

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
      this.robotX = x;
      this.robotY = y;
      this.robotHeadingDegrees = headingDegrees;
    }
  }

  @Override
  protected void paintComponent(Graphics g) {
    super.paintComponent(g);
    Graphics2D g2 = (Graphics2D) g.create();
    g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

    // Fit the field into the panel with a margin, preserving aspect ratio.
    int margin = 16;
    double usableW = getWidth() - 2.0 * margin;
    double usableH = getHeight() - 2.0 * margin;
    if (usableW <= 0 || usableH <= 0) {
      g2.dispose();
      return;
    }
    double scale = Math.min(usableW / fieldLength, usableH / fieldWidth);
    double originX = (getWidth() - fieldLength * scale) / 2.0;
    double originY = (getHeight() - fieldWidth * scale) / 2.0;

    g2.setColor(FIELD_FILL);
    g2.fillRect((int) originX, (int) originY,
        (int) (fieldLength * scale), (int) (fieldWidth * scale));
    g2.setColor(FIELD_EDGE);
    g2.drawRect((int) originX, (int) originY,
        (int) (fieldLength * scale), (int) (fieldWidth * scale));

    // Centre line.
    int midX = (int) (originX + (fieldLength / 2.0) * scale);
    g2.drawLine(midX, (int) originY, midX, (int) (originY + fieldWidth * scale));

    drawStructures(g2, originX, originY, scale);
    drawTags(g2, originX, originY, scale);
    drawRobot(g2, originX, originY, scale);
    drawSightLines(g2, originX, originY, scale);

    g2.setColor(TEXT);
    g2.setFont(getFont().deriveFont(Font.PLAIN, 11f));
    g2.drawString(poseValid
        ? String.format("vision pose  x=%.2f  y=%.2f  hdg=%.0f deg", robotX, robotY,
            robotHeadingDegrees)
        : "no tag in view - robot position unknown", margin, getHeight() - 4);

    g2.dispose();
  }

  /**
   * Draws the structures the tags belong to, inferred from the tags themselves.
   *
   * <p>Each alliance's HUB is the centroid of its eight hub tags, so the outline is derived from
   * the official layout rather than hardcoded dimensions that would drift if the field changed.
   * Seeing the hub as a shape rather than eight loose dots is what makes the robot's position
   * relative to it readable at a glance.
   */
  private void drawStructures(Graphics2D g2, double originX, double originY, double scale) {
    drawHubOutline(g2, originX, originY, scale, RED_HUB_IDS, HUB_RED);
    drawHubOutline(g2, originX, originY, scale, BLUE_HUB_IDS, HUB_BLUE);

    // Alliance wall tint, so which end is which is obvious without reading tag numbers.
    int wallWidth = (int) (0.5 * scale);
    g2.setColor(HUB_BLUE);
    g2.fillRect((int) originX, (int) originY, wallWidth, (int) (fieldWidth * scale));
    g2.setColor(HUB_RED);
    g2.fillRect((int) (originX + fieldLength * scale) - wallWidth, (int) originY,
        wallWidth, (int) (fieldWidth * scale));
  }

  private void drawHubOutline(
      Graphics2D g2, double originX, double originY, double scale,
      Set<Integer> hubIds, Color colour) {
    double sumX = 0;
    double sumY = 0;
    int count = 0;
    double minX = Double.MAX_VALUE;
    double maxX = -Double.MAX_VALUE;
    double minY = Double.MAX_VALUE;
    double maxY = -Double.MAX_VALUE;

    for (int i = 0; i + 4 < tagLayout.length; i += 5) {
      int id = (int) tagLayout[i];
      if (!hubIds.contains(id)) {
        continue;
      }
      double x = tagLayout[i + 1];
      double y = tagLayout[i + 2];
      sumX += x;
      sumY += y;
      count++;
      minX = Math.min(minX, x);
      maxX = Math.max(maxX, x);
      minY = Math.min(minY, y);
      maxY = Math.max(maxY, y);
    }
    if (count == 0) {
      return;
    }

    int px = (int) (originX + minX * scale);
    int py = (int) (originY + (fieldWidth - maxY) * scale);
    int w = (int) ((maxX - minX) * scale);
    int h = (int) ((maxY - minY) * scale);

    g2.setColor(colour);
    g2.fillRoundRect(px, py, Math.max(w, 4), Math.max(h, 4), 8, 8);
    g2.setColor(colour.brighter());
    g2.setStroke(new BasicStroke(1.5f));
    g2.drawRoundRect(px, py, Math.max(w, 4), Math.max(h, 4), 8, 8);
  }

  /** Lines from the robot to each visible tag, which makes losing sight of one visible early. */
  private void drawSightLines(Graphics2D g2, double originX, double originY, double scale) {
    if (!poseValid || visibleTagIds.isEmpty()) {
      return;
    }
    int rx = (int) (originX + robotX * scale);
    int ry = (int) (originY + (fieldWidth - robotY) * scale);

    g2.setStroke(new BasicStroke(1f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
        10f, new float[] {4f, 4f}, 0f));
    g2.setColor(SIGHT_LINE);

    for (int i = 0; i + 4 < tagLayout.length; i += 5) {
      int id = (int) tagLayout[i];
      if (!visibleTagIds.contains(id)) {
        continue;
      }
      int tx = (int) (originX + tagLayout[i + 1] * scale);
      int ty = (int) (originY + (fieldWidth - tagLayout[i + 2]) * scale);
      g2.drawLine(rx, ry, tx, ty);
    }
    g2.setStroke(new BasicStroke(1f));
  }

  private void drawTags(Graphics2D g2, double originX, double originY, double scale) {
    g2.setFont(getFont().deriveFont(Font.PLAIN, 9f));
    for (int i = 0; i + 4 < tagLayout.length; i += 5) {
      int id = (int) tagLayout[i];
      double x = tagLayout[i + 1];
      double y = tagLayout[i + 2];

      // Field +Y is left, but screen +Y is down, so Y is flipped for drawing.
      int px = (int) (originX + x * scale);
      int py = (int) (originY + (fieldWidth - y) * scale);

      boolean lit = visibleTagIds.contains(id);
      g2.setColor(lit ? TAG_LIT : TAG_DIM);
      int r = lit ? 6 : 4;
      g2.fillOval(px - r, py - r, r * 2, r * 2);

      if (lit) {
        g2.drawString(String.valueOf(id), px + 7, py - 6);
      }
    }
  }

  private void drawRobot(Graphics2D g2, double originX, double originY, double scale) {
    if (!poseValid && robotX == 0.0 && robotY == 0.0) {
      return;
    }

    int px = (int) (originX + robotX * scale);
    int py = (int) (originY + (fieldWidth - robotY) * scale);

    // Roughly a 30 inch robot.
    double sizeMeters = 0.76;
    int size = Math.max(10, (int) (sizeMeters * scale));

    AffineTransform saved = g2.getTransform();
    g2.translate(px, py);
    // Screen Y is flipped relative to the field, so CCW-positive heading rotates clockwise here.
    g2.rotate(-Math.toRadians(robotHeadingDegrees));

    g2.setStroke(new BasicStroke(2f));
    if (poseValid) {
      g2.setColor(ROBOT_FILL);
      g2.fillRoundRect(-size / 2, -size / 2, size, size, 6, 6);
      g2.setColor(BACKGROUND);
    } else {
      g2.setColor(ROBOT_STALE);
      g2.drawRoundRect(-size / 2, -size / 2, size, size, 6, 6);
    }

    // Heading tick out the FRONT, and a second mark on the back where the turret and camera are.
    g2.drawLine(0, 0, size / 2, 0);
    g2.setColor(poseValid ? BACKGROUND : ROBOT_STALE);
    g2.drawLine(-size / 2, -size / 4, -size / 2, size / 4);

    g2.setTransform(saved);
  }
}
