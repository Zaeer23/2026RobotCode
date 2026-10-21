package frc.dashboard;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.RoundRectangle2D;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Consumer;

import javax.swing.JPanel;
import javax.swing.event.MouseInputAdapter;

/**
 * Logitech F310 gamepad, drawn as an outline of the real pad.
 *
 * <p>The silhouette is traced from the actual controller: a wide body with a shoulder bump at each
 * top corner, two grips sweeping down and outward, and a shallow notch between them. The D-pad is a
 * single cross upper-left, the face buttons are a diamond upper-right in the F310's own colours,
 * and both sticks sit together at the bottom. That last part is the F310's defining difference from
 * an Xbox pad, and getting it wrong makes the picture actively misleading — the whole value of a
 * diagram over a list is that it maps onto the thing in your hands without translation.
 *
 * <p>Buttons keep their real colours whether or not they are bound, so the drawing stays
 * recognisable; binding state is carried by a ring around the button and a label instead. Labels
 * sit in a column beside whichever half of the pad the control is on, joined by a leader line, so
 * they never cover the body even when everything is bound.
 *
 * <p>Clicking a button selects it, which is how the editor beside it chooses what to rebind.
 */
public class ControllerDiagram extends JPanel {

  private static final Color INK = Theme.TEXT_ON_ACCENT;
  private static final Color BODY_EDGE = new Color(0x79859A);
  private static final Color BODY_FILL = new Color(0x222733);
  private static final Color WELL = new Color(0x1A1F27);
  private static final Color FREE = new Color(0x59626F);
  private static final Color BOUND = Theme.ACCENT;
  private static final Color SELECTED = Theme.GOOD;
  private static final Color LABEL = Theme.TEXT;
  private static final Color DIM = Theme.TEXT_DIM;

  // The F310's own button colours, as printed on the pad.
  private static final Color F310_Y = new Color(0xE8A01C);
  private static final Color F310_X = new Color(0x2F6FD0);
  private static final Color F310_B = new Color(0xD2403C);
  private static final Color F310_A = new Color(0x3FA94B);

  /** Reference canvas. All geometry below is in these coordinates, scaled to fit the panel. */
  private static final int REF_W = 600;
  private static final int REF_H = 300;

  /** The pad is drawn this far in from the left, leaving a label column either side of it. */
  private static final int SHIFT = 70;

  /** Centre line of the pad, used to decide which side a label goes on. */
  private static final int CENTRE_X = 230;

  /** Label columns, in pad coordinates: just outside the body on each side. */
  private static final int LEFT_COLUMN = 84;
  private static final int RIGHT_COLUMN = 376;

  /** Minimum vertical spacing between two labels in the same column. */
  private static final int LABEL_SPACING = 17;

  private static final Map<String, Point> LAYOUT = new LinkedHashMap<>();

  static {
    // Triggers sit behind the bumpers on the far edge.
    LAYOUT.put("LEFT_TRIGGER", new Point(133, 28));
    LAYOUT.put("RIGHT_TRIGGER", new Point(327, 28));
    LAYOUT.put("LEFT_BUMPER", new Point(133, 60));
    LAYOUT.put("RIGHT_BUMPER", new Point(327, 60));

    // D-pad, upper LEFT on an F310. These are the four arms of one cross.
    LAYOUT.put("POV_UP", new Point(141, 106));
    LAYOUT.put("POV_LEFT", new Point(117, 130));
    LAYOUT.put("POV_RIGHT", new Point(165, 130));
    LAYOUT.put("POV_DOWN", new Point(141, 154));

    // Face buttons, upper RIGHT, in the F310's diamond.
    LAYOUT.put("Y", new Point(319, 106));
    LAYOUT.put("X", new Point(295, 130));
    LAYOUT.put("B", new Point(343, 130));
    LAYOUT.put("A", new Point(319, 154));

    // Centre cluster, either side of the logo.
    LAYOUT.put("BACK", new Point(204, 120));
    LAYOUT.put("START", new Point(256, 120));

    // Both sticks together at the BOTTOM.
    LAYOUT.put("LEFT_STICK", new Point(186, 194));
    LAYOUT.put("RIGHT_STICK", new Point(274, 194));
  }

  private Map<String, String> bindingsByInput = Map.of();
  private String selectedInput;
  private String hoveredInput;
  private Consumer<String> onSelect = input -> { };
  private String title = "Controller";

  public ControllerDiagram() {
    setPreferredSize(new Dimension(REF_W, REF_H));
    setOpaque(false);

    MouseInputAdapter mouse = new MouseInputAdapter() {
      @Override
      public void mousePressed(java.awt.event.MouseEvent e) {
        String hit = inputAt(e.getX(), e.getY());
        if (hit != null) {
          selectedInput = hit;
          onSelect.accept(hit);
          repaint();
        }
      }

      @Override
      public void mouseMoved(java.awt.event.MouseEvent e) {
        String hit = inputAt(e.getX(), e.getY());
        if (!java.util.Objects.equals(hit, hoveredInput)) {
          hoveredInput = hit;
          setCursor(java.awt.Cursor.getPredefinedCursor(
              hit == null ? java.awt.Cursor.DEFAULT_CURSOR : java.awt.Cursor.HAND_CURSOR));
          repaint();
        }
      }

      @Override
      public void mouseExited(java.awt.event.MouseEvent e) {
        hoveredInput = null;
        repaint();
      }
    };
    addMouseListener(mouse);
    addMouseMotionListener(mouse);
  }

  public void setTitle(String title) {
    this.title = title;
  }

  /** @param bindings input name to the action bound to it */
  public void setBindings(Map<String, String> bindings) {
    this.bindingsByInput = bindings == null ? Map.of() : bindings;
    repaint();
  }

  public void setOnSelect(Consumer<String> listener) {
    this.onSelect = listener == null ? input -> { } : listener;
  }

  public String selectedInput() {
    return selectedInput;
  }

  /** Lets the editor drive the selection, so picking an action highlights its button. */
  public void setSelectedInput(String input) {
    this.selectedInput = input;
    repaint();
  }

  private double scale() {
    return Math.min(getWidth() / (double) REF_W, getHeight() / (double) REF_H);
  }

  private double offsetX() {
    return (getWidth() - REF_W * scale()) / 2.0;
  }

  private double offsetY() {
    return (getHeight() - REF_H * scale()) / 2.0;
  }

  private String inputAt(int x, int y) {
    double s = scale();
    for (Map.Entry<String, Point> entry : LAYOUT.entrySet()) {
      double px = offsetX() + (entry.getValue().x + SHIFT) * s;
      double py = offsetY() + entry.getValue().y * s;
      double r = 14 * s;
      if ((x - px) * (x - px) + (y - py) * (y - py) <= r * r) {
        return entry.getKey();
      }
    }
    return null;
  }

  @Override
  protected void paintComponent(Graphics g) {
    super.paintComponent(g);
    Graphics2D g2 = (Graphics2D) g.create();
    Theme.smooth(g2);

    g2.translate(offsetX(), offsetY());
    g2.scale(scale(), scale());

    g2.setFont(Theme.caption(11f));
    g2.setColor(DIM);
    g2.drawString(title, 8, 16);

    g2.translate(SHIFT, 0);
    drawBody(g2);
    drawWells(g2);
    drawDpad(g2);
    drawCentreCluster(g2);
    drawInputs(g2);
    drawLabels(g2);

    g2.dispose();
  }

  /**
   * The F310 silhouette, drawn as an outline.
   *
   * <p>Body, two shoulder bumps and two grips are unioned into one Area first, so the seams between
   * them disappear and the stroke traces a single continuous edge the way the real pad reads.
   */
  private void drawBody(Graphics2D g2) {
    java.awt.Shape body = f310Outline();
    g2.setColor(BODY_FILL);
    g2.fill(body);
    g2.setColor(BODY_EDGE);
    g2.setStroke(new BasicStroke(2.2f));
    g2.draw(body);
  }

  /**
   * The F310 silhouette as one continuous curve.
   *
   * <p>Built as a single path rather than a union of rounded rectangles. Unioning boxes leaves a
   * visible step where each shoulder meets the body and gives the grips parallel sides, so the
   * drawing came out as a generic blob with blocks stuck on it rather than this controller.
   *
   * <p>Traced down the right half from the top centre and mirrored, so the two halves cannot drift
   * apart as the numbers get nudged.
   */
  private java.awt.Shape f310Outline() {
    double cx = 230;
    Path2D.Double half = new Path2D.Double();

    // Top centre, dipping between the two shoulders.
    half.moveTo(cx, 52);
    // Up and over the right shoulder crest, where the bumper and trigger sit.
    half.curveTo(cx + 46, 48, cx + 70, 14, cx + 108, 18);
    // Round the outer corner of the shoulder and come down the right flank.
    half.curveTo(cx + 138, 22, cx + 146, 50, cx + 144, 78);
    // Waist: the pad pinches in under the bumper before the grip starts.
    half.curveTo(cx + 142, 108, cx + 132, 120, cx + 134, 142);
    // Outer edge of the grip, sweeping out and down. This long convex run is what gives the F310
    // its shape; a rotated rectangle here has parallel sides and reads as a stick, not a handle.
    half.curveTo(cx + 141, 174, cx + 141, 210, cx + 124, 240);
    // Round the tip, narrower than the base so the grip tapers into the hand.
    half.curveTo(cx + 113, 262, cx + 83, 270, cx + 68, 250);
    // Back up the inside of the grip, the short concave edge facing its twin.
    half.curveTo(cx + 60, 240, cx + 56, 230, cx + 52, 218);
    // Across the bottom of the body, dipping shallowly between the two grips. This runs BELOW the
    // sticks: they sit in the body just above the grips, not in the gap between them.
    half.curveTo(cx + 44, 234, cx + 24, 242, cx, 242);

    // Close each half along the centre line and union them. Appending the mirrored half to the
    // original instead would walk it in the same direction, leaving a visible seam straight down
    // the middle of the pad; a union drops the shared edge because the two halves meet on it.
    half.closePath();

    AffineTransform mirror = new AffineTransform();
    mirror.translate(2 * cx, 0);
    mirror.scale(-1, 1);

    Area body = new Area(half);
    body.add(new Area(mirror.createTransformedShape(half)));
    return body;
  }

  /** Recessed plates behind the D-pad and the two sticks, as on the real pad. */
  private void drawWells(Graphics2D g2) {
    g2.setColor(WELL);
    g2.fill(new Ellipse2D.Double(103, 92, 76, 76));
    g2.fill(new Ellipse2D.Double(158, 166, 56, 56));
    g2.fill(new Ellipse2D.Double(246, 166, 56, 56));

    g2.setColor(Theme.alpha(BODY_EDGE, 90));
    g2.setStroke(new BasicStroke(1f));
    g2.draw(new Ellipse2D.Double(103, 92, 76, 76));
    g2.draw(new Ellipse2D.Double(158, 166, 56, 56));
    g2.draw(new Ellipse2D.Double(246, 166, 56, 56));
  }

  /**
   * The D-pad as one cross, not four loose circles.
   *
   * <p>It is still four separate inputs underneath — the hit targets in LAYOUT sit on the arms — but
   * drawing it as the single rocker it physically is makes the picture match the pad.
   */
  private void drawDpad(Graphics2D g2) {
    double cx = 141;
    double cy = 130;
    double arm = 30;
    double half = 10;

    Path2D cross = new Path2D.Double();
    cross.moveTo(cx - half, cy - arm);
    cross.lineTo(cx + half, cy - arm);
    cross.lineTo(cx + half, cy - half);
    cross.lineTo(cx + arm, cy - half);
    cross.lineTo(cx + arm, cy + half);
    cross.lineTo(cx + half, cy + half);
    cross.lineTo(cx + half, cy + arm);
    cross.lineTo(cx - half, cy + arm);
    cross.lineTo(cx - half, cy + half);
    cross.lineTo(cx - arm, cy + half);
    cross.lineTo(cx - arm, cy - half);
    cross.lineTo(cx - half, cy - half);
    cross.closePath();

    g2.setColor(new Color(0x11151B));
    g2.fill(cross);
    g2.setColor(Theme.alpha(BODY_EDGE, 140));
    g2.setStroke(new BasicStroke(1.2f));
    g2.draw(cross);
  }

  /** The MODE button below the centre, where the real pad has it. */
  private void drawCentreCluster(Graphics2D g2) {
    g2.setColor(Theme.alpha(BODY_EDGE, 90));
    g2.setStroke(new BasicStroke(1.2f));
    g2.draw(new RoundRectangle2D.Double(218, 136, 24, 11, 11, 11));
    g2.setFont(Theme.ui(Font.PLAIN, 7f));
    g2.setColor(Theme.alpha(BODY_EDGE, 120));
    int tw = g2.getFontMetrics().stringWidth("MODE");
    g2.drawString("MODE", 230 - tw / 2, 145);
  }

  private void drawInputs(Graphics2D g2) {
    for (Map.Entry<String, Point> entry : LAYOUT.entrySet()) {
      String input = entry.getKey();
      Point at = entry.getValue();

      boolean isSelected = input.equals(selectedInput);
      boolean isHovered = input.equals(hoveredInput);
      boolean isBound = bindingsByInput.containsKey(input);
      boolean isPov = input.startsWith("POV_");
      int r = radiusFor(input);

      // Binding state is a ring, so the button underneath keeps its real colour.
      if (isSelected || isBound || isHovered) {
        g2.setColor(isSelected ? SELECTED : (isBound ? BOUND : Theme.alpha(LABEL, 110)));
        g2.setStroke(new BasicStroke(isSelected ? 3f : 2f));
        g2.draw(new Ellipse2D.Double(at.x - r - 3, at.y - r - 3, (r + 3) * 2, (r + 3) * 2));
      }

      // A D-pad direction is an arm of the cross already drawn underneath, not a separate button,
      // so it gets the arrow and the ring but no disc stamped on top of the cross.
      if (!isPov) {
        g2.setColor(faceColour(input, isBound));
        g2.fillOval(at.x - r, at.y - r, r * 2, r * 2);

        // A stick gets an inner disc, so it reads as a thumbstick rather than a button.
        if (input.endsWith("_STICK")) {
          g2.setColor(new Color(0x15191F));
          g2.fillOval(at.x - r + 5, at.y - r + 5, (r - 5) * 2, (r - 5) * 2);
        }
      }

      g2.setColor(isPov ? Theme.alpha(LABEL, isBound ? 255 : 165) : labelInk(input));
      g2.setFont(Theme.ui(Font.BOLD, r >= 13 ? 10f : 9f));
      String shortName = shortLabel(input);
      int tw = g2.getFontMetrics().stringWidth(shortName);
      g2.drawString(shortName, at.x - tw / 2, at.y + 4);
    }
  }

  /** The four face buttons keep their printed colours; everything else is neutral. */
  private static Color faceColour(String input, boolean bound) {
    Color base;
    switch (input) {
      case "Y": base = F310_Y; break;
      case "X": base = F310_X; break;
      case "B": base = F310_B; break;
      case "A": base = F310_A; break;
      case "LEFT_STICK":
      case "RIGHT_STICK": base = new Color(0x343B47); break;
      default: base = FREE;
    }
    // Unbound controls sit back a little without losing which button they are.
    return bound ? base : Theme.alpha(base, 150);
  }

  private static Color labelInk(String input) {
    switch (input) {
      case "Y": case "X": case "B": case "A": return INK;
      default: return INK;
    }
  }

  /**
   * Action names in a column beside each half of the pad, each joined to its button.
   *
   * <p>Labels are stacked top to bottom in the order of their buttons and nudged apart where two
   * buttons sit at nearly the same height (X and B, say), so none overlap.
   */
  private void drawLabels(Graphics2D g2) {
    java.util.List<String> left = new java.util.ArrayList<>();
    java.util.List<String> right = new java.util.ArrayList<>();
    for (String input : LAYOUT.keySet()) {
      if (bindingsByInput.containsKey(input)) {
        (LAYOUT.get(input).x < CENTRE_X ? left : right).add(input);
      }
    }
    drawColumn(g2, left, true);
    drawColumn(g2, right, false);
  }

  private void drawColumn(Graphics2D g2, java.util.List<String> inputs, boolean onLeft) {
    inputs.sort(java.util.Comparator.comparingInt((String input) -> LAYOUT.get(input).y)
        .thenComparingInt(input -> onLeft ? LAYOUT.get(input).x : -LAYOUT.get(input).x));

    int[] ys = new int[inputs.size()];
    for (int i = 0; i < inputs.size(); i++) {
      int want = LAYOUT.get(inputs.get(i)).y;
      ys[i] = i == 0 ? want : Math.max(want, ys[i - 1] + LABEL_SPACING);
    }
    // If the stack ran off the bottom, push it back up, keeping the spacing.
    int bottom = REF_H - 8;
    for (int i = ys.length - 1; i >= 0; i--) {
      if (ys[i] > bottom) {
        ys[i] = bottom;
      }
      bottom = ys[i] - LABEL_SPACING;
    }

    g2.setFont(Theme.ui(Font.PLAIN, 12.5f));
    java.awt.FontMetrics fm = g2.getFontMetrics();
    int maxWidth = onLeft ? LEFT_COLUMN + SHIFT - 10 : REF_W - SHIFT - RIGHT_COLUMN - 10;

    for (int i = 0; i < inputs.size(); i++) {
      String input = inputs.get(i);
      Point at = LAYOUT.get(input);
      boolean selected = input.equals(selectedInput);
      int r = radiusFor(input);
      int y = ys[i];

      String label = StatTile.ellipsize(fm, bindingsByInput.get(input), maxWidth);
      int width = fm.stringWidth(label);
      int textX = onLeft ? LEFT_COLUMN - width : RIGHT_COLUMN;
      int lineEnd = onLeft ? LEFT_COLUMN + 4 : RIGHT_COLUMN - 4;
      int lineStart = onLeft ? at.x - r - 6 : at.x + r + 6;
      int elbow = onLeft ? Math.min(lineStart - 6, lineEnd + 12) : Math.max(lineStart + 6, lineEnd - 12);

      g2.setColor(Theme.alpha(selected ? SELECTED : BOUND, 120));
      g2.setStroke(new BasicStroke(1f));
      g2.drawLine(lineStart, at.y, elbow, y);
      g2.drawLine(elbow, y, lineEnd, y);

      g2.setColor(selected ? SELECTED : LABEL);
      g2.drawString(label, textX, y + 5);
    }
  }

  /** Sticks are drawn larger than buttons, matching the real pad. */
  private static int radiusFor(String input) {
    if (input.endsWith("_STICK")) {
      return 17;
    }
    if (input.equals("BACK") || input.equals("START")) {
      return 9;
    }
    return 12;
  }

  private static String shortLabel(String input) {
    switch (input) {
      case "LEFT_TRIGGER": return "LT";
      case "RIGHT_TRIGGER": return "RT";
      case "LEFT_BUMPER": return "LB";
      case "RIGHT_BUMPER": return "RB";
      case "LEFT_STICK": return "LS";
      case "RIGHT_STICK": return "RS";
      case "POV_UP": return "▲";
      case "POV_DOWN": return "▼";
      case "POV_LEFT": return "◀";
      case "POV_RIGHT": return "▶";
      case "BACK": return "Bk";
      case "START": return "St";
      default: return input;
    }
  }
}
