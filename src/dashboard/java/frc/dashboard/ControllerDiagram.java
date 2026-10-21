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
import java.awt.geom.RoundRectangle2D;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Consumer;

import javax.swing.JPanel;
import javax.swing.event.MouseInputAdapter;

/**
 * Logitech F310 gamepad diagram showing what each input is bound to.
 *
 * <p>Drawn to the F310's actual layout, which is the PlayStation arrangement rather than the Xbox
 * one: both sticks sit together at the BOTTOM, the D-pad is upper-left, the face buttons are
 * upper-right. An earlier version used the Xbox layout, with a stick up where the D-pad belongs,
 * which makes the picture actively misleading — the whole value of a diagram over a list is that
 * it maps onto the thing in your hands without translation.
 *
 * <p>Bound inputs are lit and labelled; free ones are dim. Labels sit in a column beside whichever
 * half of the body the control is on, joined to it by a leader line, so they never cover the pad
 * or a neighbouring button even when the face buttons are all bound. Clicking selects an input,
 * which is how the editor beside it chooses what to rebind.
 */
public class ControllerDiagram extends JPanel {

  private static final Color INK = Theme.TEXT_ON_ACCENT;
  private static final Color BODY = new Color(0x262C37);
  private static final Color BODY_EDGE = new Color(0x3C4452);
  private static final Color WELL = new Color(0x1C2129);
  private static final Color FREE = new Color(0x4B5362);
  private static final Color BOUND = Theme.ACCENT;
  private static final Color SELECTED = Theme.GOOD;
  private static final Color LABEL = Theme.TEXT;
  private static final Color DIM = Theme.TEXT_DIM;

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
    LAYOUT.put("LEFT_TRIGGER", new Point(132, 30));
    LAYOUT.put("RIGHT_TRIGGER", new Point(328, 30));
    LAYOUT.put("LEFT_BUMPER", new Point(132, 62));
    LAYOUT.put("RIGHT_BUMPER", new Point(328, 62));

    // D-pad, upper LEFT on an F310.
    LAYOUT.put("POV_UP", new Point(140, 108));
    LAYOUT.put("POV_LEFT", new Point(116, 132));
    LAYOUT.put("POV_RIGHT", new Point(164, 132));
    LAYOUT.put("POV_DOWN", new Point(140, 156));

    // Face buttons, upper RIGHT, in the F310's diamond.
    LAYOUT.put("Y", new Point(320, 108));
    LAYOUT.put("X", new Point(296, 132));
    LAYOUT.put("B", new Point(344, 132));
    LAYOUT.put("A", new Point(320, 156));

    // Centre cluster.
    LAYOUT.put("BACK", new Point(206, 126));
    LAYOUT.put("START", new Point(254, 126));

    // Both sticks together at the BOTTOM. The F310's defining difference from an Xbox pad.
    LAYOUT.put("LEFT_STICK", new Point(186, 196));
    LAYOUT.put("RIGHT_STICK", new Point(274, 196));
  }

  private Map<String, String> bindingsByInput = Map.of();
  private String selectedInput;
  private Consumer<String> onSelect = input -> { };
  private String title = "Controller";

  public ControllerDiagram() {
    setPreferredSize(new Dimension(REF_W, REF_H));
    setOpaque(false);
    setCursor(java.awt.Cursor.getPredefinedCursor(java.awt.Cursor.HAND_CURSOR));

    addMouseListener(new MouseInputAdapter() {
      @Override
      public void mousePressed(java.awt.event.MouseEvent e) {
        String hit = inputAt(e.getX(), e.getY());
        if (hit != null) {
          selectedInput = hit;
          onSelect.accept(hit);
          repaint();
        }
      }
    });
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
    drawInputs(g2);
    drawLabels(g2);

    g2.dispose();
  }

  /** The F310 silhouette: a wide body with two grips angling down and outward. */
  private void drawBody(Graphics2D g2) {
    Area body = new Area(new RoundRectangle2D.Double(96, 78, 268, 112, 56, 56));
    body.add(new Area(gripShape(true)));
    body.add(new Area(gripShape(false)));

    // Shoulder blocks, so bumpers and triggers read as being on the far edge.
    body.add(new Area(new RoundRectangle2D.Double(100, 18, 66, 62, 22, 22)));
    body.add(new Area(new RoundRectangle2D.Double(294, 18, 66, 62, 22, 22)));

    g2.setColor(BODY);
    g2.fill(body);
    g2.setColor(BODY_EDGE);
    g2.setStroke(new BasicStroke(1.6f));
    g2.draw(body);
  }

  private java.awt.Shape gripShape(boolean left) {
    RoundRectangle2D grip = new RoundRectangle2D.Double(-30, 0, 60, 104, 34, 34);
    AffineTransform at = new AffineTransform();
    at.translate(left ? 160 : 300, 150);
    at.rotate(Math.toRadians(left ? -16 : 16));
    return at.createTransformedShape(grip);
  }

  /** Recessed plates behind the D-pad and the two sticks, as on the real pad. */
  private void drawWells(Graphics2D g2) {
    g2.setColor(WELL);
    g2.fill(new Ellipse2D.Double(102, 94, 76, 76));
    g2.fill(new Ellipse2D.Double(160, 170, 52, 52));
    g2.fill(new Ellipse2D.Double(248, 170, 52, 52));
  }

  private void drawInputs(Graphics2D g2) {
    for (Map.Entry<String, Point> entry : LAYOUT.entrySet()) {
      String input = entry.getKey();
      Point at = entry.getValue();
      String action = bindingsByInput.get(input);

      boolean isSelected = input.equals(selectedInput);
      boolean isBound = action != null;
      int r = radiusFor(input);

      if (isSelected) {
        g2.setColor(SELECTED.darker());
        g2.fillOval(at.x - r - 4, at.y - r - 4, (r + 4) * 2, (r + 4) * 2);
      }

      g2.setColor(isSelected ? SELECTED : (isBound ? BOUND : FREE));
      g2.fillOval(at.x - r, at.y - r, r * 2, r * 2);

      g2.setColor(INK);
      g2.setFont(Theme.ui(Font.BOLD, r >= 13 ? 10f : 9f));
      String shortName = shortLabel(input);
      int tw = g2.getFontMetrics().stringWidth(shortName);
      g2.drawString(shortName, at.x - tw / 2, at.y + 4);

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
      int lineStart = onLeft ? at.x - r - 2 : at.x + r + 2;
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
      case "POV_UP": return "Up";
      case "POV_DOWN": return "Dn";
      case "POV_LEFT": return "Lt";
      case "POV_RIGHT": return "Rt";
      case "BACK": return "Bk";
      case "START": return "St";
      default: return input;
    }
  }
}
