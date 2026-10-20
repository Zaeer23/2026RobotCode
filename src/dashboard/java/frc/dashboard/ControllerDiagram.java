package frc.dashboard;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.RenderingHints;
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
 * <p>Bound inputs are lit and labelled; free ones are dim. Labels sit outside whichever half of
 * the body the control is on, so they never cover the pad or a neighbouring button. Clicking
 * selects an input, which is how the editor beside it chooses what to rebind.
 */
public class ControllerDiagram extends JPanel {

  private static final Color BG = new Color(0x1C, 0x1F, 0x26);
  private static final Color BODY = new Color(0x2E, 0x33, 0x3C);
  private static final Color BODY_EDGE = new Color(0x45, 0x4C, 0x58);
  private static final Color WELL = new Color(0x23, 0x27, 0x2E);
  private static final Color FREE = new Color(0x51, 0x58, 0x63);
  private static final Color BOUND = new Color(0x4C, 0xC2, 0xFF);
  private static final Color SELECTED = new Color(0x3D, 0xDC, 0x84);
  private static final Color LABEL = new Color(0xC8, 0xCE, 0xD6);
  private static final Color DIM = new Color(0x8A, 0x90, 0x98);

  /** Reference canvas. All geometry below is in these coordinates, scaled to fit the panel. */
  private static final int REF_W = 460;
  private static final int REF_H = 300;

  /** Centre line of the pad, used to decide which side a label goes on. */
  private static final int CENTRE_X = 230;

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
    setBackground(BG);

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
      double px = offsetX() + entry.getValue().x * s;
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
    g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

    g2.translate(offsetX(), offsetY());
    g2.scale(scale(), scale());

    drawBody(g2);
    drawWells(g2);
    drawInputs(g2);

    g2.setFont(new Font("Segoe UI", Font.BOLD, 12));
    g2.setColor(DIM);
    g2.drawString(title, 8, 16);

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

      g2.setColor(BG);
      g2.setFont(new Font("Segoe UI", Font.BOLD, r >= 13 ? 10 : 9));
      String shortName = shortLabel(input);
      int tw = g2.getFontMetrics().stringWidth(shortName);
      g2.drawString(shortName, at.x - tw / 2, at.y + 4);

      if (isBound) {
        drawActionLabel(g2, at, action);
      }
    }
  }

  /**
   * Action name placed outside the body.
   *
   * <p>Left-half controls label leftwards and right-half controls rightwards, so text never lands
   * on the pad or on a neighbouring button.
   */
  private void drawActionLabel(Graphics2D g2, Point at, String action) {
    g2.setColor(LABEL);
    g2.setFont(new Font("Segoe UI", Font.PLAIN, 9));

    String label = action.length() > 18 ? action.substring(0, 17) + "..." : action;
    int textWidth = g2.getFontMetrics().stringWidth(label);
    boolean onLeft = at.x < CENTRE_X;

    int x = onLeft ? at.x - 16 - textWidth : at.x + 16;
    g2.drawString(label, x, at.y + 3);
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
