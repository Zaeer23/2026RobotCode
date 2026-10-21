package frc.dashboard;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.RoundRectangle2D;
import java.util.Objects;

import javax.swing.JComponent;

/**
 * One number worth watching mid-match: a caption, a big value, and a line of context under it.
 *
 * <p>The value is drawn in a fixed-width numeric face so it does not wobble as it changes. An
 * optional bar shows progress toward a target (shooter speed toward its setpoint, say), which is
 * faster to read than comparing two numbers.
 */
public class StatTile extends JComponent {

  private final String caption;
  private String value = "-";
  private String unit = "";
  private String detail = "";
  private Color valueColour = Theme.TEXT;
  private Color status = null;
  private double progress = Double.NaN;

  public StatTile(String caption) {
    this.caption = caption.toUpperCase();
    setPreferredSize(new Dimension(170, 108));
    setMinimumSize(new Dimension(120, 96));
  }

  /**
   * @param status colour of the dot in the corner, or null for none
   * @param progress 0..1 for the bar, or NaN to hide it
   */
  public void update(String newValue, String newUnit, String newDetail, Color newValueColour,
      Color newStatus, double newProgress) {
    boolean changed = !Objects.equals(value, newValue)
        || !Objects.equals(unit, newUnit)
        || !Objects.equals(detail, newDetail)
        || !Objects.equals(valueColour, newValueColour)
        || !Objects.equals(status, newStatus)
        || Double.compare(progress, newProgress) != 0;
    if (changed) {
      value = newValue;
      unit = newUnit;
      detail = newDetail;
      valueColour = newValueColour;
      status = newStatus;
      progress = newProgress;
      repaint();
    }
  }

  @Override
  protected void paintComponent(Graphics g) {
    Graphics2D g2 = (Graphics2D) g.create();
    Theme.smooth(g2);
    int w = getWidth();
    int h = getHeight();

    RoundRectangle2D shape = new RoundRectangle2D.Double(0.5, 0.5, w - 1.0, h - 1.0,
        Theme.RADIUS, Theme.RADIUS);
    g2.setColor(Theme.SURFACE);
    g2.fill(shape);
    g2.setColor(status != null ? Theme.alpha(status, 70) : Theme.BORDER);
    g2.draw(shape);

    int pad = 16;

    g2.setFont(Theme.caption(11f));
    g2.setColor(Theme.TEXT_DIM);
    g2.drawString(caption, pad, 26);

    if (status != null) {
      g2.setColor(Theme.alpha(status, 60));
      g2.fill(new Ellipse2D.Double(w - pad - 12, 15, 12, 12));
      g2.setColor(status);
      g2.fill(new Ellipse2D.Double(w - pad - 9, 18, 6, 6));
    }

    // Value, scaled down if it would not fit the tile.
    float size = 32f;
    g2.setFont(Theme.numeric(java.awt.Font.BOLD, size));
    FontMetrics fm = g2.getFontMetrics();
    g2.setFont(Theme.ui(java.awt.Font.PLAIN, 14f));
    int unitWidth = unit.isEmpty() ? 0 : g2.getFontMetrics().stringWidth(unit) + 6;
    while (size > 16f && fm.stringWidth(value) + unitWidth > w - 2 * pad) {
      size -= 2f;
      fm = g2.getFontMetrics(Theme.numeric(java.awt.Font.BOLD, size));
    }
    int baseline = 66;
    g2.setFont(Theme.numeric(java.awt.Font.BOLD, size));
    g2.setColor(valueColour);
    g2.drawString(value, pad, baseline);

    if (!unit.isEmpty()) {
      g2.setFont(Theme.ui(java.awt.Font.PLAIN, 14f));
      g2.setColor(Theme.TEXT_DIM);
      g2.drawString(unit, pad + fm.stringWidth(value) + 6, baseline);
    }

    int bottom = h - 14;
    if (!Double.isNaN(progress)) {
      int barY = baseline + 10;
      int barW = w - 2 * pad;
      g2.setColor(Theme.SURFACE_HOVER);
      g2.fill(new RoundRectangle2D.Double(pad, barY, barW, 4, 4, 4));
      double clamped = Math.max(0.0, Math.min(1.0, progress));
      if (clamped > 0) {
        g2.setColor(valueColour.equals(Theme.TEXT) ? Theme.ACCENT : valueColour);
        g2.fill(new RoundRectangle2D.Double(pad, barY, Math.max(4, barW * clamped), 4, 4, 4));
      }
    }

    if (!detail.isEmpty()) {
      g2.setFont(Theme.ui(java.awt.Font.PLAIN, 12f));
      g2.setColor(Theme.TEXT_DIM);
      g2.drawString(ellipsize(g2.getFontMetrics(), detail, w - 2 * pad), pad, bottom);
    }
    g2.dispose();
  }

  static String ellipsize(FontMetrics fm, String text, int width) {
    if (fm.stringWidth(text) <= width) {
      return text;
    }
    String ellipsis = "…";
    int end = text.length();
    while (end > 0 && fm.stringWidth(text.substring(0, end) + ellipsis) > width) {
      end--;
    }
    return text.substring(0, end) + ellipsis;
  }
}
