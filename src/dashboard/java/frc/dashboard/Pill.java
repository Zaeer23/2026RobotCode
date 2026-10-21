package frc.dashboard;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.RoundRectangle2D;

import javax.swing.JComponent;

/**
 * A small rounded status label: a coloured dot and a word or two.
 *
 * <p>Tinted rather than solid by default, so a row of them reads calmly and the solid ones (used
 * for the states that need a reaction) stand out.
 */
public class Pill extends JComponent {

  private String text;
  private Color colour;
  private boolean solid = false;
  private boolean dot = true;

  public Pill(String text, Color colour) {
    this.text = text;
    this.colour = colour;
    setFont(Theme.caption(11f));
  }

  /** Updates the pill; only repaints and re-lays out when something actually changed. */
  public void set(String newText, Color newColour) {
    if (!newText.equals(text) || !newColour.equals(colour)) {
      boolean resized = !newText.equals(text);
      text = newText;
      colour = newColour;
      if (resized) {
        revalidate();
      }
      repaint();
    }
  }

  public Pill solid(boolean on) {
    if (solid != on) {
      solid = on;
      repaint();
    }
    return this;
  }

  public Pill dot(boolean on) {
    dot = on;
    return this;
  }

  @Override
  public Dimension getPreferredSize() {
    FontMetrics fm = getFontMetrics(getFont());
    int width = fm.stringWidth(text) + (dot ? 30 : 22);
    return new Dimension(width, 26);
  }

  @Override
  public Dimension getMaximumSize() {
    return getPreferredSize();
  }

  @Override
  protected void paintComponent(Graphics g) {
    Graphics2D g2 = (Graphics2D) g.create();
    Theme.smooth(g2);
    int h = 24;
    int y = (getHeight() - h) / 2;
    RoundRectangle2D shape = new RoundRectangle2D.Double(0.5, y + 0.5, getWidth() - 1.0, h - 1.0, h, h);

    if (solid) {
      g2.setColor(colour);
      g2.fill(shape);
    } else {
      g2.setColor(Theme.alpha(colour, 30));
      g2.fill(shape);
      g2.setColor(Theme.alpha(colour, 90));
      g2.draw(shape);
    }

    Color ink = solid ? Theme.TEXT_ON_ACCENT : Theme.mix(colour, Theme.TEXT, 0.25);
    int x = 11;
    if (dot) {
      g2.setColor(solid ? Theme.TEXT_ON_ACCENT : colour);
      g2.fill(new Ellipse2D.Double(x, y + h / 2.0 - 3.5, 7, 7));
      x += 13;
    }
    g2.setFont(getFont());
    g2.setColor(ink);
    FontMetrics fm = g2.getFontMetrics();
    g2.drawString(text, x, y + (h + fm.getAscent() - fm.getDescent()) / 2);
    g2.dispose();
  }
}
