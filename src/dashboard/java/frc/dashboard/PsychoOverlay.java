package frc.dashboard;

import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.Random;

import javax.swing.JComponent;

/**
 * What the dashboard does when the robot is in CYBERPSYCHO.
 *
 * <p>A glass pane painted over the whole window: scanlines, colour-split text, and bars that tear
 * across it. It is deliberately impossible to mistake for normal operation, because the mode it
 * reports is impossible to leave — if the driver has entered it, every second they spend wondering
 * whether they did is a second wasted.
 *
 * <p>It stays click-through. Everything underneath still works, and the operator can still use the
 * dashboard; this only changes how it looks.
 */
class PsychoOverlay extends JComponent {

  private static final Color RED = new Color(0xFF2D55);
  private static final Color CYAN = new Color(0x27E8FF);
  private static final Color SCANLINE = new Color(0, 0, 0, 38);

  private final Random random = new Random();
  private boolean active;
  private int frame;

  PsychoOverlay() {
    // The overlay must never swallow a click: the dashboard underneath stays usable.
    setOpaque(false);
    setFocusable(false);
  }

  void setActive(boolean active) {
    if (this.active != active) {
      this.active = active;
      setVisible(active);
    }
    if (active) {
      frame++;
      repaint();
    }
  }

  @Override
  protected void paintComponent(Graphics g) {
    if (!active) {
      return;
    }
    Graphics2D g2 = (Graphics2D) g.create();
    Theme.smooth(g2);

    int w = getWidth();
    int h = getHeight();

    // A red wash that breathes, so the whole window reads as wrong at a glance.
    float pulse = 0.10f + 0.06f * (float) Math.abs(Math.sin(frame * 0.18));
    g2.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, pulse));
    g2.setColor(RED);
    g2.fillRect(0, 0, w, h);
    g2.setComposite(AlphaComposite.SrcOver);

    // Scanlines.
    g2.setColor(SCANLINE);
    for (int y = (frame * 2) % 4; y < h; y += 4) {
      g2.drawLine(0, y, w, y);
    }

    // Tear bars: a few slices displaced sideways, as though the signal is breaking up.
    for (int i = 0; i < 3; i++) {
      int y = random.nextInt(Math.max(1, h));
      int bandHeight = 6 + random.nextInt(26);
      int shift = random.nextInt(40) - 20;
      g2.setColor(i % 2 == 0 ? Theme.alpha(CYAN, 70) : Theme.alpha(RED, 70));
      g2.fillRect(shift, y, w, bandHeight);
    }

    // Border that strobes.
    g2.setStroke(new BasicStroke(5f));
    g2.setColor(frame % 8 < 4 ? RED : CYAN);
    g2.drawRect(2, 2, w - 5, h - 5);

    drawBanner(g2, w, h);
    g2.dispose();
  }

  /** Title in split colour, the way a broken signal doubles an image. */
  private void drawBanner(Graphics2D g2, int w, int h) {
    String text = "C Y B E R P S Y C H O";
    g2.setFont(Theme.ui(Font.BOLD, 46f));
    int textWidth = g2.getFontMetrics().stringWidth(text);
    int x = (w - textWidth) / 2;
    int y = h / 2;
    int split = 3 + random.nextInt(4);

    g2.setColor(Theme.alpha(RED, 200));
    g2.drawString(text, x - split, y);
    g2.setColor(Theme.alpha(CYAN, 200));
    g2.drawString(text, x + split, y);
    g2.setColor(Color.WHITE);
    g2.drawString(text, x, y);

    String sub = "ALL POWER TO THE WHEELS  -  NO WAY OUT BUT A REBOOT";
    g2.setFont(Theme.ui(Font.PLAIN, 15f));
    int subWidth = g2.getFontMetrics().stringWidth(sub);
    g2.setColor(Theme.alpha(Color.WHITE, 190));
    g2.drawString(sub, (w - subWidth) / 2, y + 34);
  }
}
