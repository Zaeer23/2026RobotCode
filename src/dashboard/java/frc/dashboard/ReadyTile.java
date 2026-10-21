package frc.dashboard;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.GradientPaint;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.RoundRectangle2D;

import javax.swing.Icon;
import javax.swing.JComponent;

/**
 * The one light worth watching mid-match: can we shoot right now.
 *
 * <p>When the answer is no, it says why. Three conditions have to line up (a hub tag in view, the
 * turret on it, the flywheel at speed) and each gets its own chip, so the operator sees which one
 * is holding the shot up instead of staring at a single red light.
 */
public class ReadyTile extends JComponent {

  private boolean ready;
  private boolean target;
  private boolean turret;
  private boolean speed;
  private boolean connected;

  public ReadyTile() {
    setPreferredSize(new Dimension(340, 108));
    setMinimumSize(new Dimension(260, 96));
  }

  public void update(boolean isConnected, boolean isReady, boolean hasTarget, boolean onTarget,
      boolean atSpeed) {
    if (isConnected != connected || isReady != ready || hasTarget != target
        || onTarget != turret || atSpeed != speed) {
      connected = isConnected;
      ready = isReady;
      target = hasTarget;
      turret = onTarget;
      speed = atSpeed;
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

    boolean lit = connected && ready;
    if (lit) {
      g2.setPaint(new GradientPaint(0, 0, Theme.alpha(Theme.GOOD, 70), w, h,
          Theme.alpha(Theme.GOOD, 22)));
      g2.fill(shape);
      g2.setColor(Theme.alpha(Theme.GOOD, 200));
      g2.draw(shape);
    } else {
      g2.setColor(Theme.SURFACE);
      g2.fill(shape);
      g2.setColor(Theme.BORDER);
      g2.draw(shape);
    }

    int pad = 18;
    g2.setFont(Theme.caption(11f));
    g2.setColor(lit ? Theme.GOOD : Theme.TEXT_DIM);
    g2.drawString("SHOT", pad, 26);

    String headline = !connected ? "NO ROBOT" : (ready ? "READY TO SHOOT" : "NOT READY");
    float size = 28f;
    g2.setFont(Theme.numeric(java.awt.Font.BOLD, size));
    while (size > 16f && g2.getFontMetrics().stringWidth(headline) > w - 2 * pad) {
      size -= 2f;
      g2.setFont(Theme.numeric(java.awt.Font.BOLD, size));
    }
    g2.setColor(lit ? Theme.mix(Theme.GOOD, Theme.TEXT, 0.15) : (connected ? Theme.TEXT : Theme.TEXT_FAINT));
    g2.drawString(headline, pad, 62);

    int x = pad;
    int y = h - 34;
    x = chip(g2, x, y, "Hub tag", target);
    x = chip(g2, x + 8, y, "Turret aimed", turret);
    chip(g2, x + 8, y, "At speed", speed);
    g2.dispose();
  }

  private int chip(Graphics2D g2, int x, int y, String label, boolean ok) {
    boolean live = connected;
    Color colour = !live ? Theme.TEXT_FAINT : (ok ? Theme.GOOD : Theme.TEXT_DIM);
    g2.setFont(Theme.ui(java.awt.Font.BOLD, 12f));
    FontMetrics fm = g2.getFontMetrics();
    int w = fm.stringWidth(label) + 38;
    int h = 22;
    RoundRectangle2D shape = new RoundRectangle2D.Double(x + 0.5, y + 0.5, w - 1.0, h - 1.0, h, h);
    g2.setColor(ok && live ? Theme.alpha(Theme.GOOD, 34) : Theme.alpha(Theme.TEXT_FAINT, 26));
    g2.fill(shape);
    g2.setColor(ok && live ? Theme.alpha(Theme.GOOD, 110) : Theme.alpha(Theme.TEXT_FAINT, 70));
    g2.draw(shape);

    Icon icon = ok ? Icons.check(12, colour) : Icons.cross(12, colour);
    icon.paintIcon(null, g2, x + 9, y + 5);
    g2.setColor(ok && live ? Theme.mix(Theme.GOOD, Theme.TEXT, 0.3) : colour);
    g2.drawString(label, x + 26, y + (h + fm.getAscent() - fm.getDescent()) / 2);
    return x + w;
  }
}
