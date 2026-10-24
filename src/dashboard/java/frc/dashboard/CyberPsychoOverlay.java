package frc.dashboard;

import java.awt.AlphaComposite;
import java.awt.Color;
import java.awt.Composite;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Paint;
import java.awt.Rectangle;
import java.awt.TexturePaint;
import java.awt.image.BufferedImage;

import javax.swing.JComponent;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;

/**
 * The dashboard going haywire, which is how the drive team knows the robot is in CyberPsycho.
 *
 * <p>Installed as the window's glass pane, so it draws over every tab, the header and the title
 * bar. It registers no mouse or key listeners, so clicks still go straight through to whatever
 * is underneath: the dashboard stays usable, it just looks like it is coming apart.
 *
 * <p>Two phases. For {@link #INTRO_SECONDS} after the robot reports the mode, everything is
 * thrown at the screen: the window tears into displaced slices, washes between magenta and cyan,
 * and the title fills the middle. After that it settles to a steady, lighter haywire with a hard
 * glitch burst every few seconds, because in this mode the camera view is the driver's view of the
 * field and it has to stay watchable. The mode cannot be exited, so neither can this; it goes
 * away only when the robot stops reporting it, which means the robot program restarted.
 *
 * <p>The tearing is {@link Graphics#copyArea} on the frame Swing has just painted underneath,
 * so it displaces the real dashboard, not a picture of it. Nothing in the paint path allocates:
 * colours, composites, the scanline texture and the text widths are all made once, and the
 * randomness is a xorshift held in a field. It is repainted from the dashboard's existing refresh
 * timer, so there is no second timer, and it is invisible (and costs nothing) until the mode
 * starts.
 */
final class CyberPsychoOverlay extends JComponent {

  /** How long the full-strength burst lasts once the mode starts. */
  private static final double INTRO_SECONDS = 3.0;

  /** After the intro, a hard glitch burst this often... */
  private static final double BURST_PERIOD_SECONDS = 2.6;

  /** ...lasting this long. */
  private static final double BURST_SECONDS = 0.35;

  private static final String TITLE = "CYBERPSYCHO";
  private static final String SUBTITLE = "ALL POWER TO THE WHEELS  ·  NO WAY BACK";

  private static final Color MAGENTA = new Color(0xFF2BD6);
  private static final Color CYAN = new Color(0x2BF6FF);
  private static final Color RED = new Color(0xFF1744);
  private static final Color WHITE = new Color(0xFFFFFF);
  private static final Color BLACK = new Color(0x000000);
  private static final Color[] NOISE = {MAGENTA, CYAN, RED, WHITE, new Color(0xFFE600), BLACK};

  /** Alpha in sixteenths, so a varying transparency never needs a new composite. */
  private static final AlphaComposite[] ALPHA = new AlphaComposite[17];

  static {
    for (int i = 0; i <= 16; i++) {
      ALPHA[i] = AlphaComposite.getInstance(AlphaComposite.SRC_OVER, i / 16f);
    }
  }

  private final TexturePaint scanlines;
  private final Font titleFont = Theme.ui(Font.BOLD, 96f);
  private final Font bannerFont = Theme.ui(Font.BOLD, 30f);
  private final Font subtitleFont = Theme.caption(16f);

  // Measured on the first paint; the strings and fonts never change after.
  private int titleWidth = -1;
  private int titleAscent;
  private int bannerWidth;
  private int bannerAscent;
  private int subtitleWidth;
  private int bannerDescent;

  /**
   * The settled banner sits in the tab row, the empty strip just under this component. Found from
   * the component's real position rather than a fixed offset, because a native or FlatLaf title
   * bar moves everything down by a different amount on each machine.
   */
  private JComponent bannerAnchor;
  private int bannerBaseline = -1;
  private int laidOutWidth = -1;
  private int laidOutHeight = -1;

  private boolean active = false;
  private long activatedNanos;
  private long seed = 0x9E3779B97F4A7C15L;

  CyberPsychoOverlay() {
    setOpaque(false);
    setFocusable(false);
    setVisible(false);

    // One dark line in every three: a CRT that has seen better days.
    BufferedImage tile = new BufferedImage(1, 3, BufferedImage.TYPE_INT_ARGB);
    tile.setRGB(0, 2, 0x5A000000);
    scanlines = new TexturePaint(tile, new Rectangle(0, 0, 1, 3));
  }

  /**
   * Takes the robot's latest word on the mode. Call from the refresh timer.
   *
   * <p>Follows the robot rather than latching here, so a restarted robot program, the only real
   * way out of CyberPsycho, clears the dashboard too. A dropped connection does not: the caller
   * only passes what the robot last said.
   */
  void update(boolean robotActive) {
    if (robotActive && !active) {
      active = true;
      activatedNanos = System.nanoTime();
      setVisible(true);
    } else if (!robotActive && active) {
      active = false;
      setVisible(false);
    }
    if (active) {
      repaint();
    }
  }

  /** Puts the settled banner in the strip directly below {@code anchor}, the header. */
  void placeBannerBelow(JComponent anchor) {
    bannerAnchor = anchor;
    laidOutWidth = -1;
  }

  /** True while the dashboard is showing the mode. */
  boolean isActive() {
    return active;
  }

  @Override
  protected void paintComponent(Graphics g) {
    if (!active) {
      return;
    }
    Graphics2D g2 = (Graphics2D) g;
    Composite savedComposite = g2.getComposite();
    Paint savedPaint = g2.getPaint();
    Font savedFont = g2.getFont();

    if (titleWidth < 0) {
      measure(g2);
    }
    int w = getWidth();
    int h = getHeight();
    if (w <= 0 || h <= 0) {
      return;
    }
    if (w != laidOutWidth || h != laidOutHeight) {
      layOutBanner(h);
      laidOutWidth = w;
      laidOutHeight = h;
    }

    double t = (System.nanoTime() - activatedNanos) / 1e9;
    boolean intro = t < INTRO_SECONDS;
    boolean burst = !intro && (t % BURST_PERIOD_SECONDS) < BURST_SECONDS;

    tear(g2, w, h, intro ? 14 : burst ? 7 : (rand(4) == 0 ? 1 : 0), intro ? 90 : 40);
    wash(g2, w, h, intro, burst);
    noise(g2, w, h, intro ? 46 : burst ? 20 : 4);

    g2.setComposite(ALPHA[16]);
    g2.setPaint(scanlines);
    g2.fillRect(0, 0, w, h);

    border(g2, w, h, t);

    if (intro) {
      title(g2, w, h, t);
    } else {
      banner(g2, w, h, burst);
    }

    g2.setComposite(savedComposite);
    g2.setPaint(savedPaint);
    g2.setFont(savedFont);
  }

  /** Shoves horizontal slices of what is already on screen sideways. */
  private void tear(Graphics2D g2, int w, int h, int slices, int maxShift) {
    for (int i = 0; i < slices; i++) {
      int y = rand(h);
      int sliceHeight = 3 + rand(Math.max(1, h / 18));
      int dx = rand(2 * maxShift + 1) - maxShift;
      g2.copyArea(0, y, w, Math.min(sliceHeight, h - y), dx, 0);
    }
  }

  /** A colour cast over the whole window: flickering hard in the intro, a faint red after. */
  private void wash(Graphics2D g2, int w, int h, boolean intro, boolean burst) {
    if (intro) {
      // An occasional near-white frame reads as the screen shorting out.
      boolean flash = rand(7) == 0;
      g2.setComposite(ALPHA[flash ? 11 : 3 + rand(4)]);
      g2.setColor(flash ? WHITE : (rand(2) == 0 ? MAGENTA : CYAN));
    } else {
      g2.setComposite(ALPHA[burst ? 3 + rand(3) : 1]);
      g2.setColor(burst ? (rand(2) == 0 ? MAGENTA : CYAN) : RED);
    }
    g2.fillRect(0, 0, w, h);
  }

  /** Blocks of dead pixels. */
  private void noise(Graphics2D g2, int w, int h, int count) {
    for (int i = 0; i < count; i++) {
      g2.setComposite(ALPHA[8 + rand(9)]);
      g2.setColor(NOISE[rand(NOISE.length)]);
      g2.fillRect(rand(w), rand(h), 8 + rand(140), 2 + rand(12));
    }
  }

  /** A pulsing red frame round the window, made of four rectangles rather than a stroke. */
  private void border(Graphics2D g2, int w, int h, double t) {
    int pulse = (int) Math.round(8 + 8 * Math.abs(Math.sin(t * 5.0)));
    int thick = 6;
    g2.setComposite(ALPHA[Math.min(16, pulse)]);
    g2.setColor(RED);
    g2.fillRect(0, 0, w, thick);
    g2.fillRect(0, h - thick, w, thick);
    g2.fillRect(0, 0, thick, h);
    g2.fillRect(w - thick, 0, thick, h);
  }

  /** The intro title, dead centre, split into red and cyan ghosts that jitter apart. */
  private void title(Graphics2D g2, int w, int h, double t) {
    int x = (w - titleWidth) / 2 + rand(13) - 6;
    int y = (h + titleAscent) / 2 - 20 + rand(9) - 4;
    int split = 4 + rand(14);

    g2.setFont(titleFont);
    rgbSplit(g2, TITLE, x, y, split);

    g2.setFont(subtitleFont);
    g2.setComposite(ALPHA[rand(5) == 0 ? 6 : 16]);
    g2.setColor(WHITE);
    g2.drawString(SUBTITLE, (w - subtitleWidth) / 2, y + 44);

    // Tear straight through the title as well, so it does not sit there looking tidy.
    tear(g2, w, h, 3, 30);
  }

  /** After the intro: a smaller title in the empty tab-row strip, out of the camera's way. */
  private void banner(Graphics2D g2, int w, int h, boolean burst) {
    int x = (w - bannerWidth) / 2 + (burst ? rand(11) - 5 : 0);
    int y = bannerBaseline;
    g2.setFont(bannerFont);
    rgbSplit(g2, TITLE, x, y, burst ? 3 + rand(10) : 2);
  }

  private void rgbSplit(Graphics2D g2, String text, int x, int y, int split) {
    g2.setComposite(ALPHA[12]);
    g2.setColor(RED);
    g2.drawString(text, x - split, y);
    g2.setColor(CYAN);
    g2.drawString(text, x + split, y + (split / 3));
    g2.setComposite(ALPHA[16]);
    g2.setColor(WHITE);
    g2.drawString(text, x, y);
  }

  /**
   * Works out the banner's baseline: vertically centred in the tab row under the anchor, or near
   * the bottom edge if there is no anchor. Only on a resize, since it allocates.
   */
  private void layOutBanner(int h) {
    if (bannerAnchor != null && bannerAnchor.isShowing()) {
      Rectangle header = SwingUtilities.convertRectangle(
          bannerAnchor.getParent(), bannerAnchor.getBounds(), this);
      int rowHeight = Math.max(UIManager.getInt("TabbedPane.tabHeight"), bannerAscent);
      int rowCentre = header.y + header.height + rowHeight / 2;
      bannerBaseline = rowCentre + (bannerAscent - bannerDescent) / 2;
    } else {
      bannerBaseline = h - 22 - bannerDescent;
    }
  }

  private void measure(Graphics2D g2) {
    FontMetrics title = g2.getFontMetrics(titleFont);
    titleWidth = title.stringWidth(TITLE);
    titleAscent = title.getAscent();
    FontMetrics banner = g2.getFontMetrics(bannerFont);
    bannerWidth = banner.stringWidth(TITLE);
    bannerAscent = banner.getAscent();
    bannerDescent = banner.getDescent();
    subtitleWidth = g2.getFontMetrics(subtitleFont).stringWidth(SUBTITLE);
  }

  /** Uniform in [0, bound). Xorshift64: fast, allocation-free, and plenty random to look broken. */
  private int rand(int bound) {
    if (bound <= 1) {
      return 0;
    }
    seed ^= seed << 13;
    seed ^= seed >>> 7;
    seed ^= seed << 17;
    return (int) Long.remainderUnsigned(seed, bound);
  }
}
