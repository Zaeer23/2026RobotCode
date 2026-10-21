package frc.dashboard;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.GradientPaint;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridBagLayout;
import java.awt.image.BufferedImage;
import java.awt.geom.RoundRectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;

/**
 * The strip across the top of every tab: is the robot there, what mode is it in, which auto will
 * it run, how long is left, and how is the battery.
 *
 * <p>These are the things worth knowing whichever tab is open, so they live here rather than on
 * the Match tab, and the match clock and battery are big enough to read from the driver's seat.
 */
public class HeaderBar extends JPanel {

  private final Pill connection = new Pill("CONNECTING", Theme.TEXT_FAINT);
  private final Pill mode = new Pill("DISABLED", Theme.TEXT_FAINT);
  private final Pill alliance = new Pill("NO ALLIANCE", Theme.TEXT_FAINT);
  private final Pill fms = new Pill("FMS", Theme.ACCENT);

  private final JLabel autoName = Ui.label("-", Theme.ui(Font.BOLD, 14f), Theme.TEXT);
  private final JLabel clock = Ui.label("--:--", Theme.numeric(Font.BOLD, 28f), Theme.TEXT_FAINT);
  private final BatteryGauge battery = new BatteryGauge();

  public HeaderBar(int team) {
    super(new BorderLayout(20, 0));
    setBackground(Theme.SURFACE);
    setBorder(BorderFactory.createEmptyBorder(12, 18, 10, 20));

    // Brand.
    JPanel brand = new JPanel(new FlowLayout(FlowLayout.LEFT, 12, 0));
    brand.setOpaque(false);
    brand.add(new Badge(team, 42));
    JPanel titles = new JPanel();
    titles.setLayout(new BoxLayout(titles, BoxLayout.Y_AXIS));
    titles.setOpaque(false);
    titles.add(Ui.label("Driver Dashboard", Theme.ui(Font.BOLD, 17f), Theme.TEXT));
    titles.add(Box.createVerticalStrut(1));
    titles.add(Ui.label("Team " + team + "  ·  2026 REBUILT", Theme.ui(Font.PLAIN, 12f),
        Theme.TEXT_DIM));
    brand.add(titles);

    // Status.
    JPanel pills = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 0));
    pills.setOpaque(false);
    pills.add(connection);
    pills.add(mode);
    pills.add(alliance);
    pills.add(fms);
    fms.setVisible(false);
    JPanel pillsCentred = new JPanel(new GridBagLayout());
    pillsCentred.setOpaque(false);
    pillsCentred.add(pills);

    JPanel left = new JPanel(new BorderLayout(18, 0));
    left.setOpaque(false);
    left.add(brand, BorderLayout.WEST);
    left.add(pillsCentred, BorderLayout.CENTER);

    // Readouts.
    JPanel readouts = new JPanel(new ReadoutLayout());
    readouts.setOpaque(false);
    readouts.add(readout("Autonomous", autoName, 230));
    readouts.add(divider());
    readouts.add(readout("Match", clock, 96));
    readouts.add(divider());
    readouts.add(readout("Battery", battery, 150));

    add(left, BorderLayout.WEST);
    add(readouts, BorderLayout.CENTER);
  }

  private static JComponent readout(String caption, JComponent value, int width) {
    JPanel block = new JPanel(new BorderLayout(0, 2));
    block.setOpaque(false);
    block.setBorder(BorderFactory.createEmptyBorder(0, 16, 0, 16));
    block.add(Ui.caption(caption), BorderLayout.NORTH);
    block.add(value, BorderLayout.CENTER);
    block.setPreferredSize(new Dimension(width + 32, 50));
    return block;
  }

  /**
   * Packs the readouts against the right edge. The first one (the auto name) gives up width
   * first, and its label ellipsizes, so a narrow window never makes the header overlap itself.
   * Below a usable width it is hidden; the Autonomous tab still shows the same thing.
   */
  private static final class ReadoutLayout implements java.awt.LayoutManager {
    @Override
    public void addLayoutComponent(String name, java.awt.Component comp) {
    }

    @Override
    public void removeLayoutComponent(java.awt.Component comp) {
    }

    @Override
    public Dimension preferredLayoutSize(java.awt.Container parent) {
      int width = 0;
      int height = 0;
      for (java.awt.Component child : parent.getComponents()) {
        Dimension d = child.getPreferredSize();
        width += d.width;
        height = Math.max(height, d.height);
      }
      return new Dimension(width, height);
    }

    @Override
    public Dimension minimumLayoutSize(java.awt.Container parent) {
      Dimension preferred = preferredLayoutSize(parent);
      int flexible = parent.getComponentCount() > 0
          ? parent.getComponent(0).getPreferredSize().width : 0;
      return new Dimension(preferred.width - flexible, preferred.height);
    }

    @Override
    public void layoutContainer(java.awt.Container parent) {
      int height = parent.getHeight();
      int x = parent.getWidth();
      java.awt.Component[] children = parent.getComponents();
      for (int i = children.length - 1; i >= 1; i--) {
        int w = children[i].getPreferredSize().width;
        x -= w;
        children[i].setBounds(x, 0, w, height);
      }
      if (children.length > 0) {
        int w = Math.min(children[0].getPreferredSize().width, x);
        boolean fits = w >= 120;
        children[0].setVisible(fits);
        children[0].setBounds(x - w, 0, Math.max(0, w), height);
      }
    }
  }

  private static JComponent divider() {
    JComponent line = new JComponent() {
      @Override
      protected void paintComponent(Graphics g) {
        g.setColor(Theme.BORDER);
        g.fillRect(0, 6, 1, getHeight() - 12);
      }
    };
    line.setPreferredSize(new Dimension(1, 50));
    return line;
  }

  /**
   * @param mode "AUTO", "TELEOP" or "TEST"
   * @param allianceName "Red", "Blue", or anything else for unknown
   * @param matchSeconds seconds left in the period, or negative when there is no match running
   */
  public void update(boolean connected, String address, boolean enabled, String mode,
      String allianceName, boolean fmsAttached, String autoLabel, double matchSeconds,
      double volts, boolean brownout) {

    connection.set(connected ? "CONNECTED" : "NO ROBOT", connected ? Theme.GOOD : Theme.BAD);
    connection.setToolTipText(connected && address != null ? "Robot at " + address : null);

    if (!connected) {
      this.mode.set("DISABLED", Theme.TEXT_FAINT);
      this.mode.solid(false);
    } else if (enabled) {
      this.mode.set(mode + " ENABLED", Theme.GOOD);
      this.mode.solid(true);
    } else {
      this.mode.set("DISABLED", Theme.TEXT_DIM);
      this.mode.solid(false);
    }

    if (connected && "red".equalsIgnoreCase(allianceName)) {
      alliance.set("RED ALLIANCE", Theme.ALLIANCE_RED);
    } else if (connected && "blue".equalsIgnoreCase(allianceName)) {
      alliance.set("BLUE ALLIANCE", Theme.ALLIANCE_BLUE);
    } else {
      alliance.set("NO ALLIANCE", Theme.TEXT_FAINT);
    }
    if (fms.isVisible() != (connected && fmsAttached)) {
      fms.setVisible(connected && fmsAttached);
      revalidate();
    }

    if (!connected) {
      autoName.setText("-");
      autoName.setForeground(Theme.TEXT_FAINT);
    } else if (autoLabel == null || autoLabel.isEmpty()) {
      autoName.setText("Not selected");
      autoName.setForeground(Theme.WARN);
    } else {
      autoName.setText(autoLabel);
      autoName.setForeground(Theme.TEXT);
    }
    autoName.setToolTipText(autoLabel);

    if (!connected || matchSeconds < 0) {
      clock.setText("--:--");
      clock.setForeground(Theme.TEXT_FAINT);
    } else {
      int whole = (int) Math.ceil(matchSeconds);
      clock.setText(String.format(Locale.ROOT, "%d:%02d", whole / 60, whole % 60));
      clock.setForeground(matchSeconds <= 20.0 ? Theme.WARN : Theme.TEXT);
    }

    battery.set(connected ? volts : 0.0, connected && brownout);
  }

  // ---- Parts --------------------------------------------------------------------------------------

  /** Battery outline filled to its charge, with the voltage beside it. */
  private static final class BatteryGauge extends JComponent {

    private double volts;
    private boolean brownout;

    BatteryGauge() {
      setPreferredSize(new Dimension(150, 30));
    }

    void set(double newVolts, boolean newBrownout) {
      if (Math.abs(newVolts - volts) >= 0.01 || newBrownout != brownout) {
        volts = newVolts;
        brownout = newBrownout;
        repaint();
      }
    }

    @Override
    protected void paintComponent(Graphics g) {
      Graphics2D g2 = (Graphics2D) g.create();
      Theme.smooth(g2);
      Color colour;
      if (volts <= 0) {
        colour = Theme.TEXT_FAINT;
      } else if (brownout || volts < 11.0) {
        // Below 11 V under load the roboRIO is heading for a brownout.
        colour = Theme.BAD;
      } else if (volts < 12.0) {
        colour = Theme.WARN;
      } else {
        colour = Theme.GOOD;
      }

      int bw = 34;
      int bh = 18;
      int y = (getHeight() - bh) / 2;
      g2.setColor(Theme.alpha(colour, 200));
      g2.draw(new RoundRectangle2D.Double(0.5, y + 0.5, bw, bh, 5, 5));
      g2.fill(new RoundRectangle2D.Double(bw + 2, y + 5, 3, bh - 9, 2, 2));
      double charge = Math.max(0.0, Math.min(1.0, (volts - 10.5) / (13.0 - 10.5)));
      if (charge > 0) {
        g2.setColor(colour);
        g2.fill(new RoundRectangle2D.Double(3, y + 3, (bw - 5) * charge, bh - 5, 3, 3));
      }

      g2.setFont(Theme.numeric(Font.BOLD, 22f));
      FontMetrics fm = g2.getFontMetrics();
      String text = volts <= 0 ? "--" : String.format(Locale.ROOT, "%.1f", volts);
      int baseline = (getHeight() + fm.getAscent() - fm.getDescent()) / 2;
      g2.setColor(volts <= 0 ? Theme.TEXT_FAINT : colour);
      g2.drawString(text, bw + 14, baseline);
      if (volts > 0) {
        int x = bw + 14 + fm.stringWidth(text) + 4;
        g2.setFont(brownout ? Theme.caption(10f) : Theme.ui(Font.PLAIN, 13f));
        g2.setColor(brownout ? Theme.BAD : Theme.TEXT_DIM);
        g2.drawString(brownout ? "BROWNOUT" : "V", x, baseline);
      }
      g2.dispose();
    }
  }

  /** The team number on an accent tile. Also drawn as the window icon. */
  static final class Badge extends JComponent {

    private final String text;
    private final int size;

    Badge(int team, int size) {
      this.text = String.valueOf(team);
      this.size = size;
      setPreferredSize(new Dimension(size, size));
    }

    @Override
    protected void paintComponent(Graphics g) {
      Graphics2D g2 = (Graphics2D) g.create();
      paintBadge(g2, text, size);
      g2.dispose();
    }

    static void paintBadge(Graphics2D g2, String text, int size) {
      Theme.smooth(g2);
      g2.setPaint(new GradientPaint(0, 0, Theme.ACCENT, size, size, new Color(0x2E7CF6)));
      g2.fill(new RoundRectangle2D.Double(0, 0, size, size, size * 0.3, size * 0.3));
      float fontSize = size * 0.36f;
      g2.setFont(Theme.numeric(Font.BOLD, fontSize));
      FontMetrics fm = g2.getFontMetrics();
      while (fontSize > 6 && fm.stringWidth(text) > size * 0.82) {
        fontSize -= 0.5f;
        g2.setFont(Theme.numeric(Font.BOLD, fontSize));
        fm = g2.getFontMetrics();
      }
      g2.setColor(Theme.TEXT_ON_ACCENT);
      g2.drawString(text, (size - fm.stringWidth(text)) / 2f,
          (size + fm.getAscent() - fm.getDescent()) / 2f);
    }

    /** The badge at the sizes Windows asks for, for the taskbar and title bar. */
    static List<BufferedImage> windowIcons(int team) {
      List<BufferedImage> icons = new ArrayList<>();
      for (int size : new int[] {16, 24, 32, 48, 64, 128}) {
        BufferedImage image = new BufferedImage(size, size, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2 = image.createGraphics();
        paintBadge(g2, String.valueOf(team), size);
        g2.dispose();
        icons.add(image);
      }
      return icons;
    }
  }
}
