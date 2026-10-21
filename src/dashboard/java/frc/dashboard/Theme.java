package frc.dashboard;

import java.awt.Color;
import java.awt.Font;
import java.awt.GraphicsEnvironment;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.RenderingHints;
import java.awt.font.TextAttribute;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import javax.swing.UIManager;

import com.formdev.flatlaf.FlatDarkLaf;
import com.formdev.flatlaf.FlatLaf;

/**
 * The dashboard's look, in one place.
 *
 * <p>Every colour and font comes from here, so the tabs read as one app rather than seven panels
 * that happen to share a window. The palette is dark on purpose: the driver station sits in a dim
 * arena beside a bright field, and a white window would be the brightest thing in the driver's
 * peripheral vision.
 *
 * <p>The status colours mean the same thing everywhere: green is good, amber needs attention, red
 * is bad. Nothing decorative uses them, so a lit tile always means something. The route colours
 * used to draw autonomous paths are a separate set for the same reason.
 */
final class Theme {

  private Theme() {
  }

  // ---- Surfaces ---------------------------------------------------------------

  static final Color BG = new Color(0x0F1217);
  static final Color SURFACE = new Color(0x161A21);
  static final Color SURFACE_RAISED = new Color(0x1D222B);
  static final Color SURFACE_HOVER = new Color(0x232935);
  static final Color BORDER = new Color(0x262C36);
  static final Color BORDER_STRONG = new Color(0x363E4B);

  // ---- Text -------------------------------------------------------------------

  static final Color TEXT = new Color(0xE7EAEF);
  static final Color TEXT_DIM = new Color(0x9AA3B2);
  static final Color TEXT_FAINT = new Color(0x616A7A);

  /** Dark text for use on top of a bright fill. */
  static final Color TEXT_ON_ACCENT = new Color(0x08121C);

  // ---- Status -----------------------------------------------------------------

  static final Color ACCENT = new Color(0x4CC2FF);
  static final Color GOOD = new Color(0x3DDC84);
  static final Color WARN = new Color(0xFFC45C);
  static final Color BAD = new Color(0xFF6B6B);

  static final Color ALLIANCE_BLUE = new Color(0x3B82F6);
  static final Color ALLIANCE_RED = new Color(0xEF4444);

  // ---- Autonomous routes ------------------------------------------------------

  static final Color ROUTE_DRIVE = ACCENT;
  static final Color ROUTE_INTAKE = new Color(0xB48CFF);
  static final Color ROUTE_SHOT = new Color(0xFF9F5A);
  static final Color ROUTE_OTHER = new Color(0x8C96A8);

  // ---- Shape ------------------------------------------------------------------

  /** Corner radius of cards and tiles. */
  static final int RADIUS = 14;

  /** Gap between cards. */
  static final int GAP = 12;

  // ---- Type -------------------------------------------------------------------

  private static String numericFamily;

  /**
   * Installs FlatLaf with the dashboard's palette. Call before any component is created.
   *
   * <p>The variables set here ({@code @background}, {@code @accentColor}) are the ones FlatLaf
   * derives the rest of its theme from, so every stock control (combo boxes, lists, scrollbars,
   * tabs, even the window's title bar) picks up the same colours as the custom-painted panels.
   */
  static void install() {
    FlatLaf.setGlobalExtraDefaults(Map.of(
        "@background", hex(BG),
        "@foreground", hex(TEXT),
        "@accentColor", hex(ACCENT),
        "@selectionBackground", hex(new Color(0x1F4E6B)),
        "@selectionInactiveBackground", hex(SURFACE_HOVER)));
    FlatDarkLaf.setup();

    UIManager.put("Component.arc", 10);
    UIManager.put("Button.arc", 10);
    UIManager.put("TextComponent.arc", 10);
    UIManager.put("CheckBox.arc", 6);
    UIManager.put("ProgressBar.arc", 999);
    UIManager.put("Component.focusWidth", 1);
    UIManager.put("Component.innerFocusWidth", 0);
    UIManager.put("Component.borderColor", BORDER_STRONG);
    UIManager.put("Component.focusedBorderColor", ACCENT);

    UIManager.put("Button.background", SURFACE_RAISED);
    UIManager.put("Button.hoverBackground", SURFACE_HOVER);
    UIManager.put("Button.borderColor", BORDER_STRONG);
    UIManager.put("Button.margin", new Insets(6, 14, 6, 14));
    UIManager.put("ComboBox.background", SURFACE_RAISED);
    UIManager.put("ComboBox.buttonBackground", SURFACE_RAISED);
    UIManager.put("ComboBox.popupBackground", SURFACE_RAISED);
    UIManager.put("TextField.background", SURFACE_RAISED);
    UIManager.put("TextField.placeholderForeground", TEXT_FAINT);

    UIManager.put("List.background", SURFACE);
    UIManager.put("List.selectionArc", 8);
    UIManager.put("List.cellMargins", new Insets(6, 10, 6, 10));

    UIManager.put("ScrollBar.width", 10);
    UIManager.put("ScrollBar.thumbArc", 999);
    UIManager.put("ScrollBar.thumbInsets", new Insets(2, 2, 2, 2));
    UIManager.put("ScrollBar.track", new Color(0, 0, 0, 0));
    UIManager.put("ScrollBar.thumb", BORDER_STRONG);
    UIManager.put("ScrollBar.hoverThumbColor", TEXT_FAINT);
    UIManager.put("ScrollPane.border", null);

    UIManager.put("SplitPane.dividerSize", Theme.GAP);
    UIManager.put("SplitPane.background", BG);
    UIManager.put("SplitPaneDivider.gripColor", BORDER_STRONG);

    UIManager.put("Slider.trackWidth", 4);
    UIManager.put("Slider.thumbSize", new java.awt.Dimension(14, 14));
    UIManager.put("Slider.trackColor", BORDER_STRONG);

    UIManager.put("TabbedPane.tabHeight", 44);
    UIManager.put("TabbedPane.tabInsets", new Insets(0, 18, 0, 18));
    UIManager.put("TabbedPane.tabAreaInsets", new Insets(0, 12, 0, 12));
    UIManager.put("TabbedPane.tabSelectionHeight", 3);
    UIManager.put("TabbedPane.tabSelectionArc", 999);
    UIManager.put("TabbedPane.underlineColor", ACCENT);
    UIManager.put("TabbedPane.inactiveUnderlineColor", ACCENT);
    UIManager.put("TabbedPane.hoverColor", SURFACE_HOVER);
    UIManager.put("TabbedPane.focusColor", SURFACE);
    UIManager.put("TabbedPane.selectedForeground", TEXT);
    UIManager.put("TabbedPane.foreground", TEXT_DIM);
    UIManager.put("TabbedPane.background", SURFACE);
    UIManager.put("TabbedPane.contentAreaColor", BORDER);
    UIManager.put("TabbedPane.showTabSeparators", false);
    UIManager.put("TabbedPane.textIconGap", 8);
    UIManager.put("TabbedPane.font", ui(Font.BOLD, 13f));

    UIManager.put("TitlePane.unifiedBackground", true);
    UIManager.put("TitlePane.background", SURFACE);
    UIManager.put("TitlePane.inactiveBackground", SURFACE);
    UIManager.put("TitlePane.foreground", TEXT_DIM);
    UIManager.put("ToolTip.background", SURFACE_RAISED);
  }

  /** The UI font at a size. Derived from FlatLaf's, which falls back for any missing glyph. */
  static Font ui(int style, float size) {
    Font base = UIManager.getFont("defaultFont");
    if (base == null) {
      base = new Font(Font.DIALOG, Font.PLAIN, 13);
    }
    return base.deriveFont(style, size);
  }

  /** Small capitals-style caption: bold, slightly tracked. Pair with upper-case text. */
  static Font caption(float size) {
    return ui(Font.BOLD, size).deriveFont(Map.of(TextAttribute.TRACKING, 0.06));
  }

  /**
   * Font for live numbers.
   *
   * <p>Bahnschrift is the DIN-style face shipped with Windows 10 and 11; its digits are all the
   * same width, so a changing value does not make the text beside it jitter. Falls back to a
   * monospace face on anything older.
   */
  static Font numeric(int style, float size) {
    if (numericFamily == null) {
      Set<String> installed = new HashSet<>(Arrays.asList(
          GraphicsEnvironment.getLocalGraphicsEnvironment().getAvailableFontFamilyNames()));
      numericFamily = Font.MONOSPACED;
      for (String candidate : new String[] {"Bahnschrift", "Cascadia Mono", "Consolas"}) {
        if (installed.contains(candidate)) {
          numericFamily = candidate;
          break;
        }
      }
    }
    return new Font(numericFamily, style, Math.round(size)).deriveFont(size);
  }

  static Color alpha(Color colour, int alpha) {
    return new Color(colour.getRed(), colour.getGreen(), colour.getBlue(), alpha);
  }

  static Color mix(Color a, Color b, double t) {
    double u = Math.max(0.0, Math.min(1.0, t));
    return new Color(
        (int) Math.round(a.getRed() + (b.getRed() - a.getRed()) * u),
        (int) Math.round(a.getGreen() + (b.getGreen() - a.getGreen()) * u),
        (int) Math.round(a.getBlue() + (b.getBlue() - a.getBlue()) * u),
        (int) Math.round(a.getAlpha() + (b.getAlpha() - a.getAlpha()) * u));
  }

  /** Turns on antialiasing for shapes and text. Every custom painter calls this first. */
  static void smooth(Graphics2D g2) {
    g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,
        RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
    g2.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
    g2.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS,
        RenderingHints.VALUE_FRACTIONALMETRICS_ON);
  }

  private static String hex(Color colour) {
    return String.format("#%02x%02x%02x", colour.getRed(), colour.getGreen(), colour.getBlue());
  }
}
