package frc.dashboard;

import java.awt.Color;
import java.awt.Font;

import javax.swing.Icon;
import javax.swing.JButton;
import javax.swing.JLabel;

import com.formdev.flatlaf.FlatClientProperties;

/** Small factories for the labels and buttons every tab uses, so they look the same everywhere. */
final class Ui {

  private Ui() {
  }

  static JLabel label(String text, Font font, Color colour) {
    JLabel label = new JLabel(text);
    label.setFont(font);
    label.setForeground(colour);
    return label;
  }

  /** Upper-case dim caption, for naming a field or group. */
  static JLabel caption(String text) {
    return label(text.toUpperCase(), Theme.caption(11f), Theme.TEXT_DIM);
  }

  /** Dim wrapping body text. */
  static JLabel note(String text, int widthPx) {
    JLabel label = label("<html><body style='width:" + widthPx + "px'>" + text + "</body></html>",
        Theme.ui(Font.PLAIN, 12f), Theme.TEXT_DIM);
    return label;
  }

  /** The one button on a screen that does the main thing. */
  static JButton primary(String text, Icon icon) {
    JButton button = new JButton(text, icon);
    stylePrimary(button);
    return button;
  }

  static void stylePrimary(JButton button) {
    button.putClientProperty(FlatClientProperties.STYLE,
        "arc: 12; borderWidth: 0; focusWidth: 0; innerFocusWidth: 0;"
            + "background: #4cc2ff; foreground: #08121c;"
            + "hoverBackground: #74d1ff; pressedBackground: #39a9e3;"
            + "disabledText: #616a7a; font: bold +1; margin: 10,16,10,16");
  }

  /** A quieter button for secondary actions. */
  static JButton secondary(String text, Icon icon) {
    JButton button = new JButton(text, icon);
    button.putClientProperty(FlatClientProperties.STYLE,
        "arc: 10; focusWidth: 0; innerFocusWidth: 0; font: bold; margin: 6,12,6,12");
    return button;
  }

  /** A square icon-only button, for transport controls. */
  static JButton icon(Icon icon, String tooltip) {
    JButton button = new JButton(icon);
    button.setToolTipText(tooltip);
    button.putClientProperty(FlatClientProperties.STYLE,
        "arc: 999; focusWidth: 0; innerFocusWidth: 0; margin: 8,8,8,8");
    return button;
  }
}
