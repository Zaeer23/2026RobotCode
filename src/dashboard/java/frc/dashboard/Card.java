package frc.dashboard;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.RoundRectangle2D;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;

/**
 * A rounded panel with an optional title row. Every tab is laid out from these.
 *
 * <p>The title row holds a title, an optional one-line subtitle under it, and a slot on the right
 * for small controls or status pills that belong to this card.
 */
public class Card extends JPanel {

  private final JPanel body = new JPanel(new BorderLayout());
  private final JPanel headerRight = new JPanel(new FlowLayout(FlowLayout.RIGHT, 8, 0));
  private JLabel titleLabel;
  private JLabel subtitleLabel;

  public Card() {
    this(null, null);
  }

  public Card(String title) {
    this(title, null);
  }

  public Card(String title, String subtitle) {
    super(new BorderLayout(0, 12));
    setOpaque(false);
    setPadding(14, 16, 16, 16);

    body.setOpaque(false);
    add(body, BorderLayout.CENTER);

    if (title != null) {
      JPanel header = new JPanel(new BorderLayout(12, 0));
      header.setOpaque(false);

      JPanel titles = new JPanel();
      titles.setLayout(new BoxLayout(titles, BoxLayout.Y_AXIS));
      titles.setOpaque(false);

      titleLabel = new JLabel(title);
      titleLabel.setFont(Theme.ui(java.awt.Font.BOLD, 14f));
      titleLabel.setForeground(Theme.TEXT);
      titles.add(titleLabel);

      subtitleLabel = new JLabel(subtitle == null ? "" : subtitle);
      subtitleLabel.setFont(Theme.ui(java.awt.Font.PLAIN, 12f));
      subtitleLabel.setForeground(Theme.TEXT_DIM);
      subtitleLabel.setVisible(subtitle != null);
      titles.add(Box.createVerticalStrut(2));
      titles.add(subtitleLabel);

      headerRight.setOpaque(false);
      header.add(titles, BorderLayout.CENTER);
      header.add(headerRight, BorderLayout.EAST);
      add(header, BorderLayout.NORTH);
    }
  }

  /** Puts a component in the card's body, replacing whatever was there. */
  public Card content(Component component) {
    body.removeAll();
    body.add(component, BorderLayout.CENTER);
    return this;
  }

  public JPanel body() {
    return body;
  }

  /** Adds a small control or pill to the right end of the title row. */
  public Card addHeaderRight(Component component) {
    headerRight.add(component);
    return this;
  }

  public void setTitle(String title) {
    if (titleLabel != null) {
      titleLabel.setText(title);
    }
  }

  public void setSubtitle(String subtitle) {
    if (subtitleLabel != null) {
      subtitleLabel.setText(subtitle == null ? "" : subtitle);
      subtitleLabel.setVisible(subtitle != null && !subtitle.isEmpty());
    }
  }

  public final void setPadding(int top, int left, int bottom, int right) {
    setBorder(BorderFactory.createEmptyBorder(top, left, bottom, right));
  }

  @Override
  protected void paintComponent(Graphics g) {
    Graphics2D g2 = (Graphics2D) g.create();
    Theme.smooth(g2);
    RoundRectangle2D shape = new RoundRectangle2D.Double(
        0.5, 0.5, getWidth() - 1.0, getHeight() - 1.0, Theme.RADIUS, Theme.RADIUS);
    g2.setColor(Theme.SURFACE);
    g2.fill(shape);
    g2.setColor(Theme.BORDER);
    g2.draw(shape);
    g2.dispose();
  }
}
