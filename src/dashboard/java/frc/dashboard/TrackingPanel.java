package frc.dashboard;

import java.awt.Dimension;
import java.awt.LayoutManager;
import java.awt.Rectangle;

import javax.swing.JPanel;
import javax.swing.Scrollable;

/**
 * A panel that is always exactly as wide as the scroll pane it sits in, and scrolls only
 * vertically. Its contents re-flow to the window instead of running off the right edge.
 */
class TrackingPanel extends JPanel implements Scrollable {

  TrackingPanel() {
    super();
  }

  TrackingPanel(LayoutManager layout) {
    super(layout);
  }

  @Override
  public Dimension getPreferredScrollableViewportSize() {
    return getPreferredSize();
  }

  @Override
  public int getScrollableUnitIncrement(Rectangle visible, int orientation, int direction) {
    return 24;
  }

  @Override
  public int getScrollableBlockIncrement(Rectangle visible, int orientation, int direction) {
    return visible.height;
  }

  @Override
  public boolean getScrollableTracksViewportWidth() {
    return true;
  }

  @Override
  public boolean getScrollableTracksViewportHeight() {
    // Fill the height when there is room, scroll when there is not.
    return getParent() != null && getParent().getHeight() > getPreferredSize().height;
  }
}
