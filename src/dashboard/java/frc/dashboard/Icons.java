package frc.dashboard;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Component;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Arc2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.RoundRectangle2D;
import java.util.function.Consumer;

import javax.swing.Icon;

/**
 * Small line icons, drawn rather than loaded.
 *
 * <p>Drawn so they stay sharp at any display scaling and need no image files in the jar, and so
 * their colour can follow state. Each is laid out on a 16 by 16 grid and scaled to the size asked
 * for.
 */
final class Icons {

  private Icons() {
  }

  static Icon play(int size, Color colour) {
    return icon(size, colour, g -> {
      Path2D p = new Path2D.Double();
      p.moveTo(4.5, 2.8);
      p.lineTo(13.2, 8.0);
      p.lineTo(4.5, 13.2);
      p.closePath();
      g.fill(p);
    });
  }

  static Icon pause(int size, Color colour) {
    return icon(size, colour, g -> {
      g.fill(new RoundRectangle2D.Double(3.5, 2.5, 3.2, 11, 1.4, 1.4));
      g.fill(new RoundRectangle2D.Double(9.3, 2.5, 3.2, 11, 1.4, 1.4));
    });
  }

  static Icon next(int size, Color colour) {
    return icon(size, colour, g -> {
      Path2D p = new Path2D.Double();
      p.moveTo(3.0, 3.0);
      p.lineTo(10.5, 8.0);
      p.lineTo(3.0, 13.0);
      p.closePath();
      g.fill(p);
      g.fill(new RoundRectangle2D.Double(11.0, 3.0, 2.4, 10, 1.2, 1.2));
    });
  }

  static Icon previous(int size, Color colour) {
    return icon(size, colour, g -> {
      Path2D p = new Path2D.Double();
      p.moveTo(13.0, 3.0);
      p.lineTo(5.5, 8.0);
      p.lineTo(13.0, 13.0);
      p.closePath();
      g.fill(p);
      g.fill(new RoundRectangle2D.Double(2.6, 3.0, 2.4, 10, 1.2, 1.2));
    });
  }

  static Icon stop(int size, Color colour) {
    return icon(size, colour, g -> g.fill(new RoundRectangle2D.Double(3.5, 3.5, 9, 9, 2.4, 2.4)));
  }

  static Icon restart(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.7f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new Arc2D.Double(2.8, 2.8, 10.4, 10.4, 110, 280, Arc2D.OPEN));
      Path2D head = new Path2D.Double();
      head.moveTo(2.2, 2.4);
      head.lineTo(6.6, 3.2);
      head.lineTo(3.6, 6.6);
      head.closePath();
      g.fill(head);
    });
  }

  static Icon plus(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.8f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new Line2D.Double(8, 3, 8, 13));
      g.draw(new Line2D.Double(3, 8, 13, 8));
    });
  }

  static Icon minus(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.8f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new Line2D.Double(3, 8, 13, 8));
    });
  }

  static Icon check(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(2.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      Path2D p = new Path2D.Double();
      p.moveTo(3.2, 8.4);
      p.lineTo(6.6, 11.6);
      p.lineTo(12.8, 4.6);
      g.draw(p);
    });
  }

  static Icon cross(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(2.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new Line2D.Double(4.2, 4.2, 11.8, 11.8));
      g.draw(new Line2D.Double(11.8, 4.2, 4.2, 11.8));
    });
  }

  static Icon swap(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.7f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new Line2D.Double(2.5, 5.5, 12.5, 5.5));
      g.draw(new Line2D.Double(10.0, 3.0, 12.5, 5.5));
      g.draw(new Line2D.Double(10.0, 8.0, 12.5, 5.5));
      g.draw(new Line2D.Double(13.5, 10.5, 3.5, 10.5));
      g.draw(new Line2D.Double(6.0, 8.0, 3.5, 10.5));
      g.draw(new Line2D.Double(6.0, 13.0, 3.5, 10.5));
    });
  }

  static Icon send(int size, Color colour) {
    return icon(size, colour, g -> {
      Path2D p = new Path2D.Double();
      p.moveTo(2.0, 2.6);
      p.lineTo(14.2, 8.0);
      p.lineTo(2.0, 13.4);
      p.lineTo(3.8, 8.0);
      p.closePath();
      g.fill(p);
    });
  }

  /** A stopwatch, for the Match tab. */
  static Icon match(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new Ellipse2D.Double(2.5, 3.5, 11, 11));
      g.draw(new Line2D.Double(8, 9, 8, 6));
      g.draw(new Line2D.Double(8, 9, 10.2, 10.4));
      g.draw(new Line2D.Double(6.5, 1.5, 9.5, 1.5));
    });
  }

  /** A route between two points, for the Autonomous tab. */
  static Icon route(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      Path2D p = new Path2D.Double();
      p.moveTo(3.5, 12.5);
      p.curveTo(3.5, 7.0, 12.5, 9.0, 12.5, 3.5);
      g.draw(p);
      g.fill(new Ellipse2D.Double(1.5, 10.5, 4, 4));
      g.draw(new Ellipse2D.Double(10.5, 1.5, 4, 4));
    });
  }

  /** A gamepad, for the Controls tab. */
  static Icon gamepad(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new RoundRectangle2D.Double(1.5, 4.5, 13, 8, 6, 6));
      g.draw(new Line2D.Double(5, 7, 5, 10));
      g.draw(new Line2D.Double(3.5, 8.5, 6.5, 8.5));
      g.fill(new Ellipse2D.Double(10.0, 6.6, 1.8, 1.8));
      g.fill(new Ellipse2D.Double(11.4, 8.6, 1.8, 1.8));
    });
  }

  /** A music note, for the Music tab. */
  static Icon music(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new Line2D.Double(6.5, 12, 6.5, 3));
      g.draw(new Line2D.Double(6.5, 3, 13, 2));
      g.draw(new Line2D.Double(13, 2, 13, 10.5));
      g.fill(new Ellipse2D.Double(2.5, 10, 4.6, 3.8));
      g.fill(new Ellipse2D.Double(9.0, 8.6, 4.6, 3.8));
    });
  }

  static Icon shuffle(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      Path2D a = new Path2D.Double();
      a.moveTo(2, 4.5);
      a.curveTo(7, 4.5, 8, 11.5, 13, 11.5);
      g.draw(a);
      Path2D b = new Path2D.Double();
      b.moveTo(2, 11.5);
      b.curveTo(7, 11.5, 8, 4.5, 13, 4.5);
      g.draw(b);
      g.draw(new Line2D.Double(11, 2.5, 13.2, 4.5));
      g.draw(new Line2D.Double(11, 6.5, 13.2, 4.5));
      g.draw(new Line2D.Double(11, 9.5, 13.2, 11.5));
      g.draw(new Line2D.Double(11, 13.5, 13.2, 11.5));
    });
  }

  static Icon volume(int size, Color colour) {
    return icon(size, colour, g -> {
      Path2D p = new Path2D.Double();
      p.moveTo(2, 6);
      p.lineTo(5, 6);
      p.lineTo(8.5, 2.8);
      p.lineTo(8.5, 13.2);
      p.lineTo(5, 10);
      p.lineTo(2, 10);
      p.closePath();
      g.fill(p);
      g.setStroke(new BasicStroke(1.4f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new Arc2D.Double(7.0, 5.0, 5.0, 6.0, -55, 110, Arc2D.OPEN));
      g.draw(new Arc2D.Double(7.0, 2.8, 8.0, 10.4, -55, 110, Arc2D.OPEN));
    });
  }

  static Icon search(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.7f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new Ellipse2D.Double(2.5, 2.5, 8.2, 8.2));
      g.draw(new Line2D.Double(9.6, 9.6, 13.5, 13.5));
    });
  }

  static Icon folder(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      Path2D p = new Path2D.Double();
      p.moveTo(1.8, 4.0);
      p.lineTo(6.0, 4.0);
      p.lineTo(7.5, 5.6);
      p.lineTo(14.2, 5.6);
      p.lineTo(14.2, 12.8);
      p.lineTo(1.8, 12.8);
      p.closePath();
      g.draw(p);
    });
  }

  /** Lines with a note, for a playlist. */
  static Icon playlist(int size, Color colour) {
    return icon(size, colour, g -> {
      g.setStroke(new BasicStroke(1.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.draw(new Line2D.Double(2, 3.5, 11, 3.5));
      g.draw(new Line2D.Double(2, 7.5, 11, 7.5));
      g.draw(new Line2D.Double(2, 11.5, 7, 11.5));
      g.draw(new Line2D.Double(13, 6, 13, 12));
      g.fill(new Ellipse2D.Double(9.4, 10.6, 4.2, 3.4));
    });
  }

  /** Empty space the size of an icon, for keeping list text aligned. */
  static Icon blank(int size) {
    return icon(size, Theme.TEXT, g -> { });
  }

  private static Icon icon(int size, Color colour, Consumer<Graphics2D> draw) {
    return new Icon() {
      @Override
      public void paintIcon(Component c, Graphics g, int x, int y) {
        Graphics2D g2 = (Graphics2D) g.create();
        Theme.smooth(g2);
        g2.translate(x, y);
        g2.scale(size / 16.0, size / 16.0);
        g2.setColor(c != null && !c.isEnabled() ? Theme.TEXT_FAINT : colour);
        draw.accept(g2);
        g2.dispose();
      }

      @Override
      public int getIconWidth() {
        return size;
      }

      @Override
      public int getIconHeight() {
        return size;
      }
    };
  }
}
