package frc.dashboard;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.RoundRectangle2D;

import javax.swing.JPanel;
import javax.swing.Timer;

/**
 * Small audio visualiser driven by the decoder's PCM output.
 *
 * <p>Bars are RMS energy per band, taken straight from the samples the player is currently
 * writing to the sound card. Not an FFT — bands are contiguous slices of the sample window, which
 * is cheap and looks right at this size. A real spectrum would need a transform per frame for no
 * visible gain on a 40-pixel-tall widget.
 *
 * <p>Bars fall gradually rather than tracking the signal exactly, because at 50 fps an
 * instantaneous meter reads as noise.
 */
public class Visualizer extends JPanel {

  private static final int BANDS = 40;
  private static final double FALL_RATE = 0.12;

  private static final Color BAR_LOW = Theme.ACCENT;
  private static final Color BAR_HIGH = Theme.ROUTE_INTAKE;

  private final MusicPlayer player;
  private final double[] levels = new double[BANDS];

  public Visualizer(MusicPlayer player) {
    this.player = player;
    setPreferredSize(new Dimension(240, 120));
    setOpaque(false);

    Timer timer = new Timer(20, e -> {
      sample();
      repaint();
    });
    timer.start();
  }

  private void sample() {
    float[] samples = player.latestSamples();

    if (samples.length == 0 || !player.isPlaying()) {
      for (int i = 0; i < BANDS; i++) {
        levels[i] = Math.max(resting(i), levels[i] - FALL_RATE);
      }
      return;
    }

    int perBand = Math.max(1, samples.length / BANDS);
    for (int band = 0; band < BANDS; band++) {
      int start = band * perBand;
      int end = Math.min(samples.length, start + perBand);

      double sumSquares = 0.0;
      for (int i = start; i < end; i++) {
        sumSquares += samples[i] * samples[i];
      }
      int count = Math.max(1, end - start);
      double rms = Math.sqrt(sumSquares / count);

      // Scaled up because typical music RMS sits well below 1 and would barely move the bars.
      double target = Math.min(1.0, rms * 3.5);
      levels[band] = target > levels[band]
          ? target
          : Math.max(target, levels[band] - FALL_RATE);
    }
  }

  /**
   * Bars mirrored about the middle, like a waveform, so the panel is balanced whether the music
   * is loud or silent.
   */
  @Override
  protected void paintComponent(Graphics g) {
    Graphics2D g2 = (Graphics2D) g.create();
    Theme.smooth(g2);

    int w = getWidth();
    int h = getHeight();
    double mid = h / 2.0;
    double gap = 4;
    double barWidth = Math.max(2, (w - gap * (BANDS - 1)) / BANDS);
    double arc = Math.min(barWidth, 8);

    for (int i = 0; i < BANDS; i++) {
      double x = i * (barWidth + gap);
      double barHeight = Math.max(4, levels[i] * (h - 8));
      Color colour = blend(BAR_LOW, BAR_HIGH, i / (float) (BANDS - 1));
      g2.setPaint(new java.awt.LinearGradientPaint(
          0f, (float) (mid - barHeight / 2), 0f, (float) (mid + barHeight / 2 + 0.01),
          new float[] {0f, 0.5f, 1f},
          new Color[] {Theme.alpha(colour, 110), colour, Theme.alpha(colour, 110)}));
      g2.fill(new RoundRectangle2D.Double(x, mid - barHeight / 2, barWidth, barHeight, arc, arc));
    }
    g2.dispose();
  }

  /** A low, still skyline shown while nothing plays, so the panel never looks broken. */
  private static double resting(int band) {
    return 0.02 + 0.015 * Math.abs(Math.sin(band * 0.55) * Math.sin(band * 0.21));
  }

  private static Color blend(Color a, Color b, float t) {
    return new Color(
        (int) (a.getRed() + (b.getRed() - a.getRed()) * t),
        (int) (a.getGreen() + (b.getGreen() - a.getGreen()) * t),
        (int) (a.getBlue() + (b.getBlue() - a.getBlue()) * t));
  }
}
