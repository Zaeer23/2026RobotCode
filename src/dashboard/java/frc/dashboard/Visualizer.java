package frc.dashboard;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;

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

  private static final int BANDS = 28;
  private static final double FALL_RATE = 0.12;

  private static final Color BACKGROUND = new Color(0x1E, 0x20, 0x24);
  private static final Color BAR_LOW = new Color(0x3D, 0xDC, 0x84);
  private static final Color BAR_HIGH = new Color(0x4C, 0xC2, 0xFF);
  private static final Color IDLE = new Color(0x3A, 0x3E, 0x45);

  private final MusicPlayer player;
  private final double[] levels = new double[BANDS];

  public Visualizer(MusicPlayer player) {
    this.player = player;
    setPreferredSize(new Dimension(240, 48));
    setBackground(BACKGROUND);

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
        levels[i] = Math.max(0.0, levels[i] - FALL_RATE);
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

  @Override
  protected void paintComponent(Graphics g) {
    super.paintComponent(g);
    Graphics2D g2 = (Graphics2D) g.create();
    g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

    int w = getWidth();
    int h = getHeight();
    int gap = 2;
    int barWidth = Math.max(1, (w - gap * (BANDS - 1)) / BANDS);

    for (int i = 0; i < BANDS; i++) {
      int x = i * (barWidth + gap);
      int barHeight = (int) (levels[i] * (h - 4));

      g2.setColor(IDLE);
      g2.fillRect(x, h - 2, barWidth, 2);

      if (barHeight > 0) {
        float t = i / (float) (BANDS - 1);
        g2.setColor(blend(BAR_LOW, BAR_HIGH, t));
        g2.fillRoundRect(x, h - barHeight - 2, barWidth, barHeight, 2, 2);
      }
    }
    g2.dispose();
  }

  private static Color blend(Color a, Color b, float t) {
    return new Color(
        (int) (a.getRed() + (b.getRed() - a.getRed()) * t),
        (int) (a.getGreen() + (b.getGreen() - a.getGreen()) * t),
        (int) (a.getBlue() + (b.getBlue() - a.getBlue()) * t));
  }
}
