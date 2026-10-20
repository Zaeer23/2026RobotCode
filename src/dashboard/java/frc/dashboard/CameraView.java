package frc.dashboard;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.BufferedInputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URI;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.imageio.ImageIO;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

/**
 * Live Limelight view.
 *
 * <p>Pulls the camera's MJPEG stream and draws the frames. MJPEG is just JPEG images concatenated
 * with boundary markers, so this scans for the JPEG start/end markers and decodes whatever lands
 * between them — no media framework needed, which matters because the WPILib JDK ships neither
 * JavaFX nor any video support.
 *
 * <p>Reconnects on its own. The camera's address changes with how the robot is cabled (mDNS,
 * static IP, or USB), so the stream URL is supplied by the robot and can change at runtime; a
 * dropped connection is normal and not worth surfacing as an error.
 */
public class CameraView extends JPanel {

  private static final Color BACKGROUND = new Color(0x1E, 0x20, 0x24);
  private static final Color TEXT = new Color(0x9A, 0xA0, 0xA8);

  private static final byte[] JPEG_START = {(byte) 0xFF, (byte) 0xD8};
  private static final byte[] JPEG_END = {(byte) 0xFF, (byte) 0xD9};

  private volatile java.util.List<String> candidates = java.util.List.of();
  private volatile String url = "";
  private volatile BufferedImage frame;
  private volatile String status = "no stream";
  private final AtomicBoolean running = new AtomicBoolean(false);

  /**
   * Sets the URLs to try, in order, until one produces frames.
   *
   * <p>Probing rather than trusting one address, because the camera's own IP is frequently
   * unreachable from here. When the Limelight is cabled to the roboRIO over USB it lives on
   * 172.29.0.1 — a private link between those two devices that the driver station laptop has no
   * route to. The way in is the roboRIO's port forwards, so the robot's address on ports
   * 5800/5820/5830 is tried alongside whatever the robot reports.
   */
  public void setCandidates(java.util.List<String> urls) {
    if (urls == null || urls.isEmpty() || urls.equals(candidates)) {
      return;
    }
    candidates = java.util.List.copyOf(urls);
    start();
  }

  public CameraView() {
    setPreferredSize(new Dimension(320, 240));
    setBackground(BACKGROUND);
  }

  /** Points the view at a stream. Safe to call repeatedly with the same URL. */
  public void setStreamUrl(String newUrl) {
    if (newUrl == null || newUrl.isBlank() || newUrl.equals(url)) {
      return;
    }
    url = newUrl;
    start();
  }

  public void start() {
    if (running.getAndSet(true)) {
      return;
    }
    Thread reader = new Thread(this::readLoop, "camera-view");
    reader.setDaemon(true);
    reader.start();
  }

  public void stop() {
    running.set(false);
  }

  private void readLoop() {
    while (running.get()) {
      java.util.List<String> targets = candidates;
      if (targets.isEmpty()) {
        sleep(1000);
        continue;
      }

      boolean connectedToAny = false;
      for (String target : targets) {
        if (!running.get()) {
          return;
        }
        try {
          status = "trying " + hostOf(target);
          repaintSoon();

          HttpURLConnection connection =
              (HttpURLConnection) URI.create(target).toURL().openConnection();
          connection.setConnectTimeout(1200);
          connection.setReadTimeout(4000);

          try (InputStream in = new BufferedInputStream(connection.getInputStream(), 64 * 1024)) {
            url = target;
            status = "live";
            connectedToAny = true;
            // Blocks until the stream drops, then falls out to retry the whole list.
            readFrames(in, target);
          } finally {
            connection.disconnect();
          }
        } catch (Exception e) {
          // This route is not reachable; try the next.
          continue;
        }
        if (connectedToAny) {
          break;
        }
      }

      if (!connectedToAny) {
        status = "offline - no route to camera";
        frame = null;
        repaintSoon();
        sleep(2000);
      }
    }
  }

  private static String hostOf(String url) {
    try {
      URI uri = URI.create(url);
      return uri.getHost() + ":" + uri.getPort();
    } catch (Exception e) {
      return url;
    }
  }

  /** Scans the stream for JPEG boundaries and decodes each complete image. */
  private void readFrames(InputStream in, String openedUrl) throws Exception {
    java.io.ByteArrayOutputStream buffer = new java.io.ByteArrayOutputStream(64 * 1024);
    int previous = -1;
    boolean inImage = false;

    while (running.get() && openedUrl.equals(url)) {
      int current = in.read();
      if (current < 0) {
        return;
      }

      if (!inImage) {
        if (previous == (JPEG_START[0] & 0xFF) && current == (JPEG_START[1] & 0xFF)) {
          inImage = true;
          buffer.reset();
          buffer.write(JPEG_START[0]);
          buffer.write(JPEG_START[1]);
        }
      } else {
        buffer.write(current);
        if (previous == (JPEG_END[0] & 0xFF) && current == (JPEG_END[1] & 0xFF)) {
          inImage = false;
          BufferedImage decoded =
              ImageIO.read(new ByteArrayInputStream(buffer.toByteArray()));
          if (decoded != null) {
            frame = decoded;
            status = "live";
            repaintSoon();
          }
        }
        // A frame this large means the boundary was missed; resync rather than grow forever.
        if (buffer.size() > 4 * 1024 * 1024) {
          inImage = false;
          buffer.reset();
        }
      }
      previous = current;
    }
  }

  private void repaintSoon() {
    SwingUtilities.invokeLater(this::repaint);
  }

  private static void sleep(long millis) {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  @Override
  protected void paintComponent(Graphics g) {
    super.paintComponent(g);
    Graphics2D g2 = (Graphics2D) g.create();
    g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
        RenderingHints.VALUE_INTERPOLATION_BILINEAR);

    BufferedImage current = frame;
    if (current == null) {
      g2.setColor(TEXT);
      String message = "Limelight: " + status;
      g2.drawString(message, 10, getHeight() / 2);
      g2.dispose();
      return;
    }

    // Letterbox so the image is never stretched out of proportion.
    double scale = Math.min(
        getWidth() / (double) current.getWidth(),
        getHeight() / (double) current.getHeight());
    int w = (int) (current.getWidth() * scale);
    int h = (int) (current.getHeight() * scale);
    g2.drawImage(current, (getWidth() - w) / 2, (getHeight() - h) / 2, w, h, null);
    g2.dispose();
  }
}
