package frc.robot.util;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * TCP forwarding with a fixed buffer per direction and a limit on simultaneous connections.
 *
 * <p>Blocking writes apply TCP backpressure: a slow viewer stops reads from the camera instead
 * of accumulating video in an asynchronous native write queue. All I/O runs on daemon threads,
 * never on the robot loop. Intended for the Limelight's continuous MJPEG stream.
 */
public final class BoundedTcpForwarder implements AutoCloseable {
  private static final int BUFFER_BYTES = 16 * 1024;
  private static final int CONNECT_TIMEOUT_MILLIS = 1500;
  private static final long STALLED_VIDEO_NANOS = TimeUnit.SECONDS.toNanos(5);

  /** Shared by every route to the camera so changing ports cannot multiply video traffic. */
  public static final class VideoBudget {
    private final int maxConnections;
    private final Semaphore slots;
    private final long bytesPerSecond;
    private long nextWriteNanos;

    public VideoBudget(int maxConnections, long bytesPerSecond) {
      if (maxConnections < 1 || bytesPerSecond < 0) {
        throw new IllegalArgumentException("Positive connections and nonnegative bandwidth required");
      }
      this.maxConnections = maxConnections;
      this.slots = new Semaphore(maxConnections);
      this.bytesPerSecond = bytesPerSecond;
    }

    private synchronized long reserveDelayNanos(int bytes) {
      if (bytesPerSecond == 0) {
        return 0;
      }
      long now = System.nanoTime();
      long writeAt = Math.max(now, nextWriteNanos);
      nextWriteNanos = writeAt + TimeUnit.SECONDS.toNanos(1) * bytes / bytesPerSecond;
      return writeAt - now;
    }
  }

  private final ServerSocket server;
  private final String remoteHost;
  private final int remotePort;
  private final VideoBudget budget;
  private final Set<Connection> connections = ConcurrentHashMap.newKeySet();
  private final AtomicBoolean closed = new AtomicBoolean();

  public BoundedTcpForwarder(int localPort, String remoteHost, int remotePort,
      int maxConnections) throws IOException {
    this(localPort, remoteHost, remotePort, new VideoBudget(maxConnections, 0));
  }

  public BoundedTcpForwarder(int localPort, String remoteHost, int remotePort,
      VideoBudget budget) throws IOException {
    this.remoteHost = remoteHost;
    this.remotePort = remotePort;
    this.budget = java.util.Objects.requireNonNull(budget);
    server = new ServerSocket();
    try {
      server.setReuseAddress(true);
      server.setSoTimeout(1000);
      server.bind(new InetSocketAddress(localPort), budget.maxConnections);
    } catch (IOException e) {
      server.close();
      throw e;
    }
    startDaemon("accept", this::acceptConnections);
  }

  public int getLocalPort() {
    return server.getLocalPort();
  }

  private void acceptConnections() {
    while (!closed.get()) {
      try {
        // A radio loss may leave TCP half-open. Reclaim its slot even if a write is blocked.
        long now = System.nanoTime();
        for (Connection connection : connections) {
          if (now - connection.lastResponseNanos > STALLED_VIDEO_NANOS) {
            connection.close();
          }
        }
        Socket client = server.accept();
        // Only this thread adds connections. Completed connections remove themselves.
        if (closed.get() || !budget.slots.tryAcquire()) {
          client.close();
          continue;
        }
        Connection connection = new Connection(client);
        connections.add(connection);
        // close() may have run between accept() and adding this connection.
        if (closed.get()) {
          connection.close();
        } else {
          startDaemon("connect", connection::run);
        }
      } catch (SocketTimeoutException e) {
        // Wake periodically to expire stalled viewers, even when nobody new connects.
      } catch (IOException e) {
        if (!closed.get()) {
          System.err.println("Camera forwarder on port " + getLocalPort() + ": " + e.getMessage());
          close();
        }
      }
    }
  }

  private void startDaemon(String direction, Runnable action) {
    Thread thread = new Thread(action, "camera-forward-" + getLocalPort() + "-" + direction);
    thread.setDaemon(true);
    thread.start();
  }

  private final class Connection implements AutoCloseable {
    private final Socket client;
    private final Socket remote = new Socket();
    private final AtomicBoolean finished = new AtomicBoolean();
    private volatile long lastResponseNanos = System.nanoTime();

    Connection(Socket client) {
      this.client = client;
    }

    void run() {
      try {
        configure(client);
        configure(remote);
        remote.connect(new InetSocketAddress(remoteHost, remotePort), CONNECT_TIMEOUT_MILLIS);
        startDaemon("response", () -> copy(remote, client, true));
        copy(client, remote, false);
      } catch (IOException e) {
        // A disconnected camera/viewer is normal. Closing both sides releases blocked I/O.
      } finally {
        close();
      }
    }

    private void copy(Socket source, Socket destination, boolean videoResponse) {
      byte[] buffer = new byte[BUFFER_BYTES];
      try {
        InputStream input = source.getInputStream();
        OutputStream output = destination.getOutputStream();
        int count;
        while (!finished.get() && (count = input.read(buffer)) != -1) {
          if (videoResponse) {
            // Only camera workers wait here. Neither the robot loop nor camera requests wait.
            TimeUnit.NANOSECONDS.sleep(budget.reserveDelayNanos(count));
          }
          if (finished.get()) {
            break;
          }
          output.write(buffer, 0, count);
          if (videoResponse) {
            lastResponseNanos = System.nanoTime();
          }
        }
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      } catch (IOException e) {
        // Includes the other direction closing this connection.
      } finally {
        close();
      }
    }

    @Override
    public void close() {
      if (finished.compareAndSet(false, true)) {
        closeSocket(client);
        closeSocket(remote);
        connections.remove(this);
        budget.slots.release();
      }
    }
  }

  private static void configure(Socket socket) throws IOException {
    socket.setReceiveBufferSize(BUFFER_BYTES);
    socket.setSendBufferSize(BUFFER_BYTES);
    socket.setTcpNoDelay(true);
  }

  private static void closeSocket(Socket socket) {
    try {
      socket.close();
    } catch (IOException ignored) {
      // Continue closing the other sockets.
    }
  }

  @Override
  public void close() {
    closed.set(true);
    try {
      server.close();
    } catch (IOException ignored) {
      // Still close every active connection.
    }
    for (Connection connection : connections) {
      connection.close();
    }
  }
}
