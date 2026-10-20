package frc.robot.util;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
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

  private final ServerSocket server;
  private final String remoteHost;
  private final int remotePort;
  private final int maxConnections;
  private final Set<Connection> connections = ConcurrentHashMap.newKeySet();
  private final AtomicBoolean closed = new AtomicBoolean();

  public BoundedTcpForwarder(int localPort, String remoteHost, int remotePort,
      int maxConnections) throws IOException {
    if (maxConnections < 1) {
      throw new IllegalArgumentException("maxConnections must be positive");
    }
    this.remoteHost = remoteHost;
    this.remotePort = remotePort;
    this.maxConnections = maxConnections;
    server = new ServerSocket();
    try {
      server.setReuseAddress(true);
      server.bind(new InetSocketAddress(localPort), maxConnections);
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
        Socket client = server.accept();
        // Only this thread adds connections. Completed connections remove themselves.
        if (closed.get() || connections.size() >= maxConnections) {
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

    Connection(Socket client) {
      this.client = client;
    }

    void run() {
      try {
        configure(client);
        configure(remote);
        remote.connect(new InetSocketAddress(remoteHost, remotePort), CONNECT_TIMEOUT_MILLIS);
        startDaemon("response", () -> copy(remote, client));
        copy(client, remote);
      } catch (IOException e) {
        // A disconnected camera/viewer is normal. Closing both sides releases blocked I/O.
      } finally {
        close();
      }
    }

    private void copy(Socket source, Socket destination) {
      byte[] buffer = new byte[BUFFER_BYTES];
      try {
        InputStream input = source.getInputStream();
        OutputStream output = destination.getOutputStream();
        int count;
        while (!finished.get() && (count = input.read(buffer)) != -1) {
          output.write(buffer, 0, count);
        }
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
