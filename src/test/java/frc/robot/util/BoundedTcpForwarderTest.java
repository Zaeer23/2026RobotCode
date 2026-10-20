package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

@Timeout(10)
class BoundedTcpForwarderTest {
  @Test
  void forwardsRequestsAndResponsesWithoutChangingBytes() throws Exception {
    byte[] payload = new byte[256 * 1024];
    for (int i = 0; i < payload.length; i++) {
      payload[i] = (byte) i;
    }
    try (ServerSocket camera = camera();
        BoundedTcpForwarder forwarder = new BoundedTcpForwarder(
            0, "127.0.0.1", camera.getLocalPort(), 2);
        Socket viewer = viewer(forwarder);
        Socket upstream = camera.accept()) {
      upstream.setSoTimeout(3000);
      CompletableFuture<byte[]> request = new CompletableFuture<>();
      daemon(() -> {
        try {
          byte[] received = upstream.getInputStream().readNBytes(payload.length);
          upstream.getOutputStream().write(received);
          request.complete(received);
        } catch (IOException e) {
          request.completeExceptionally(e);
        }
      });
      viewer.getOutputStream().write(payload);
      assertArrayEquals(payload, viewer.getInputStream().readNBytes(payload.length));
      assertArrayEquals(payload, request.get(3, TimeUnit.SECONDS));
    }
  }

  @Test
  void slowViewerBackpressuresCameraInsteadOfQueueingVideo() throws Exception {
    try (ServerSocket camera = camera();
        BoundedTcpForwarder forwarder = new BoundedTcpForwarder(
            0, "127.0.0.1", camera.getLocalPort(), 2);
        Socket viewer = viewer(forwarder);
        Socket upstream = camera.accept()) {
      upstream.setSendBufferSize(16 * 1024);
      AtomicLong sent = new AtomicLong();
      CompletableFuture<Void> producer = new CompletableFuture<>();
      byte[] chunk = new byte[16 * 1024];
      daemon(() -> {
        try {
          // An asynchronous, unbounded forwarder eagerly consumes all 32 MiB even when
          // the viewer never reads. The bounded forwarder must stop consuming upstream.
          for (int i = 0; i < 2048; i++) {
            upstream.getOutputStream().write(chunk);
            sent.addAndGet(chunk.length);
          }
          producer.complete(null);
        } catch (IOException e) {
          producer.complete(null); // Expected when the test closes the blocked connection.
        }
      });
      Thread.sleep(500);
      assertTrue(sent.get() > 0, "The camera must have started sending");
      assertTrue(sent.get() < 4 * 1024 * 1024,
          "A stalled viewer must stop upstream reads; sent " + sent.get() + " bytes");
      assertTrue(!producer.isDone(), "The producer should be blocked by backpressure");

      // Closing the forwarder must interrupt even a blocked write and close both sides.
      forwarder.close();
      producer.get(3, TimeUnit.SECONDS);
      // Closing a socket with unread video may send a TCP reset rather than a graceful EOF.
    }
  }

  @Test
  void limitsConnectionsAndReleasesSlotAfterDisconnect() throws Exception {
    try (ServerSocket camera = camera();
        BoundedTcpForwarder forwarder = new BoundedTcpForwarder(
            0, "127.0.0.1", camera.getLocalPort(), 1);
        Socket firstViewer = viewer(forwarder);
        Socket firstUpstream = camera.accept()) {
      camera.setSoTimeout(3000);
      firstUpstream.setSoTimeout(3000);
      try (Socket excessViewer = viewer(forwarder)) {
        assertEquals(-1, excessViewer.getInputStream().read(), "Reject excess connections");
      }
      firstViewer.close();
      assertEquals(-1, firstUpstream.getInputStream().read());

      // Allow the worker that closed the upstream socket to release its connection slot.
      long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(3);
      boolean reconnected = false;
      while (!reconnected && System.nanoTime() < deadline) {
        try (Socket nextViewer = viewer(forwarder)) {
          nextViewer.getOutputStream().write(42);
          camera.setSoTimeout(100);
          try (Socket nextUpstream = camera.accept()) {
            nextUpstream.setSoTimeout(3000);
            assertEquals(42, nextUpstream.getInputStream().read());
            reconnected = true;
          } catch (java.net.SocketTimeoutException e) {
            // A rejected connection can race the previous worker's final cleanup.
          }
        }
      }
      assertTrue(reconnected, "A disconnected viewer must free its connection slot");
    }
  }

  private static Socket viewer(BoundedTcpForwarder forwarder) throws IOException {
    Socket socket = new Socket();
    socket.setReceiveBufferSize(16 * 1024);
    socket.connect(new java.net.InetSocketAddress("127.0.0.1", forwarder.getLocalPort()), 3000);
    socket.setSoTimeout(3000);
    return socket;
  }

  private static ServerSocket camera() throws IOException {
    ServerSocket socket = new ServerSocket(0);
    socket.setSoTimeout(3000);
    return socket;
  }

  private static void daemon(Runnable action) {
    Thread thread = new Thread(action, "test-camera");
    thread.setDaemon(true);
    thread.start();
  }
}
