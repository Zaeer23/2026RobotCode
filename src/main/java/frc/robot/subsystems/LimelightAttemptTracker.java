package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicInteger;

public final class LimelightAttemptTracker {
  private static final AtomicInteger ATTEMPT_COUNTER = new AtomicInteger(0);

  private LimelightAttemptTracker() {
  }

  public static int nextAttemptId() {
    return ATTEMPT_COUNTER.incrementAndGet();
  }
}
