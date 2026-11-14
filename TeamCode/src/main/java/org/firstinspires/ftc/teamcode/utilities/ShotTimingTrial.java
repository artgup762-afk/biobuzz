package org.firstinspires.ftc.teamcode.utilities;

import java.util.ArrayList;
import java.util.List;

/** One timing measurement, including spin-up and the last ball's flywheel recovery. */
public final class ShotTimingTrial {
  public enum State {
    RUNNING,
    COMPLETE,
    CANCELLED
  }

  private final FlywheelDipDetector detector;
  private final int expectedBalls;
  private final long startMs;
  private final List<Integer> ballTimes = new ArrayList<>();
  private State state = State.RUNNING;
  private boolean armed;
  private int elapsedMs;

  public ShotTimingTrial(FlywheelDipDetector detector, int expectedBalls, long startMs) {
    if (expectedBalls <= 0) throw new IllegalArgumentException("Positive count required");
    this.detector = detector;
    this.expectedBalls = expectedBalls;
    this.startMs = startMs;
    detector.reset();
  }

  /** Returns true only for a new counted ball. Caller supplies a monotonic clock. */
  public boolean update(double velocity, boolean flywheelReady, long nowMs) {
    if (state != State.RUNNING) return false;
    elapsedMs = (int) Math.clamp(nowMs - startMs, 0, Integer.MAX_VALUE);
    if (!armed && flywheelReady) armed = true;
    // Once all balls are counted, keep updating recovery but never record a fifth event.
    boolean dip = armed && detector.update(velocity, nowMs);
    boolean ball = dip && ballTimes.size() < expectedBalls;
    if (ball) ballTimes.add(elapsedMs);
    if (ballTimes.size() >= expectedBalls && !detector.isInDip() && flywheelReady) {
      state = State.COMPLETE;
    }
    return ball;
  }

  public void cancel() {
    if (state == State.RUNNING) state = State.CANCELLED;
  }

  public State state() {
    return state;
  }

  public boolean armed() {
    return armed;
  }

  public int elapsedMs() {
    return elapsedMs;
  }

  public List<Integer> ballTimesMs() {
    return List.copyOf(ballTimes);
  }
}
