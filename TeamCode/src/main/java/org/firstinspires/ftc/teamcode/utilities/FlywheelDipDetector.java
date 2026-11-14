package org.firstinspires.ftc.teamcode.utilities;

public final class FlywheelDipDetector {

  private final double dipFraction;
  private final double reboundFraction;
  private final double baselineAlpha;
  private final long refractoryMs;

  private double baselineVelocity = Double.NaN;
  private double troughVelocity = Double.NaN;
  private boolean inDip = false;
  private long lastEventMs = Long.MIN_VALUE;
  private int eventCount = 0;
  private double lastDipFraction = 0.0;
  private double deepestDipFraction = 0.0;

  /**
   * @param dipFraction how far below the reference, as a fraction, the wheel must fall to count a
   *     ball
   * @param reboundFraction how far back up off the dip's own trough the wheel must climb before the
   *     next ball can register. This is what ends a dip; it is not measured against the reference.
   * @param baselineAlpha EMA weight per update for the reference speed while not in a dip
   * @param refractoryMs minimum spacing between counted events
   */
  public FlywheelDipDetector(
      double dipFraction, double reboundFraction, double baselineAlpha, long refractoryMs) {
    this.dipFraction = dipFraction;
    this.reboundFraction = reboundFraction;
    this.baselineAlpha = baselineAlpha;
    this.refractoryMs = refractoryMs;
  }

  /**
   * Feeds one velocity sample. Call once per loop while the flywheel is at speed.
   *
   * @param velocity measured flywheel velocity; sign is ignored, as everywhere else that reads it
   * @return true on the leading edge of a new dip, i.e. exactly once per ball
   */
  public boolean update(double velocity, long nowMs) {
    double speed = Math.abs(velocity);
    if (!Double.isFinite(speed)) {
      return false;
    }
    if (Double.isNaN(baselineVelocity) || baselineVelocity <= 0.0) {
      baselineVelocity = speed;
      return false;
    }

    lastDipFraction = (baselineVelocity - speed) / baselineVelocity;
    deepestDipFraction = Math.max(deepestDipFraction, lastDipFraction);

    boolean risingEdge = false;
    if (inDip) {
      troughVelocity = Math.min(troughVelocity, speed);
      // A dip that reached zero must also be able to recover; zero itself is not a rebound.
      if (speed > troughVelocity && speed >= troughVelocity * (1.0 + reboundFraction)) {
        inDip = false;
        // Re-seed where the wheel came back to, not where it was before the ball.
        baselineVelocity = speed;
      }
    } else if (lastDipFraction >= dipFraction) {
      // Latched on entry regardless of the refractory window below: without this, an edge
      // suppressed as too-soon would fire again the moment the window expired, while the wheel was
      // still down inside the very same dip. The reference is deliberately left alone here —
      // averaging in the sample that opened the dip moves the very number the dip is measured
      // against, understating its depth.
      inDip = true;
      troughVelocity = speed;
      risingEdge = true;
    } else {
      // Asymmetric on purpose: follow the wheel up instantly, let it down slowly.
      //
      // A dip ends the moment the wheel turns the corner off its trough, which is still far below
      // where it started — so the reference has to climb with the recovery, or the next ball is
      // judged against a number from the bottom of the previous dip and vanishes. Following the
      // recovery upward keeps the reference at the recent peak, which is what a ball is a dip from.
      // Downward it decays slowly instead, so a wheel that settles into a sag is tracked without
      // that sag ever looking like an event.
      baselineVelocity =
          speed > baselineVelocity
              ? speed
              : baselineVelocity + baselineAlpha * (speed - baselineVelocity);
    }

    if (risingEdge && (lastEventMs == Long.MIN_VALUE || nowMs - lastEventMs >= refractoryMs)) {
      lastEventMs = nowMs;
      eventCount++;
      return true;
    }
    return false;
  }

  /** Reference speed the next dip is measured against; NaN until the first sample. */
  public double getBaselineVelocity() {
    return baselineVelocity;
  }

  /** Most recent sample's depth below baseline, as a fraction. Negative means above it. */
  public double getLastDipFraction() {
    return lastDipFraction;
  }

  /**
   * Deepest dip seen since the last reset.
   *
   * <p>This is how the threshold gets set without guessing: if real balls are bottoming out at 8%
   * and the threshold is 5%, it is in the right place; if the deepest dip all shot was 3%, the
   * threshold is above the signal and the count is going to read zero.
   */
  public double getDeepestDipFraction() {
    return deepestDipFraction;
  }

  public boolean isInDip() {
    return inDip;
  }

  public int getEventCount() {
    return eventCount;
  }

  /** Clears reference, count, and dip statistics for a new shot. */
  public void reset() {
    baselineVelocity = Double.NaN;
    troughVelocity = Double.NaN;
    inDip = false;
    lastEventMs = Long.MIN_VALUE;
    eventCount = 0;
    lastDipFraction = 0.0;
    deepestDipFraction = 0.0;
  }
}
