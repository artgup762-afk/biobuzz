package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.*;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.utilities.FlywheelDipDetector;
import org.firstinspires.ftc.teamcode.utilities.ShotTimingTrial;
import org.junit.jupiter.api.Test;

class ShotTimingTrialTest {
  private ShotTimingTrial trial(int balls) {
    return new ShotTimingTrial(new FlywheelDipDetector(.09, .05, .03, 120), balls, 0);
  }

  @Test
  void countsFourDistinctDipsAndWaitsForFinalRecovery() {
    var t = trial(4);
    assertFalse(t.update(200, false, 100));
    assertFalse(t.update(150, false, 200));
    assertTrue(t.ballTimesMs().isEmpty());
    assertFalse(t.armed());
    t.update(1000, true, 300);
    assertTrue(t.armed());
    for (int i = 1; i <= 4; i++) {
      assertTrue(t.update(800, false, i * 1000L));
      assertFalse(t.update(750, false, i * 1000L + 20));
      assertEquals(ShotTimingTrial.State.RUNNING, t.state());
      t.update(1000, true, i * 1000L + 200);
    }
    assertEquals(ShotTimingTrial.State.COMPLETE, t.state());
    assertEquals(List.of(1000, 2000, 3000, 4000), t.ballTimesMs());
    assertEquals(4200, t.elapsedMs());
    assertFalse(t.update(700, false, 5000));
    t.cancel();
    assertEquals(ShotTimingTrial.State.COMPLETE, t.state());
    assertEquals(4200, t.elapsedMs());
  }

  @Test
  void noRecoveryOrMissingBallsCannotBecomeAnAcceptedResult() {
    var t = trial(1);
    t.update(1000, true, 0);
    t.update(800, false, 200);
    t.update(750, false, 500);
    assertEquals(ShotTimingTrial.State.RUNNING, t.state());
    t.update(750, false, 60000);
    assertEquals(ShotTimingTrial.State.RUNNING, t.state());
    var noSpinUp = trial(4);
    noSpinUp.update(100, false, 500);
    assertEquals(ShotTimingTrial.State.RUNNING, noSpinUp.state());
  }

  @Test
  void cancelStaysStoppedAndEachTrialStartsFresh() {
    var t = trial(4);
    t.cancel();
    assertFalse(t.update(1000, true, 100));
    assertEquals(ShotTimingTrial.State.CANCELLED, t.state());
    assertTrue(t.ballTimesMs().isEmpty());
    assertEquals(ShotTimingTrial.State.RUNNING, trial(4).state());
    assertThrows(IllegalArgumentException.class, () -> trial(0));
  }

  @Test
  void noTimeLimitWaitsPastOldDeadlineForAllFourShots() {
    var t = trial(4);
    t.update(1000, true, 300);
    for (int time : new int[] {5729, 6816}) {
      assertTrue(t.update(800, false, time));
      t.update(1000, true, time + 200);
    }
    t.update(1000, true, 9500);
    assertEquals(ShotTimingTrial.State.RUNNING, t.state());
    t.update(1000, true, 60000);
    assertEquals(2, t.ballTimesMs().size());
    assertEquals(ShotTimingTrial.State.RUNNING, t.state());
    assertTrue(t.update(800, false, 61000));
    t.update(1000, true, 61200);
    assertTrue(t.update(800, false, 65000));
    t.update(1000, true, 65200);
    assertEquals(ShotTimingTrial.State.COMPLETE, t.state());
    assertEquals(List.of(5729, 6816, 61000, 65000), t.ballTimesMs());
  }

  @Test
  void finalSmallReboundIsNotFullRecoveryAndCannotAddAFifthBall() {
    var t = trial(1);
    t.update(1000, true, 0);
    assertTrue(t.update(800, false, 200));
    t.update(850, false, 300); // Detector re-arms, but flywheel is still below its operating band.
    assertEquals(ShotTimingTrial.State.RUNNING, t.state());
    assertFalse(t.update(600, false, 500));
    assertEquals(List.of(200), t.ballTimesMs());
    t.update(1000, true, 700);
    assertEquals(ShotTimingTrial.State.COMPLETE, t.state());
  }

  @Test
  void zeroSpeedDipCanRecoverWithoutCountingAgainWhileStopped() {
    var t = trial(1);
    t.update(1000, true, 0);
    assertTrue(t.update(0, false, 200));
    assertFalse(t.update(0, false, 400));
    assertEquals(ShotTimingTrial.State.RUNNING, t.state());
    assertFalse(t.update(1000, true, 600));
    assertEquals(ShotTimingTrial.State.COMPLETE, t.state());
    assertEquals(List.of(200), t.ballTimesMs());
  }

  @Test
  void unlimitedTrialStillCancelsWithoutSpinUpOrFourShots() {
    var t = trial(4);
    t.update(0, false, 60000);
    assertEquals(ShotTimingTrial.State.RUNNING, t.state());
    assertTrue(t.ballTimesMs().isEmpty());
    t.cancel();
    t.update(1000, true, 61000);
    assertEquals(ShotTimingTrial.State.CANCELLED, t.state());
  }

  @Test
  void recordedFourDipRunDoesNotFinishAt620TicksPerSecond() throws IOException {
    var t = trial(4);
    // Numeric, unquoted trace emitted by the robot. The previous build stopped at its final row.
    try (var reader =
        new BufferedReader(
            new InputStreamReader(
                Objects.requireNonNull(
                    getClass().getResourceAsStream("/shot-timing-early-stop.csv")),
                StandardCharsets.UTF_8))) {
      for (String line : reader.lines().skip(1).toList()) {
        String[] fields = line.split(",");
        t.update(Double.parseDouble(fields[3]), fields[9].equals("1"), Long.parseLong(fields[2]));
      }
    }
    assertEquals(List.of(1940, 2595, 3204, 3411), t.ballTimesMs());
    assertEquals(3487, t.elapsedMs());
    assertEquals(ShotTimingTrial.State.RUNNING, t.state());
    // Synthetic continuation establishes the required stop condition, not a measured recovery time.
    t.update(1000, true, 3600);
    assertEquals(ShotTimingTrial.State.COMPLETE, t.state());
  }
}
