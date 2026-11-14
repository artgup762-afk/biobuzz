package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.Nullable;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Bench test for a turret driven by two positional servos ({@code turnservo1}, {@code turnservo2})
 * with a Pinpoint on the turret ({@code turnpinpoint}) measuring its yaw.
 *
 * <p>Both servos drive the same turret, so if both are powered with positions that disagree they
 * fight each other and stall. Nothing here powers both until each has been measured alone: a servo
 * that is not being tested has its PWM disabled, so it goes limp and is back-driven.
 *
 * <ul>
 *   <li>[A] / [B]: measure servo 1 / 2 alone (the other limp). It goes to CENTER, then CENTER -
 *       DELTA, CENTER, CENTER + DELTA, CENTER, and fits yaw = gain * position + offset. The sign of
 *       the gain is the servo's direction; the two visits to CENTER from opposite sides give the
 *       backlash; the CENTER to CENTER + DELTA move gives the step response time.
 *   <li>[X]: with both measured, powers both at the positions their fits give for the turret's
 *       current yaw, then steps the target out by STEP_DEG until the turret stops following (a hard
 *       stop: it limps both servos at once), a servo runs out of range, or MAX_SWEEP_DEG; then the
 *       other way. Reports the reachable range and how closely the turret follows.
 *   <li>[Y]: limp both servos (abort). Any procedure also limps both if the yaw passes ABORT_DEG.
 *   <li>[dpad up]: zero the yaw here (turret still).
 * </ul>
 *
 * Everything runs inside the OpMode loop, so a stall is caught within a loop, not after a round
 * trip to a laptop.
 */
@TeleOp(name = "Turret Servo Test", group = "Tuning")
@Configurable
public class TurretServoTestOpMode extends LinearOpMode {

  public static double CENTER_1 = 0.5;
  public static double CENTER_2 = 0.5;

  /** Servo-position step for the single-servo measurement. */
  public static double DELTA = 0.03;

  /** Time to hold each position before averaging the yaw, and the averaging window. */
  public static double SETTLE_SEC = 0.6;

  public static double AVERAGE_SEC = 0.15;

  /** Range sweep: target step, time per step, and how far the yaw may lag before it is a stop. */
  public static double STEP_DEG = 1.0;

  public static double STEP_SEC = 0.25;
  public static double TRACK_TOL_DEG = 5.0;
  public static double MAX_SWEEP_DEG = 75.0;

  /** Any procedure limps both servos once |yaw| passes this. */
  public static double ABORT_DEG = 90.0;

  /**
   * Slow sweep ([dpad left] servo 1, [dpad right] servo 2, the other limp): position step, time per
   * step, the window of steps with less than LIMIT_MOVE_DEG of yaw change that means a stop.
   */
  public static double SLOW_STEP = 0.001;

  public static double SLOW_DWELL_SEC = 0.06;

  /**
   * Rate sweep ([left bumper] servo 1, [right bumper] servo 2, the other limp): the command creeps
   * away from NEUTRAL until the turret turns at RATE_DPS, and is held at that speed. A turret that
   * stops while being pushed (under STALL_DPS for STALL_SEC after it has moved) is at a stop: the
   * servo is limped at once. Works for a continuous servo (command = speed) and a positional one.
   */
  public static double NEUTRAL = 0.5;

  public static double RATE_DPS = 10.0;
  public static double CREEP_PER_SEC = 0.01;
  public static double BACKOFF_PER_SEC = 0.05;
  public static double MAX_OFFSET = 0.06;
  public static double STALL_DPS = 1.0;
  public static double STALL_SEC = 0.3;
  public static double LEG_TIMEOUT_SEC = 20.0;

  /**
   * Jog sweep ([left bumper] servo 1, [right bumper] servo 2): the turret sticks until the command
   * is a few thousandths off NEUTRAL, then lurches, so it is moved in jogs. Each jog creeps the
   * command at JOG_CREEP_PER_SEC until the yaw has moved JOG_DETECT_DEG, returns to NEUTRAL and
   * waits JOG_SETTLE_SEC. A jog that reaches twice the largest offset that has moved the turret (at
   * least JOG_MIN_LIMIT_OFFSET) without moving it is at a stop; a jog that travels more than
   * JOG_MAX_DEG aborts.
   */
  public static double JOG_CREEP_PER_SEC = 0.006;

  public static double JOG_DETECT_DEG = 0.4;
  public static double JOG_SETTLE_SEC = 0.35;
  public static double JOG_MIN_LIMIT_OFFSET = 0.012;
  public static double JOG_MAX_DEG = 15.0;
  public static int LIMIT_WINDOW_STEPS = 6;
  public static double LIMIT_MOVE_DEG = 0.8;

  /** One servo's fit: yaw (deg) = gain * position + offset. */
  private record Fit(
      double gain, double offset, double linearityDeg, double backlashDeg, double t90Sec) {
    double positionFor(double yawDeg) {
      return (yawDeg - offset) / gain;
    }
  }

  private ServoImplEx s1;
  private ServoImplEx s2;
  private GoBildaPinpointDriver imu;
  private @Nullable Fit fit1;
  private @Nullable Fit fit2;
  private String status = "idle: both servos limp";
  private final List<String> results = new ArrayList<>();
  private boolean aborted;
  private double yaw;

  @Override
  @SuppressWarnings("NullAway.Init") // Disabled test returns before any hardware-dependent method.
  public void runOpMode() {
    config.reload();
    OpModeUtil.reportConfig(telemetry);
    if (!config.turret.turn_pinpoint_enabled) {
      telemetry.addLine("Test unavailable: turret.turn_pinpoint_enabled is false.");
      telemetry.update();
      return;
    }
    // get, not tryGet: a missing device fails INIT with the SDK's "unable to find" message.
    s1 = hardwareMap.get(ServoImplEx.class, "turnservo1");
    s2 = hardwareMap.get(ServoImplEx.class, "turnservo2");
    imu = hardwareMap.get(GoBildaPinpointDriver.class, "turnpinpoint");
    limpBoth();
    imu.resetPosAndIMU();
    sleep(500);

    while (opModeInInit()) {
      readYaw();
      show();
    }

    while (opModeIsActive()) {
      readYaw();
      aborted = false;
      if (gamepad1.aWasPressed()) {
        fit1 = measure(s1, CENTER_1, "servo 1");
      } else if (gamepad1.bWasPressed()) {
        fit2 = measure(s2, CENTER_2, "servo 2");
      } else if (gamepad1.xWasPressed()) {
        sweep();
      } else if (gamepad1.dpadLeftWasPressed()) {
        fit1 = slowSweep(s1, CENTER_1, "servo 1");
      } else if (gamepad1.dpadRightWasPressed()) {
        fit2 = slowSweep(s2, CENTER_2, "servo 2");
      } else if (gamepad1.leftBumperWasPressed()) {
        jogSweep(s1, "servo 1");
      } else if (gamepad1.rightBumperWasPressed()) {
        jogSweep(s2, "servo 2");
      } else if (gamepad1.yWasPressed()) {
        limpBoth();
        status = "idle: both servos limp";
      } else if (gamepad1.dpadUpWasPressed()) {
        limpBoth();
        imu.resetPosAndIMU();
        sleep(500);
        status = "yaw zeroed";
      }
      show();
    }
    limpBoth();
  }

  private void limpBoth() {
    s1.setPwmDisable();
    s2.setPwmDisable();
  }

  private void readYaw() {
    imu.update();
    yaw = imu.getHeading(UnnormalizedAngleUnit.DEGREES);
  }

  /**
   * Holds for {@code seconds} (reading the yaw and watching for an abort), then returns the mean
   * yaw over the last AVERAGE_SEC. NaN once aborted.
   */
  private double hold(double seconds) {
    return hold(seconds, null);
  }

  private double hold(double seconds, @Nullable List<double[]> trace) {
    ElapsedTime t = new ElapsedTime();
    double sum = 0;
    int n = 0;
    while (opModeIsActive() && t.seconds() < seconds) {
      readYaw();
      if (Math.abs(yaw) > ABORT_DEG || gamepad1.yWasPressed()) {
        limpBoth();
        aborted = true;
        status =
            String.format(
                Locale.ROOT, "ABORTED at yaw %.1f deg (limit %.0f or [Y])", yaw, ABORT_DEG);
        return Double.NaN;
      }
      if (trace != null) {
        trace.add(new double[] {t.seconds(), yaw});
      }
      if (t.seconds() >= seconds - AVERAGE_SEC) {
        sum += yaw;
        n++;
      }
      show();
    }
    return n > 0 ? sum / n : yaw;
  }

  /** One jog: how far it moved and the command offset that broke the turret loose (NaN if none). */
  private record Jog(double movedDeg, double breakOffset) {}

  /**
   * One jog: command NEUTRAL + cmdDir * offset with the offset creeping from 0 until the turret
   * moves JOG_DETECT_DEG, or the offset reaches {@code limitOffset}; then NEUTRAL and a settle.
   * Null once aborted.
   */
  private @Nullable Jog jog(ServoImplEx s, int cmdDir, double limitOffset, String label) {
    s.setPosition(NEUTRAL);
    s.setPwmEnable();
    double start = yaw;
    double offset = 0;
    double breakOffset = Double.NaN;
    ElapsedTime loop = new ElapsedTime();
    while (opModeIsActive()) {
      double dt = loop.seconds();
      loop.reset();
      readYaw();
      if (Math.abs(yaw) > ABORT_DEG || gamepad1.yWasPressed()) {
        limpBoth();
        aborted = true;
        status = String.format(Locale.ROOT, "%s ABORTED at yaw %.1f", label, yaw);
        return null;
      }
      if (Math.abs(yaw - start) >= JOG_DETECT_DEG) {
        breakOffset = offset;
        break;
      }
      if (offset >= limitOffset) {
        break;
      }
      offset = Math.min(limitOffset, offset + JOG_CREEP_PER_SEC * dt);
      s.setPosition(NEUTRAL + cmdDir * offset);
      status =
          String.format(
              Locale.ROOT,
              "%s jog %s: command %.4f",
              label,
              cmdDir > 0 ? "up" : "down",
              NEUTRAL + cmdDir * offset);
      show();
    }
    s.setPosition(NEUTRAL);
    if (Double.isNaN(hold(JOG_SETTLE_SEC))) {
      return null;
    }
    double moved = yaw - start;
    if (Math.abs(moved) > JOG_MAX_DEG) {
      limpBoth();
      aborted = true;
      status = String.format(Locale.ROOT, "%s ABORTED: one jog moved %.1f deg", label, moved);
      return null;
    }
    return new Jog(moved, breakOffset);
  }

  /** Result of jogging one way until a stop. */
  private record JogLeg(
      double endYaw, int jogs, double meanJogDeg, double maxBreakOffset, String why) {}

  private @Nullable JogLeg jogLeg(ServoImplEx s, int cmdDir, double untilYaw, String label) {
    double maxBreak = 0;
    double total = 0;
    int jogs = 0;
    double towardTarget = Math.signum(untilYaw - yaw);
    while (opModeIsActive() && jogs < 200) {
      if (!Double.isNaN(untilYaw) && (yaw - untilYaw) * towardTarget >= 0) {
        return new JogLeg(yaw, jogs, jogs > 0 ? total / jogs : 0, maxBreak, "reached target");
      }
      double limitOffset = Math.max(JOG_MIN_LIMIT_OFFSET, 2 * maxBreak);
      Jog j = jog(s, cmdDir, limitOffset, label);
      if (j == null) {
        return null;
      }
      if (Double.isNaN(j.breakOffset())) {
        // Pushed to twice what has moved it, at a barely-moving command, and nothing moved.
        s.setPwmDisable();
        return new JogLeg(yaw, jogs, jogs > 0 ? total / jogs : 0, maxBreak, "stop");
      }
      maxBreak = Math.max(maxBreak, j.breakOffset());
      total += Math.abs(j.movedDeg());
      jogs++;
    }
    s.setPwmDisable();
    return new JogLeg(yaw, jogs, jogs > 0 ? total / jogs : 0, maxBreak, "jog limit");
  }

  /** Jog one servo (the other limp) up to a stop, down to the other stop, then to the middle. */
  private void jogSweep(ServoImplEx s, String label) {
    limpBoth();
    hold(0.3);
    double start = yaw;
    JogLeg up = jogLeg(s, 1, Double.NaN, label);
    if (up == null) {
      return;
    }
    JogLeg down = jogLeg(s, -1, Double.NaN, label);
    if (down == null) {
      return;
    }
    double upTurn = Math.abs(up.endYaw() - start) > 3 ? up.endYaw() - start : start - down.endYaw();
    int sign = upTurn >= 0 ? 1 : -1;
    double mid = (up.endYaw() + down.endYaw()) / 2;
    int cmdDir = (int) Math.signum(mid - yaw) * sign;
    if (cmdDir != 0) {
      jogLeg(s, cmdDir, mid, label);
    }
    s.setPwmDisable();
    results.add(
        String.format(
            Locale.ROOT,
            "%s jog: higher command turns yaw %s; command-up end %+.1f (%s, %d jogs of %.1f deg, "
                + "breaks loose at +%.4f), command-down end %+.1f (%s, %d jogs of %.1f deg, at "
                + "-%.4f); range %.1f deg",
            label,
            sign > 0 ? "+" : "-",
            up.endYaw(),
            up.why(),
            up.jogs(),
            up.meanJogDeg(),
            up.maxBreakOffset(),
            down.endYaw(),
            down.why(),
            down.jogs(),
            down.meanJogDeg(),
            down.maxBreakOffset(),
            Math.abs(up.endYaw() - down.endYaw())));
    status = label + " jog sweep done; limp, turret near the middle";
  }

  /** Outcome of one rate-controlled leg: where it ended, the offset that first moved it, why. */
  private record Leg(double endYaw, double startOffset, String why) {}

  /**
   * Turns the turret with command NEUTRAL + cmdDir * offset at RATE_DPS (speed, either direction)
   * until it stops while pushed (a stop: limps the servo at once) or passes {@code targetYaw} (NaN
   * for none: then NEUTRAL). The offset creeps up while the turret is slower than RATE_DPS and
   * backs off while it is faster. Speed is |yaw rate|, so a servo whose + command turns the turret
   * either way is held to RATE_DPS. Null once aborted (|yaw| past ABORT_DEG, speed past 3 RATE_DPS,
   * [Y]).
   */
  private @Nullable Leg rateLeg(ServoImplEx s, int cmdDir, double targetYaw, String label) {
    s.setPosition(NEUTRAL);
    s.setPwmEnable();
    ElapsedTime t = new ElapsedTime();
    ElapsedTime loop = new ElapsedTime();
    double offset = 0;
    double startOffset = Double.NaN;
    double legStart = yaw;
    double towardTarget = Math.signum(targetYaw - legStart);
    double slowSince = Double.NaN;
    double speed = 0;
    while (opModeIsActive()) {
      double dt = loop.seconds();
      loop.reset();
      readYaw();
      double raw = Math.abs(imu.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
      speed += (raw - speed) * Math.min(1.0, dt / 0.05);
      if (Math.abs(yaw) > ABORT_DEG || speed > 3 * RATE_DPS || gamepad1.yWasPressed()) {
        limpBoth();
        aborted = true;
        status =
            String.format(
                Locale.ROOT,
                "%s ABORTED: yaw %.1f, speed %.1f deg/s (limits %.0f deg, %.0f deg/s) or [Y]",
                label,
                yaw,
                speed,
                ABORT_DEG,
                3 * RATE_DPS);
        return null;
      }
      if (!Double.isNaN(targetYaw) && (yaw - targetYaw) * towardTarget >= 0) {
        s.setPosition(NEUTRAL);
        return new Leg(yaw, startOffset, "reached target");
      }
      if (t.seconds() > LEG_TIMEOUT_SEC) {
        s.setPwmDisable();
        return new Leg(yaw, startOffset, "timed out");
      }
      if (Double.isNaN(startOffset) && speed > RATE_DPS / 2) {
        startOffset = offset;
      }
      boolean moved = Math.abs(yaw - legStart) > 3.0;
      if (!moved && offset > MAX_OFFSET / 2) {
        // Pushed harder and harder without moving: already at a stop, or not connected.
        s.setPwmDisable();
        return new Leg(yaw, startOffset, "never moved (already at a stop?)");
      }
      if (moved && speed < STALL_DPS) {
        if (Double.isNaN(slowSince)) {
          slowSince = t.seconds();
        } else if (t.seconds() - slowSince > STALL_SEC) {
          s.setPwmDisable();
          return new Leg(yaw, startOffset, "stopped: at a stop");
        }
      } else {
        slowSince = Double.NaN;
      }
      offset += (speed < RATE_DPS ? CREEP_PER_SEC : -BACKOFF_PER_SEC) * dt;
      offset = Math.max(0, Math.min(MAX_OFFSET, offset));
      s.setPosition(NEUTRAL + cmdDir * offset);
      status =
          String.format(
              Locale.ROOT,
              "%s rate sweep, command %s: %.4f, %.1f deg/s",
              label,
              cmdDir > 0 ? "up" : "down",
              NEUTRAL + cmdDir * offset,
              speed);
      show();
    }
    return null;
  }

  /**
   * Rate sweep of one servo, the other limp: command up to a stop, command down to the other stop,
   * then back to the middle. Reports which way a higher command turns the turret.
   */
  private void rateSweep(ServoImplEx s, String label) {
    limpBoth();
    hold(0.3);
    double start = yaw;
    Leg up = rateLeg(s, 1, Double.NaN, label);
    if (up == null || Double.isNaN(hold(0.3))) {
      return;
    }
    Leg down = rateLeg(s, -1, Double.NaN, label);
    if (down == null || Double.isNaN(hold(0.3))) {
      return;
    }
    // Which way a higher command turns the turret, from the up leg (or the down leg if the turret
    // started against the up-side stop).
    double upTurn = Math.abs(up.endYaw() - start) > 3 ? up.endYaw() - start : start - down.endYaw();
    int sign = upTurn >= 0 ? 1 : -1;
    double mid = (up.endYaw() + down.endYaw()) / 2;
    int cmdDir = (int) Math.signum(mid - yaw) * sign;
    if (cmdDir != 0) {
      rateLeg(s, cmdDir, mid, label);
      hold(0.3);
    }
    s.setPwmDisable();
    results.add(
        String.format(
            Locale.ROOT,
            "%s rate sweep: higher command turns yaw %s; command-up end %+.1f (%s), command-down "
                + "end %+.1f (%s), range %.1f deg; moving starts %.4f above / %.4f below %.3f",
            label,
            sign > 0 ? "+" : "-",
            up.endYaw(),
            up.why(),
            down.endYaw(),
            down.why(),
            Math.abs(up.endYaw() - down.endYaw()),
            up.startOffset(),
            down.startOffset(),
            NEUTRAL));
    status = label + " rate sweep done; limp, turret near the middle";
  }

  /** One sample of a slow sweep. */
  private record Sample(double pos, double yaw, int dir) {}

  /**
   * Ramps one servo (the other limp) slowly from {@code start} up until the turret stops following
   * (a stop: limps at once), then from there down to the other stop, then back to the middle of the
   * two, and fits yaw = gain * position + offset over the travel between them.
   */
  private @Nullable Fit slowSweep(ServoImplEx s, double start, String label) {
    limpBoth();
    hold(0.3);
    status = label + ": to start " + start;
    s.setPwmEnable();
    s.setPosition(start);
    if (Double.isNaN(hold(1.0))) {
      return null;
    }
    List<Sample> samples = new ArrayList<>();
    double[] limitPos = new double[2];
    double[] limitYaw = new double[2];
    String[] why = new String[2];
    double pos = start;
    for (int leg = 0; leg < 2; leg++) {
      int dir = leg == 0 ? 1 : -1;
      List<Double> legYaw = new ArrayList<>();
      double legStartYaw = yaw;
      why[leg] = "servo position range ends";
      while (opModeIsActive()) {
        double next = pos + dir * SLOW_STEP;
        if (next < 0 || next > 1) {
          break;
        }
        pos = next;
        s.setPosition(pos);
        status =
            String.format(
                Locale.ROOT,
                "%s: slow sweep %s, position %.3f",
                label,
                dir > 0 ? "up" : "down",
                pos);
        double y = hold(SLOW_DWELL_SEC);
        if (Double.isNaN(y)) {
          return null;
        }
        legYaw.add(y);
        samples.add(new Sample(pos, y, dir));
        int n = legYaw.size();
        if (n > LIMIT_WINDOW_STEPS) {
          double recent = Math.abs(y - legYaw.get(n - 1 - LIMIT_WINDOW_STEPS));
          boolean moved = Math.abs(y - legStartYaw) > 3 * LIMIT_MOVE_DEG;
          if (recent < LIMIT_MOVE_DEG && moved) {
            // Pushing into a stop: stop pushing at once, and resume from where it last moved.
            s.setPwmDisable();
            why[leg] = "turret stopped following";
            pos -= dir * SLOW_STEP * LIMIT_WINDOW_STEPS;
            break;
          }
          if (!moved && n > 40) {
            limpBoth();
            status = label + ": 0.04 of position moved the turret under 2.4 deg: stopped";
            return null;
          }
        }
      }
      limitPos[leg] = pos;
      limitYaw[leg] = yaw;
      // Resume powered at the last position that moved the turret (about where it now rests).
      s.setPosition(pos);
      s.setPwmEnable();
      if (Double.isNaN(hold(0.3))) {
        return null;
      }
    }
    // Back to the middle, slowly.
    double mid = (limitPos[0] + limitPos[1]) / 2;
    while (opModeIsActive() && Math.abs(pos - mid) > SLOW_STEP) {
      pos += Math.signum(mid - pos) * SLOW_STEP;
      s.setPosition(pos);
      if (Double.isNaN(hold(SLOW_DWELL_SEC))) {
        return null;
      }
    }
    hold(0.5);
    s.setPwmDisable();

    // Fit over the samples clear of both stops (5 deg margin), both directions.
    double lo = Math.min(limitYaw[0], limitYaw[1]) + 5, hi = Math.max(limitYaw[0], limitYaw[1]) - 5;
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    int n = 0;
    for (Sample p : samples) {
      if (p.yaw() > lo && p.yaw() < hi) {
        sx += p.pos();
        sy += p.yaw();
        sxx += p.pos() * p.pos();
        sxy += p.pos() * p.yaw();
        n++;
      }
    }
    if (n < 5) {
      results.add(label + ": too little travel between the stops to fit");
      status = label + " slow sweep done; limp";
      return null;
    }
    double gain = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    double offset = (sy - gain * sx) / n;
    // Up-ramp minus down-ramp residual: the lag and backlash between the two directions.
    double up = 0, down = 0, worst = 0;
    int nu = 0, nd = 0;
    for (Sample p : samples) {
      if (p.yaw() > lo && p.yaw() < hi) {
        double r = p.yaw() - (gain * p.pos() + offset);
        worst = Math.max(worst, Math.abs(r));
        if (p.dir() > 0) {
          up += r;
          nu++;
        } else {
          down += r;
          nd++;
        }
      }
    }
    double hysteresis = nu > 0 && nd > 0 ? Math.abs(up / nu - down / nd) : Double.NaN;
    results.add(
        String.format(
            Locale.ROOT,
            "%s slow: stops at yaw %+.1f (pos %.3f, %s) and %+.1f (pos %.3f, %s); "
                + "%+.2f deg per 0.01 (+position turns yaw %s); up/down difference %.1f deg, "
                + "worst residual %.1f deg",
            label,
            limitYaw[0],
            limitPos[0],
            why[0],
            limitYaw[1],
            limitPos[1],
            why[1],
            gain * 0.01,
            gain > 0 ? "+" : "-",
            hysteresis,
            worst));
    status = label + " slow sweep done; limp";
    return new Fit(gain, offset, worst, hysteresis, Double.NaN);
  }

  /** Measures one servo with the other limp; see the class comment. */
  private @Nullable Fit measure(ServoImplEx s, double center, String label) {
    limpBoth();
    hold(0.3);
    status = label + ": moving to CENTER " + center;
    s.setPwmEnable();
    s.setPosition(center);
    if (Double.isNaN(hold(1.0))) {
      return null;
    }
    double[] pos = {center - DELTA, center, center + DELTA, center};
    double[] seen = new double[pos.length];
    double t90 = Double.NaN;
    for (int i = 0; i < pos.length; i++) {
      double before = yaw;
      status = String.format(Locale.ROOT, "%s: position %.3f", label, pos[i]);
      s.setPosition(pos[i]);
      List<double[]> trace = i == 2 ? new ArrayList<>() : null;
      seen[i] = hold(SETTLE_SEC, trace);
      if (Double.isNaN(seen[i])) {
        return null;
      }
      // Every step after the first is a DELTA change of position: a turret that does not move
      // is held by a hard stop (the servo is stalled against it) or the servo is not connected.
      if (i > 0 && Math.abs(seen[i] - seen[i - 1]) < 0.5) {
        limpBoth();
        status =
            String.format(
                Locale.ROOT,
                "%s: a %.3f step moved the turret only %.2f deg: at a stop or not connected; limp",
                label,
                DELTA,
                Math.abs(seen[i] - seen[i - 1]));
        return null;
      }
      if (trace != null) {
        t90 = time90(trace, before, seen[i]);
      }
    }
    s.setPwmDisable();
    // Least squares over the three distinct positions (the second CENTER visit is for backlash).
    double mx = (pos[0] + pos[1] + pos[2]) / 3, my = (seen[0] + seen[1] + seen[2]) / 3;
    double sxy = 0, sxx = 0;
    for (int i = 0; i < 3; i++) {
      sxy += (pos[i] - mx) * (seen[i] - my);
      sxx += (pos[i] - mx) * (pos[i] - mx);
    }
    double gain = sxy / sxx;
    double offset = my - gain * mx;
    double lin = 0;
    for (int i = 0; i < 3; i++) {
      lin = Math.max(lin, Math.abs(seen[i] - (gain * pos[i] + offset)));
    }
    Fit f = new Fit(gain, offset, lin, Math.abs(seen[3] - seen[1]), t90);
    results.add(
        String.format(
            Locale.ROOT,
            "%s: %+.1f deg per 0.1 position (+position turns yaw %s), yaw %.1f at %.2f, "
                + "nonlinearity %.2f deg, backlash %.2f deg, %.0f%% step in %.0f ms",
            label,
            gain * 0.1,
            gain > 0 ? "+" : "-",
            gain * center + offset,
            center,
            lin,
            f.backlashDeg(),
            90.0,
            t90 * 1000));
    status = label + " measured; now limp";
    return f;
  }

  /** First time the trace covers 90% of the move from {@code before} to {@code after}. */
  private static double time90(List<double[]> trace, double before, double after) {
    double goal = before + 0.9 * (after - before);
    for (double[] p : trace) {
      if ((after - before) * (p[1] - goal) >= 0) {
        return p[0];
      }
    }
    return Double.NaN;
  }

  /** Both servos together, sweeping out each way until the turret stops following. */
  private void sweep() {
    Fit f1 = fit1;
    Fit f2 = fit2;
    if (f1 == null || f2 == null) {
      status = "measure servo 1 [A] and servo 2 [B] first";
      return;
    }
    if (Math.abs(f1.gain()) < 1 || Math.abs(f2.gain()) < 1) {
      status = "a servo barely moved the turret in its measurement: not sweeping";
      return;
    }
    double start = Math.round(yaw);
    double[] limits = new double[2];
    String[] why = new String[2];
    double worstErr = 0;
    double sumSq = 0;
    int n = 0;
    for (int dir = 0; dir < 2 && !aborted; dir++) {
      double sign = dir == 0 ? 1 : -1;
      double target = start;
      if (!command(f1, f2, target)) {
        why[dir] = "start outside servo range";
        break;
      }
      s1.setPwmEnable();
      s2.setPwmEnable();
      if (Double.isNaN(hold(0.6))) {
        return;
      }
      double lastGood = yaw;
      int misses = 0;
      why[dir] = "MAX_SWEEP_DEG";
      while (opModeIsActive() && Math.abs(target + sign * STEP_DEG) <= MAX_SWEEP_DEG) {
        target += sign * STEP_DEG;
        if (!command(f1, f2, target)) {
          why[dir] = "servo position range ends";
          break;
        }
        status =
            String.format(Locale.ROOT, "sweep %s: target %.0f deg", sign > 0 ? "+" : "-", target);
        double got = hold(STEP_SEC);
        if (Double.isNaN(got)) {
          return;
        }
        double err = got - target;
        if (Math.abs(err) > TRACK_TOL_DEG) {
          if (++misses >= 2) {
            // Stalled against something: stop pushing at once.
            limpBoth();
            why[dir] = String.format(Locale.ROOT, "stopped following (error %.1f deg)", err);
            break;
          }
        } else {
          misses = 0;
          lastGood = got;
          worstErr = Math.max(worstErr, Math.abs(err));
          sumSq += err * err;
          n++;
        }
      }
      limits[dir] = lastGood;
      // Back to the start with both servos before the other direction.
      if (command(f1, f2, start)) {
        s1.setPwmEnable();
        s2.setPwmEnable();
        hold(0.8);
      }
    }
    limpBoth();
    results.add(
        String.format(
            Locale.ROOT,
            "range: %+.1f deg (%s) to %+.1f deg (%s); following error rms %.2f, worst %.2f deg",
            limits[1],
            why[1],
            limits[0],
            why[0],
            n > 0 ? Math.sqrt(sumSq / n) : Double.NaN,
            worstErr));
    status = "sweep done; both limp";
  }

  /**
   * Sets both servos to their fits' positions for {@code yawDeg}; false if either is out of [0,1].
   */
  private boolean command(Fit f1, Fit f2, double yawDeg) {
    double p1 = f1.positionFor(yawDeg);
    double p2 = f2.positionFor(yawDeg);
    if (p1 < 0 || p1 > 1 || p2 < 0 || p2 > 1) {
      return false;
    }
    s1.setPosition(p1);
    s2.setPosition(p2);
    return true;
  }

  private void show() {
    telemetry.addData("Turret yaw", "%.2f deg", yaw);
    telemetry.addData("Status", status);
    telemetry.addData(
        "Servo 1", "%s pos %.3f", s1.isPwmEnabled() ? "POWERED" : "limp", s1.getPosition());
    telemetry.addData(
        "Servo 2", "%s pos %.3f", s2.isPwmEnabled() ? "POWERED" : "limp", s2.getPosition());
    for (String r : results) {
      telemetry.addLine(r);
    }
    telemetry.addLine(
        "[LB]/[RB] jog sweep servo 1/2  [dpad L/R] slow sweep  [A]/[B] step  [X] both  [Y] limp");
    telemetry.update();
  }
}
