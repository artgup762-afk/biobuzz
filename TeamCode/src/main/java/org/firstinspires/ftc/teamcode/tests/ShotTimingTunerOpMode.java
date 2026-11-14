package org.firstinspires.ftc.teamcode.tests;

import android.os.Environment;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.CalibrationRay;
import org.firstinspires.ftc.teamcode.utilities.FlywheelDipDetector;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.firstinspires.ftc.teamcode.utilities.ShotTimingTrial;

@TeleOp(name = "Shot Timing Tuner", group = "Calibration")
@Configurable
public class ShotTimingTunerOpMode extends LinearOpMode {

  /** One accepted measurement for one distance. */
  public record TimingPoint(
      double distanceInches,
      int shootWindowMs,
      int firstBallMs,
      int lastBallMs,
      int ballCount,
      long timestampMs) {}

  /** One control loop inside one shot. Written straight out as a CSV row. */
  private record TraceRow(
      int shotId,
      double distanceInches,
      int elapsedMs,
      double velocity,
      double targetVelocity,
      double dipFraction,
      double baseline,
      boolean ballEvent,
      boolean feedCommanded,
      boolean flywheelReady,
      boolean aimed,
      boolean solverValid,
      double hoodTarget,
      double solvedRpm) {}

  private Robot robot;
  private List<LynxModule> allHubs;

  private final List<TimingPoint> trials = new ArrayList<>();
  private final List<TraceRow> trace = new ArrayList<>();

  /**
   * Starts at shooter.ball_detection.dip_fraction (9%, see above), the threshold the match code
   * counts with; the stick adjusts it for this run only. Set in runOpMode, after the config reload,
   * so a pushed value applies.
   */
  private double dipFraction;

  private int expectedBalls;
  private int shotId = 0;
  private boolean benchMode;

  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry);
    benchMode = config.shooter.timing_tuner.bench_mode;
    dipFraction = config.shooter.ball_detection.dip_fraction;
    expectedBalls = config.auto.balls_per_volley_count;

    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    MatchProfile profile = config.loadMatchProfile(Alliance.RED);
    robot = new Robot(hardwareMap, telemetry, profile);
    CalibrationRay ray =
        CalibrationRay.forCell(
            robot.field, HiveCell.A, config.shooter.timing_tuner.ray_off_axis_deg);
    OpModeUtil.setupTurretAndShooter(robot.turret, robot.shooter);

    telemetry.addLine("Shot Timing Tuner");
    telemetry.addLine("");
    if (benchMode) {
      telemetry.addLine("FIXED-SPEED BENCH TIMING — aim into a clear catch area before START.");
      telemetry.addLine("Turret stays idle. Both flywheels spin up, then the intake feeds.");
    } else {
      telemetry.addLine("The robot will NOT drive. Its pose is faked to each");
      telemetry.addLine("distance. It DOES fire and the turret DOES slew.");
    }
    telemetry.addData("Load before each shot", "%d balls", expectedBalls);
    if (!benchMode && !config.shooter.shot_table.calibrated) {
      telemetry.addLine("!! Shot table not calibrated: every shot is refused. Run Ballistics");
      telemetry.addLine("!! Calibration first and set shooter.shot_table.calibrated: true.");
    }
    telemetry.addLine("START begins one trial. RT retries; LT stops both motors and intake.");
    telemetry.addLine(
        "Time is measured from START through the last shot's recovery; LT/STOP cancels.");
    telemetry.update();

    waitForStart();

    double tableMin = Double.NEGATIVE_INFINITY;
    double tableMax = Double.POSITIVE_INFINITY;
    try {
      if (!benchMode) {
        ShotTable table = ShotTable.fromConfig(robot.shotController.getBallType());
        tableMin = table.minDistance();
        tableMax = table.maxDistance();
      }
    } catch (RuntimeException e) {
      // A malformed table is named on the INIT config report; shots refuse until it is fixed.
    }
    try {
      if (benchMode) {
        measureEndpoint(0, true);
        finish();
        return;
      }
      boolean firstEndpoint = true;
      for (double distance : config.shooter.timing_tuner.distances_in) {
        if (!opModeIsActive()) break;
        if (distance > ray.maxDistance()) {
          continue;
        }
        // Inside the measured table: Ballistics Calibration records the distance it actually
        // shot from (say 24.3 in), and the solver refuses anything outside the table's rows, so
        // an exact 24.0 in endpoint could never fire.
        double at = Math.clamp(distance, tableMin, tableMax);
        robot.follower.setPose(Turret.robotPoseForPivot(ray.waypoint(at)));
        measureEndpoint(at, firstEndpoint);
        firstEndpoint = false;
      }
      finish();
    } finally {
      stopEverything();
      robot.shutdown();
    }
  }

  /** Full wind-down: no shot, no feed, no flywheel, and it stays down until the next fire. */
  private void stopEverything() {
    robot.shotController.stopShot();
    robot.intake.stop();
    robot.shooter.setTargetPower(0.0);
  }

  private void tick() {
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }
    robot.update();
  }

  /** Runs one distance until a measurement is accepted or the distance is skipped. */
  private void measureEndpoint(double distance, boolean startRequested) {
    FlywheelDipDetector detector = newDetector();
    List<Integer> ballTimesMs = List.of();

    boolean shotRunning = false;
    boolean armed = false;
    ShotTimingTrial trial = new ShotTimingTrial(detector, expectedBalls, 0);
    trial.cancel();

    int resultFirstMs = -1;
    int resultLastMs = -1;
    int resultCount = 0;
    int resultWindowMs = -1;
    String status = "load " + expectedBalls + " balls";

    boolean intakeOn = false;

    boolean aPrev = true;
    boolean bPrev = true;
    boolean xPrev = true;
    boolean upPrev = true;
    boolean downPrev = true;
    boolean lsPrev = true;
    boolean rsPrev = true;
    boolean rtPrev = true;

    while (opModeIsActive()) {
      long nowMs = System.nanoTime() / 1_000_000;

      if (gamepad1.dpad_up && !upPrev && !shotRunning) {
        expectedBalls = Math.min(config.inventory.capacity_count, expectedBalls + 1);
      }
      upPrev = gamepad1.dpad_up;

      if (gamepad1.dpad_down && !downPrev && !shotRunning) {
        expectedBalls = Math.max(1, expectedBalls - 1);
      }
      downPrev = gamepad1.dpad_down;

      // The one control to reach for when the ball count disagrees with what you watched. The
      // "deepest dip" readout says which way to move it.
      if (gamepad1.right_stick_button && !rsPrev && !shotRunning) {
        dipFraction += config.shooter.timing_tuner.dip_step;
      }
      rsPrev = gamepad1.right_stick_button;

      if (gamepad1.left_stick_button && !lsPrev && !shotRunning) {
        dipFraction =
            Math.max(
                config.shooter.timing_tuner.dip_step,
                dipFraction - config.shooter.timing_tuner.dip_step);
      }
      lsPrev = gamepad1.left_stick_button;

      if (gamepad1.x && !xPrev && !shotRunning) {
        intakeOn = !intakeOn;
        if (intakeOn) {
          robot.intake.run(config.teleop.intake_power);
        } else {
          robot.intake.stop();
        }
      }
      xPrev = gamepad1.x;

      boolean rightTrigger = gamepad1.right_trigger > config.shooter.timing_tuner.trigger_threshold;
      boolean retryPressed = rightTrigger && !rtPrev;
      rtPrev = rightTrigger;
      boolean stopPressed = gamepad1.left_trigger > config.shooter.timing_tuner.trigger_threshold;

      // STOP wins even when START/RT is requested in the same loop.
      if (stopPressed) {
        trial.cancel();
        shotRunning = false;
        startRequested = false;
        intakeOn = false;
        status = "stopped by driver";
        stopEverything();
      } else if ((startRequested || retryPressed) && !shotRunning) {
        startRequested = false;
        detector = newDetector();
        trial = new ShotTimingTrial(detector, expectedBalls, nowMs);
        ballTimesMs = List.of();
        shotRunning = true;
        armed = false;
        intakeOn = false;
        resultFirstMs = resultLastMs = resultWindowMs = -1;
        resultCount = 0;
        shotId++;
        if (benchMode) {
          robot.turret.setAimMode(Turret.AimMode.IDLE);
          // Explicit manually aimed calibration shot. Match OpModes retain their solution gates.
          robot.shotController.startShot(Shooter.constantPower(), false, false, false);
        } else {
          robot.shotController.startShot(Shooter.constantPower(), true);
        }
        status =
            benchMode
                ? "spinning up; intake waits for speed"
                : "spinning up; feed waits for shot gates";
      }

      boolean flywheelReady = robot.shotController.isFlywheelReady();
      int elapsedMs = trial.elapsedMs();
      if (shotRunning) {
        boolean ball = trial.update(robot.shooter.getShooterVelocity(), flywheelReady, nowMs);
        armed = trial.armed();
        elapsedMs = trial.elapsedMs();
        ballTimesMs = trial.ballTimesMs();
        trace.add(row(shotId, distance, elapsedMs, detector, ball));
        if (trial.state() == ShotTimingTrial.State.COMPLETE) {
          shotRunning = false;
          stopEverything();
          resultFirstMs = ballTimesMs.get(0);
          resultLastMs = ballTimesMs.get(ballTimesMs.size() - 1);
          resultCount = ballTimesMs.size();
          resultWindowMs = elapsedMs;
          status =
              String.format(
                  Locale.ROOT, "all %d detected; recovery complete; motors stopped", expectedBalls);
          saveProgress();
        }
      }

      tick();
      render(
          distance,
          detector,
          armed,
          flywheelReady,
          shotRunning,
          elapsedMs,
          ballTimesMs,
          resultFirstMs,
          resultLastMs,
          resultCount,
          resultWindowMs,
          intakeOn,
          status);

      boolean aPressed = gamepad1.a && !aPrev;
      boolean bPressed = gamepad1.b && !bPrev;
      aPrev = gamepad1.a;
      bPrev = gamepad1.b;

      if (aPressed && !shotRunning && resultWindowMs > 0) {
        trials.add(
            new TimingPoint(
                distance,
                resultWindowMs,
                resultFirstMs,
                resultLastMs,
                resultCount,
                System.currentTimeMillis()));
        saveProgress();
        break;
      }
      if (bPressed && !shotRunning) {
        break;
      }
    }

    // Wind down on the way out. Leaving via `return` skipped this entirely, so the flywheel stayed
    // at constant_rpm from the last endpoint through the whole completion screen — minutes of a
    // spun-up shooter on a robot nobody is watching, which is also what made a finished run look
    // like a hung one.
    stopEverything();
  }

  private FlywheelDipDetector newDetector() {
    var d = config.shooter.ball_detection;
    return new FlywheelDipDetector(
        dipFraction, d.rebound_fraction, d.baseline_alpha, d.refractory_ms);
  }

  private TraceRow row(
      int id, double distance, int elapsedMs, FlywheelDipDetector detector, boolean ballEvent) {
    var solution = robot.shotController.getLastSolution();
    return new TraceRow(
        id,
        distance,
        elapsedMs,
        robot.shooter.getShooterVelocity(),
        robot.shotController.getFlywheelTarget(),
        detector.getLastDipFraction(),
        detector.getBaselineVelocity(),
        ballEvent,
        robot.shotController.isFeedCommanded(),
        robot.shotController.isFlywheelReady(),
        robot.turret.isAimed(robot.follower.pose()),
        solution.isValid(),
        robot.shooter.getTargetHoodPosition(),
        solution.targetRpm());
  }

  private void render(
      double distance,
      FlywheelDipDetector detector,
      boolean armed,
      boolean flywheelReady,
      boolean shotRunning,
      int elapsedMs,
      List<Integer> ballTimesMs,
      int resultFirstMs,
      int resultLastMs,
      int resultCount,
      int resultWindowMs,
      boolean intakeOn,
      String status) {

    if (benchMode) telemetry.addData("MODE", "Fixed-speed bench timing; manually aimed");
    else telemetry.addData("DISTANCE", "%.0f in", distance);
    telemetry.addData("LOAD", "%d balls", expectedBalls);
    telemetry.addData(
        "Stop condition", "%d detected shots, then speed recovery; LT cancels", expectedBalls);

    String flywheel =
        !shotRunning
            ? "STOPPED — RT starts a new trial"
            : flywheelReady
                ? "READY"
                : String.format(
                    Locale.ROOT,
                    "SPINNING UP  %.0f%%",
                    robot.shotController.getFlywheelVelocityRatio() * 100);
    telemetry.addData("FLYWHEEL", flywheel);
    telemetry.addData("shooter motor", robot.shooter.getFirstMotorStatus());
    telemetry.addData("shooter2 motor", robot.shooter.getSecondMotorStatus());
    telemetry.addData("rightIntake motor", robot.intake.getMotorStatus());

    telemetry.addData(
        "SHOT",
        shotRunning ? String.format(Locale.ROOT, "FIRING %d ms", elapsedMs) : "idle — " + status);
    telemetry.addData("Balls this shot", "%d %s", ballTimesMs.size(), ballTimesMs);

    if (resultWindowMs > 0) {
      telemetry.addLine("--- result ---");
      telemetry.addData(
          "MEASURED RUN TIME", "%d ms (spin-up + shots + final recovery)", resultWindowMs);
      telemetry.addData("First ball", "%d ms", resultFirstMs);
      telemetry.addData("Last ball", "%d ms   of %d balls", resultLastMs, resultCount);
      telemetry.addLine("A to accept");
    }

    telemetry.addLine("--- detector ---");
    telemetry.addData(
        "Dip now / deepest",
        "%.1f%% / %.1f%%   (trigger %.1f%%)",
        detector.getLastDipFraction() * 100,
        detector.getDeepestDipFraction() * 100,
        dipFraction * 100);
    telemetry.addData("State", armed ? (detector.isInDip() ? "DIP" : "watching") : "not armed");
    telemetry.addData("Shot gate", robot.shotController.getGateReason());
    telemetry.addData("Flywheel gate", robot.shotController.getFlywheelGateDetail());
    telemetry.addData(
        "Intake",
        robot.shotController.isFeedCommanded() ? "FEEDING" : intakeOn ? "LOADING" : "off");

    telemetry.addLine("--------------------------------------------------");
    telemetry.addLine("RT  fire     LT  stop     X  intake");
    telemetry.addLine("A   accept   B   skip     DPad U/D  ball count");
    telemetry.addData(
        "Stick clicks: dip adjustment", "-/+ %.1f%%", config.shooter.timing_tuner.dip_step * 100);
    telemetry.update();
  }

  private void finish() {
    // Everything off before the completion screen: this loop only exits when the driver presses
    // STOP, and until then tick() keeps writing whatever the subsystems were last told to do.
    stopEverything();
    saveProgress();

    long idleStartMs = System.currentTimeMillis();
    while (opModeIsActive()) {
      tick();
      telemetry.addLine("Done — PRESS STOP");
      telemetry.addLine("========================================");
      for (TimingPoint p : trials) {
        telemetry.addLine(
            String.format(
                Locale.ROOT,
                "  %.0f in -> %d ms  (%d balls, first %d ms)",
                p.distanceInches(),
                p.shootWindowMs(),
                p.ballCount(),
                p.firstBallMs()));
      }
      telemetry.addLine("========================================");
      telemetry.addData("Trace rows", trace.size());
      telemetry.addLine(
          Environment.getExternalStorageDirectory() + "/FIRST/teamcode/" + resultFileName());
      telemetry.addLine(
          Environment.getExternalStorageDirectory() + "/FIRST/teamcode/shot_timing_log.csv");
      telemetry.addData(
          "Safe to stop", "idle %ds", (System.currentTimeMillis() - idleStartMs) / 1000);
      telemetry.update();
      sleep(50);
    }
  }

  /**
   * Flushes both files after every shot.
   *
   * <p>Everything here lives in memory, and an OpMode that never reaches its last line never saves.
   * A dead battery or a mis-press part way through would otherwise discard the whole run.
   */
  private void saveProgress() {
    List<TimingPoint> sorted = new ArrayList<>(trials);
    sorted.sort(Comparator.comparingDouble(TimingPoint::distanceInches));
    saveTable(sorted);
    saveTrace();
  }

  private void saveTable(List<TimingPoint> points) {
    File dir = new File(Environment.getExternalStorageDirectory(), "FIRST/teamcode/");
    if (!dir.exists() && !dir.mkdirs()) {
      telemetry.addData("Error", "Failed to create " + dir);
    }
    try (FileWriter writer = new FileWriter(new File(dir, resultFileName()), false)) {
      if (benchMode) {
        writer.write(
            "# Manually aimed fixed-speed bench timing; not a distance-calibrated AUTO table.\n");
        writer.write("target_ticks_per_second: " + config.shooter.constant_rpm + "\n");
        writer.write("trials:\n");
        for (TimingPoint p : points) {
          writer.write(
              String.format(
                  Locale.ROOT,
                  "  - {balls: %d, window_ms: %d, first_ball_ms: %d, last_ball_ms: %d}%n",
                  p.ballCount(),
                  p.shootWindowMs(),
                  p.firstBallMs(),
                  p.lastBallMs()));
        }
        return;
      }
      writer.write("# Generated by Shot Timing Tuner\n");
      writer.write(
          "# Paste under auto: in config.yaml; AUTO volleys look this up by distance.\n\n");
      writer.write("  # distance_in, shoot_window_ms\n");
      writer.write("  shot_time_points: [\n");
      for (int i = 0; i < points.size(); i++) {
        TimingPoint p = points.get(i);
        writer.write(
            String.format(
                Locale.ROOT,
                "        %.1f, %d%s%n",
                p.distanceInches(),
                p.shootWindowMs(),
                (i == points.size() - 1) ? "" : ","));
      }
      writer.write("    ]\n");
      writer.write("\n# --- accepted measurements ---\n");
      for (TimingPoint p : trials) {
        writer.write(
            String.format(
                Locale.ROOT,
                "# %.0f in, window %d ms, first %d ms, last %d ms, %d balls, t=%d%n",
                p.distanceInches(),
                p.shootWindowMs(),
                p.firstBallMs(),
                p.lastBallMs(),
                p.ballCount(),
                p.timestampMs()));
      }
    } catch (IOException e) {
      telemetry.addData("Error", "Failed to write shot time table: " + e.getMessage());
    }
  }

  private String resultFileName() {
    return benchMode ? "shot_timing_bench.yaml" : "shot_time_table.yaml";
  }

  private void saveTrace() {
    File dir = new File(Environment.getExternalStorageDirectory(), "FIRST/teamcode/");
    if (!dir.exists() && !dir.mkdirs()) {
      telemetry.addData("Error", "Failed to create " + dir);
    }
    try (FileWriter writer = new FileWriter(new File(dir, "shot_timing_log.csv"), false)) {
      writer.write(
          "shot,distance_in,t_ms,velocity,target_velocity,dip_pct,baseline,ball,feeding,"
              + "flywheel_ready,aimed,solver_valid,hood,solved_rpm\n");
      for (TraceRow r : trace) {
        writer.write(
            String.format(
                Locale.ROOT,
                "%d,%.0f,%d,%.1f,%.1f,%.2f,%.1f,%d,%d,%d,%d,%d,%.4f,%.0f%n",
                r.shotId(),
                r.distanceInches(),
                r.elapsedMs(),
                r.velocity(),
                r.targetVelocity(),
                r.dipFraction() * 100,
                r.baseline(),
                r.ballEvent() ? 1 : 0,
                r.feedCommanded() ? 1 : 0,
                r.flywheelReady() ? 1 : 0,
                r.aimed() ? 1 : 0,
                r.solverValid() ? 1 : 0,
                r.hoodTarget(),
                r.solvedRpm()));
      }
    } catch (IOException e) {
      telemetry.addData("Error", "Failed to write trace log: " + e.getMessage());
    }
  }
}
