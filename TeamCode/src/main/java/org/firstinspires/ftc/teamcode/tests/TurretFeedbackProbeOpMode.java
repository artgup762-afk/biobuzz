package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.Nullable;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Finds the turret servos' analog feedback (Axon 4th wire) without any configuration entry: reads
 * all four analog inputs of every hub straight from its bulk data, next to the turret yaw from the
 * test Pinpoint ({@code turnpinpoint}), and logs every loop to {@code
 * /sdcard/FIRST/turret_probe.csv}.
 *
 * <ul>
 *   <li>With both servos limp (the default), turn the turret slowly by hand: the inputs that track
 *       the yaw are the feedback wires, with their volts per degree and direction.
 *   <li>[A] / [B]: nudge servo 1 / 2 alone (the other limp): NUDGE_CENTER, then +NUDGE, back,
 *       -NUDGE, back. The driven servo's shaft moves before the other takes up its gear slack, so
 *       its feedback leads; and its feedback shows whether the shaft goes to a fixed angle per
 *       command (position mode) or keeps turning (continuous). Limps both if the turret moves more
 *       than NUDGE_ABORT_DEG from where the nudge started.
 *   <li>[Y]: limp both.
 * </ul>
 */
@TeleOp(name = "Turret Feedback Probe", group = "Tuning")
@Configurable
public class TurretFeedbackProbeOpMode extends LinearOpMode {

  public static double NUDGE_CENTER = 0.5;
  public static double NUDGE = 0.002;
  public static double NUDGE_HOLD_SEC = 0.6;
  public static double NUDGE_ABORT_DEG = 10.0;

  private List<LynxModule> hubs;
  private ServoImplEx s1;
  private ServoImplEx s2;
  private GoBildaPinpointDriver imu;
  private @Nullable BufferedWriter log;
  private final ElapsedTime clock = new ElapsedTime();
  private double yaw;
  private String status = "both servos limp: turn the turret slowly by hand";
  private String phase = "hand";

  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry);
    hubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : hubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    s1 = hardwareMap.get(ServoImplEx.class, "turnservo1");
    s2 = hardwareMap.get(ServoImplEx.class, "turnservo2");
    imu = hardwareMap.get(GoBildaPinpointDriver.class, "turnpinpoint");
    limpBoth();
    imu.resetPosAndIMU();
    sleep(500);
    try {
      BufferedWriter out =
          new BufferedWriter(new FileWriter("/sdcard/FIRST/turret_probe.csv", false));
      log = out;
      StringBuilder h = new StringBuilder("t,phase,yaw,cmd1,pwm1,cmd2,pwm2");
      for (LynxModule hub : hubs) {
        for (int p = 0; p < 4; p++) {
          h.append(',').append(hubName(hub)).append("_a").append(p);
        }
      }
      out.write(h.append('\n').toString());
    } catch (IOException e) {
      telemetry.addLine("cannot write /sdcard/FIRST/turret_probe.csv: " + e.getMessage());
    }
    clock.reset();
    while (opModeInInit()) {
      step();
    }
    while (opModeIsActive()) {
      step();
      if (gamepad1.aWasPressed()) {
        nudge(s1, "servo1");
      } else if (gamepad1.bWasPressed()) {
        nudge(s2, "servo2");
      } else if (gamepad1.yWasPressed()) {
        limpBoth();
        status = "both servos limp";
      }
    }
    limpBoth();
    BufferedWriter out = log;
    if (out != null) {
      try {
        out.close();
      } catch (IOException ignored) {
        // Nothing more to do on stop.
      }
    }
  }

  private static String hubName(LynxModule hub) {
    return hub.isParent() ? "CH" : "EH" + hub.getModuleAddress();
  }

  private void limpBoth() {
    s1.setPwmDisable();
    s2.setPwmDisable();
  }

  /** One loop: read every hub's analog inputs and the yaw, log them, show them. */
  private void step() {
    for (LynxModule hub : hubs) {
      hub.clearBulkCache();
    }
    imu.update();
    yaw = imu.getHeading(UnnormalizedAngleUnit.DEGREES);
    StringBuilder row =
        new StringBuilder(
            String.format(
                Locale.ROOT,
                "%.4f,%s,%.3f,%.4f,%d,%.4f,%d",
                clock.seconds(),
                phase,
                yaw,
                s1.getPosition(),
                s1.isPwmEnabled() ? 1 : 0,
                s2.getPosition(),
                s2.isPwmEnabled() ? 1 : 0));
    telemetry.addData("Turret yaw", "%.2f deg", yaw);
    telemetry.addData("Status", status);
    for (LynxModule hub : hubs) {
      LynxModule.BulkData d = hub.getBulkData();
      StringBuilder line = new StringBuilder();
      for (int p = 0; p < 4; p++) {
        double v = d.getAnalogInputVoltage(p);
        row.append(',').append(String.format(Locale.ROOT, "%.4f", v));
        line.append(String.format(Locale.ROOT, "  a%d %.3f V", p, v));
      }
      telemetry.addData(hubName(hub), line.toString());
    }
    telemetry.addLine("hand-turn with both limp  [A]/[B] nudge servo 1/2  [Y] limp");
    telemetry.update();
    BufferedWriter out = log;
    if (out != null) {
      try {
        out.write(row.append('\n').toString());
      } catch (IOException ignored) {
        log = null;
      }
    }
  }

  /** Holds {@code s} at {@code pos} for {@code sec}; false (both limp) if the turret ran off. */
  private boolean holdAt(ServoImplEx s, double pos, double sec, double startYaw) {
    s.setPosition(pos);
    ElapsedTime t = new ElapsedTime();
    while (opModeIsActive() && t.seconds() < sec) {
      step();
      if (Math.abs(yaw - startYaw) > NUDGE_ABORT_DEG || gamepad1.yWasPressed()) {
        limpBoth();
        status =
            String.format(
                Locale.ROOT,
                "ABORTED: turret moved %.1f deg during the nudge at command %.4f",
                yaw - startYaw,
                pos);
        return false;
      }
    }
    return true;
  }

  private void nudge(ServoImplEx s, String label) {
    limpBoth();
    double startYaw = yaw;
    phase = label;
    status = label + ": nudging";
    s.setPwmEnable();
    double c = NUDGE_CENTER;
    boolean ok =
        holdAt(s, c, 1.0, startYaw)
            && holdAt(s, c + NUDGE, NUDGE_HOLD_SEC, startYaw)
            && holdAt(s, c, NUDGE_HOLD_SEC, startYaw)
            && holdAt(s, c - NUDGE, NUDGE_HOLD_SEC, startYaw)
            && holdAt(s, c, NUDGE_HOLD_SEC, startYaw);
    s.setPwmDisable();
    phase = "hand";
    if (ok) {
      status = label + " nudge done; both limp";
    }
  }
}
