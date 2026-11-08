package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.HiveVision;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/**
 * Checks HIVE CELL detection without the rest of the robot: opens the webcam through {@link
 * HiveVision} and shows, for every CELL cluster in view, its owner, the robot-frame position and
 * height of the opening, and whether that height classifies it upward ({@code
 * hive.up_height_threshold_in}). The raw camera-frame poses are shown too, so the camera mount
 * ({@code vision.camera}) can be checked against a CELL whose state is known. Moves nothing, so it
 * works the same in INIT and after START.
 */
@TeleOp(name = "Hive Vision Test", group = "Test")
public class HiveVisionTestOpMode extends OpMode {
  private HiveVision vision;

  @Override
  public void init() {
    config.reload();
    OpModeUtil.reportConfig(telemetry);
    vision = new HiveVision();
    vision.init(hardwareMap);
  }

  @Override
  public void init_loop() {
    show();
  }

  @Override
  public void loop() {
    show();
  }

  private void show() {
    if (!vision.isAvailable()) {
      telemetry.addLine(
          String.format(
              Locale.ROOT,
              "NO CAMERA: vision.enabled=%b, no webcam named '%s' in the hardware map",
              config.vision.enabled,
              config.vision.camera_name));
      telemetry.update();
      return;
    }
    double threshold = config.hive.up_height_threshold_in;
    var mount = vision.mount();
    telemetry.addLine(
        String.format(
            Locale.ROOT,
            "mount fwd %.2f left %.2f h %.2f yaw %.1f pitch %.1f | up >= %.1f in",
            mount.forwardIn(),
            mount.leftIn(),
            mount.heightIn(),
            Math.toDegrees(mount.yawRad()),
            Math.toDegrees(mount.pitchUpRad()),
            threshold));
    var sightings = vision.poll();
    telemetry.addData("CELLS seen", sightings.size());
    for (HiveVision.CellSighting s : sightings) {
      telemetry.addLine(
          String.format(
              Locale.ROOT,
              "CELL %s %s: %s  height %.1f in  fwd %.1f left %.1f range %.1f  found %d%%",
              s.owner(),
              s.cell(),
              HiveVision.isUpward(s, threshold) ? "UP" : "DOWN",
              s.heightIn(),
              s.forwardIn(),
              s.leftIn(),
              s.rangeIn(),
              s.percentFound()));
    }
    var raw = vision.rawDetections();
    telemetry.addData("raw detections", raw.size());
    for (AprilTagDetection d : raw) {
      String kind;
      if (d instanceof AprilTagClusterDetection c) {
        kind =
            String.format(
                Locale.ROOT,
                "cluster '%s' %d%%",
                c.metadata != null ? c.metadata.name : "?",
                c.percentClusterFound);
      } else if (d instanceof AprilTagSingleDetection t) {
        kind =
            String.format(
                Locale.ROOT, "tag %d '%s'", t.id, t.metadata != null ? t.metadata.name : "?");
      } else {
        kind = d.getClass().getSimpleName();
      }
      if (d.ftcPose != null) {
        telemetry.addLine(
            String.format(
                Locale.ROOT,
                "raw %s: cam x %.2f y %.2f z %.2f  range %.1f elev %.1f deg",
                kind,
                d.ftcPose.x,
                d.ftcPose.y,
                d.ftcPose.z,
                d.ftcPose.range,
                Math.toDegrees(d.ftcPose.elevation)));
      } else {
        telemetry.addLine("raw " + kind + ": no pose");
      }
    }
    telemetry.update();
  }

  @Override
  public void stop() {
    if (vision != null) {
      vision.close();
    }
  }
}
