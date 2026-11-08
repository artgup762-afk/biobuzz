package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.EnumMap;
import java.util.Locale;
import java.util.Map;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.HiveVision;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Which alliance's CELL is up on the side of the HIVE the camera faces.
 *
 * <p>The CELL tags can only be seen from the side where that alliance's CELL is up: from the side
 * where BLUE is up only BLUE tags are visible (the upward CELL's, and sometimes the downward
 * one's), and from the side where RED is up only RED tags. So the colour of the tags in view is the
 * answer, and no height is needed for it. Height ({@code hive.up_height_threshold_in}) only says
 * which of that colour's two CELLS is the upper one when both are seen.
 *
 * <p>A colour counts as seen for {@code hive.max_observation_age_ms} after its last sighting, so a
 * dropped frame does not blank the answer. Moves nothing: it works the same in INIT and after
 * START.
 */
@TeleOp(name = "Hive Up Detector", group = "Test")
public class HiveUpDetectorOpMode extends OpMode {
  private HiveVision vision;
  private final Map<Alliance, Long> lastSeenMs = new EnumMap<>(Alliance.class);
  private final Map<Alliance, Map<HiveCell, HiveVision.CellSighting>> latest =
      new EnumMap<>(Alliance.class);
  private final Map<Alliance, Map<HiveCell, Long>> cellSeenMs = new EnumMap<>(Alliance.class);

  @Override
  public void init() {
    config.reload();
    OpModeUtil.reportConfig(telemetry);
    vision = new HiveVision();
    vision.init(hardwareMap);
  }

  @Override
  public void init_loop() {
    update();
  }

  @Override
  public void loop() {
    update();
  }

  private void update() {
    if (!vision.isAvailable()) {
      telemetry.addLine(
          String.format(
              Locale.ROOT,
              "NO CAMERA: vision.enabled=%b, no webcam named '%s'",
              config.vision.enabled,
              config.vision.camera_name));
      telemetry.update();
      return;
    }
    long now = System.currentTimeMillis();
    for (HiveVision.CellSighting s : vision.poll()) {
      lastSeenMs.put(s.owner(), now);
      latest.computeIfAbsent(s.owner(), a -> new EnumMap<>(HiveCell.class)).put(s.cell(), s);
      cellSeenMs.computeIfAbsent(s.owner(), a -> new EnumMap<>(HiveCell.class)).put(s.cell(), now);
    }
    long maxAge = config.hive.max_observation_age_ms;
    boolean red = recent(Alliance.RED, now, maxAge);
    boolean blue = recent(Alliance.BLUE, now, maxAge);

    String answer;
    if (red && blue) {
      answer = "BOTH COLOURS VISIBLE: check the camera view";
    } else if (blue) {
      answer = "BLUE IS UP";
    } else if (red) {
      answer = "RED IS UP";
    } else {
      answer = "NO HIVE TAGS IN VIEW";
    }
    telemetry.addLine("==========================");
    telemetry.addLine("   " + answer);
    telemetry.addLine("==========================");

    double threshold = config.hive.up_height_threshold_in;
    for (Alliance a : Alliance.values()) {
      if (!recent(a, now, maxAge)) {
        continue;
      }
      for (Map.Entry<HiveCell, HiveVision.CellSighting> e :
          latest.getOrDefault(a, Map.of()).entrySet()) {
        HiveVision.CellSighting s = e.getValue();
        Long seen = cellSeenMs.getOrDefault(a, Map.of()).get(e.getKey());
        long ageMs = seen == null ? Long.MAX_VALUE : now - seen;
        if (ageMs > maxAge) {
          continue;
        }
        telemetry.addLine(
            String.format(
                Locale.ROOT,
                "%s CELL %s: %s (%.1f in high, %.0f in away, %d%% of tags)",
                a,
                e.getKey(),
                HiveVision.isUpward(s, threshold) ? "upper CELL" : "lower CELL",
                s.heightIn(),
                s.rangeIn(),
                s.percentFound()));
        if (ageMs > 0) {
          telemetry.addData("  last seen", "%d ms ago", ageMs);
        }
      }
    }
    telemetry.update();
  }

  private boolean recent(Alliance a, long now, long maxAgeMs) {
    Long t = lastSeenMs.get(a);
    return t != null && now - t <= maxAgeMs;
  }

  @Override
  public void stop() {
    if (vision != null) {
      vision.close();
    }
  }
}
