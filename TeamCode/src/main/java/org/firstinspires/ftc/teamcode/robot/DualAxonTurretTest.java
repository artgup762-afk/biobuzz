package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/** Isolated positional-servo checkout. Feedback is raw voltage, not calibrated turret degrees. */
public final class DualAxonTurretTest {
  public enum Selection {
    NONE,
    LEFT,
    RIGHT,
    BOTH
  }

  private final AxonTurretServos servos;
  private final LongSupplier clock;
  private Selection selection = Selection.NONE;
  private boolean enableRequested;
  private boolean releaseRequired = true;
  private boolean active;
  private long enabledAt;
  private double offset;
  private double leftVoltage;
  private double rightVoltage;
  private String status = "PWM disabled";

  public DualAxonTurretTest(HardwareMap hardwareMap) {
    this(hardwareMap, System::nanoTime);
  }

  DualAxonTurretTest(HardwareMap hardwareMap, LongSupplier clock) {
    this.clock = clock;
    servos = new AxonTurretServos(hardwareMap);
  }

  public void select(Selection selected) {
    selection = selected;
    offset = 0;
    releaseRequired = true;
  }

  public void setEnableRequested(boolean enabled) {
    enableRequested = enabled;
  }

  public Command enableCommand() {
    return Command.build()
        .setStart(() -> setEnableRequested(true))
        .setEnd(interrupted -> setEnableRequested(false))
        .requiring(this);
  }

  public void stop() {
    enableRequested = false;
    selection = Selection.NONE;
    releaseRequired = true;
  }

  /** One step per button press, only while the previous loop actually enabled a servo. */
  public void jog(boolean increase) {
    if (!enableRequested || !active || releaseRequired) return;
    var c = config.turret_servo_test;
    offset =
        Range.clip(
            offset + (increase ? c.jog_step_position : -c.jog_step_position),
            -c.max_jog_offset,
            c.max_jog_offset);
  }

  /** Robot.TurretTestRig owns this loop; OpModes and commands only set targets. */
  void periodic() {
    try {
      leftVoltage = servos.leftVoltage();
      rightVoltage = servos.rightVoltage();
      String problem = configurationProblem();
      if (problem == null && (!Double.isFinite(leftVoltage) || !Double.isFinite(rightVoltage))) {
        problem = "Non-finite feedback: PWM disabled";
      }
      boolean wasActive = active;
      active = false;
      if (!enableRequested) {
        releaseRequired = false;
        status = problem == null ? "PWM disabled: hold LB to enable selected servo" : problem;
      } else if (problem != null) {
        releaseRequired = true;
        status = problem;
      } else if (selection == Selection.NONE || releaseRequired) {
        status = "Release LB, select A/B/X, then hold LB";
      } else {
        offset =
            Range.clip(
                offset,
                -config.turret_servo_test.max_jog_offset,
                config.turret_servo_test.max_jog_offset);
        if (!wasActive) enabledAt = clock.getAsLong();
        if ((clock.getAsLong() - enabledAt) / 1e9 >= config.turret_servo_test.max_enable_sec) {
          releaseRequired = true;
          status = "Enable timeout: release LB to re-arm";
        } else {
          active = true;
          status = "Enabled: dpad left/right jog; release LB or Y disables PWM";
        }
      }
      writeServos();
    } catch (RuntimeException e) {
      active = false;
      releaseRequired = true;
      writeServos();
      throw e;
    }
  }

  /** Sole actuator writer; disable the inactive side before enabling the selected one. */
  private void writeServos() {
    boolean useLeft = active && (selection == Selection.LEFT || selection == Selection.BOTH);
    boolean useRight = active && (selection == Selection.RIGHT || selection == Selection.BOTH);
    servos.write(useLeft, useRight, offset);
  }

  private double leftTarget() {
    var c = config.turret_servo_test;
    return c.left_center_position + (c.left_reversed ? -offset : offset);
  }

  private double rightTarget() {
    var c = config.turret_servo_test;
    return c.right_center_position + (c.right_reversed ? -offset : offset);
  }

  public static @Nullable String configurationProblem() {
    var c = config.turret_servo_test;
    if (c.left_servo_name.isBlank()
        || c.right_servo_name.isBlank()
        || c.left_servo_name.equals(c.right_servo_name)
        || c.left_analog_name.isBlank()
        || c.right_analog_name.isBlank()
        || c.left_analog_name.equals(c.right_analog_name)) {
      return "turret_servo_test: distinct nonempty servo and analog names required";
    }
    if (!(c.pwm_lower_us >= 500 && c.pwm_upper_us <= 2500 && c.pwm_lower_us < c.pwm_upper_us)) {
      return "turret_servo_test: invalid PWM range";
    }
    if (!(c.max_jog_offset > 0
        && c.max_jog_offset <= 0.5
        && c.jog_step_position > 0
        && c.jog_step_position <= c.max_jog_offset
        && c.left_center_position - c.max_jog_offset >= 0
        && c.left_center_position + c.max_jog_offset <= 1
        && c.right_center_position - c.max_jog_offset >= 0
        && c.right_center_position + c.max_jog_offset <= 1
        && c.max_enable_sec > 0
        && Double.isFinite(c.max_enable_sec))) {
      return "turret_servo_test: invalid center, jog range, step or enable timeout";
    }
    return null;
  }

  public void report(Telemetry telemetry) {
    var c = config.turret_servo_test;
    telemetry.addData("Selection", selection);
    telemetry.addData("Status", status);
    telemetry.addData("Left command / volts", "%.4f / %.4f", leftTarget(), leftVoltage);
    telemetry.addData("Right command / volts", "%.4f / %.4f", rightTarget(), rightVoltage);
    telemetry.addData("Pair verified for automatic aiming", c.pair_verified);
    telemetry.addData(
        "Left PWM", active && (selection == Selection.LEFT || selection == Selection.BOTH));
    telemetry.addData(
        "Right PWM", active && (selection == Selection.RIGHT || selection == Selection.BOTH));
    telemetry.addLine("X permits bounded paired commissioning; it does not certify calibration.");
    telemetry.addLine("A left / B right / X both (select with LB released)");
    telemetry.addLine("Hold LB: enable; dpad left/right: step; Y: disable both");
    telemetry.addLine(
        "First enable uses configured reference. Keep both gears engaged on this turret.");
    telemetry.addLine(
        "Feedback is shaft voltage; turret angle and 360-degree travel are NOT calibrated.");
  }
}
