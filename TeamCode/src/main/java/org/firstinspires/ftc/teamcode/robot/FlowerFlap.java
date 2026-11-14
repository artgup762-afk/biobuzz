package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/** TeleOp flower flap. Commands select targets; only Robot.update() calls the hardware writer. */
public final class FlowerFlap {
  public enum Target {
    NONE,
    UP,
    DOWN
  }

  private final @Nullable ServoImplEx servo;
  private Target target = Target.NONE;

  /** Non-TeleOp robots neither map nor write the flap hardware. */
  public FlowerFlap(HardwareMap hardwareMap, boolean forTeleOp) {
    servo =
        forTeleOp && config.flap.enabled
            ? hardwareMap.get(ServoImplEx.class, config.flap.servo_name)
            : null;
  }

  public void toggle() {
    target = target == Target.DOWN ? Target.UP : Target.DOWN;
  }

  public void stop() {
    target = Target.NONE;
  }

  public void setTarget(Target target) {
    this.target = target;
  }

  /** SDK command readback, not a measurement of the flap's physical angle. */
  public String outputStatus() {
    if (servo == null) return "Not mapped";
    var range = servo.getPwmRange();
    return String.format(
        Locale.ROOT,
        "SDK position %.4f; PWM enabled %s; range %.0f..%.0f us; %s; no angle sensor",
        servo.getPosition(),
        servo.isPwmEnabled(),
        range.usPulseLower,
        range.usPulseUpper,
        servo.getDirection());
  }

  public Target target() {
    return target;
  }

  public Command toggleCommand() {
    return Command.build().setStart(this::toggle).setDone(() -> true).requiring(this);
  }

  public static @Nullable String configurationProblem() {
    var c = config.flap;
    if (c.servo_name.isBlank()) return "flap.servo_name must name the configured positional servo";
    if (!(c.servo_range_deg > 0
        && Double.isFinite(c.servo_range_deg)
        && c.up_angle_deg >= 0
        && c.up_angle_deg <= c.servo_range_deg
        && c.down_angle_deg >= 0
        && c.down_angle_deg <= c.servo_range_deg
        && c.up_angle_deg != c.down_angle_deg)) {
      return "flap: distinct up/down angles must lie within servo_range_deg";
    }
    if (!(c.pwm_lower_us >= 500 && c.pwm_upper_us <= 2500 && c.pwm_lower_us < c.pwm_upper_us)) {
      return "flap: invalid positional servo PWM range";
    }
    return null;
  }

  public String status() {
    if (servo == null || !config.flap.enabled) return "Disabled";
    String problem = configurationProblem();
    if (problem != null) return problem;
    String state = target == Target.NONE ? "Unpowered; D-pad left lowers" : target + " target";
    return String.format(
        Locale.ROOT,
        "%s (UP %.0f / DOWN %.0f commanded deg)",
        state,
        config.flap.up_angle_deg,
        config.flap.down_angle_deg);
  }

  /** Sole actuator writer. No centering or movement until the operator selects a target. */
  void periodic() {
    if (servo == null) return;
    if (!config.flap.enabled || target == Target.NONE || configurationProblem() != null) {
      servo.setPwmDisable();
      return;
    }
    var c = config.flap;
    servo.setPwmRange(new PwmControl.PwmRange(c.pwm_lower_us, c.pwm_upper_us));
    servo.setDirection(c.reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    double angle = target == Target.DOWN ? c.down_angle_deg : c.up_angle_deg;
    servo.setPosition(angle / c.servo_range_deg);
    servo.setPwmEnable();
  }
}
