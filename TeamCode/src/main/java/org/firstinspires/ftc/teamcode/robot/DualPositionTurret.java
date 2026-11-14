package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/** Positional Axons close their own shaft loops. Never send the legacy CR power law to them. */
final class DualPositionTurret {
  private final AxonTurretServos servos;
  private final LongSupplier clock;
  private double offset;
  private long lastUpdate = Long.MIN_VALUE;
  private long enabledAt;
  private boolean active;
  private boolean manualReleaseRequired;
  private @Nullable String fault;
  private @Nullable String blocked = "PWM disabled";

  DualPositionTurret(HardwareMap hw, LongSupplier clock) {
    servos = new AxonTurretServos(hw);
    this.clock = clock;
  }

  static double degreesPerPosition() {
    var c = config.turret.dual_position;
    return c.servo_sweep_deg
        * c.servo_gear_teeth
        / c.turret_gear_teeth
        * (c.positive_offset_ccw ? 1 : -1);
  }

  static @Nullable String configurationProblem() {
    var c = config.turret.dual_position;
    if (!(c.servo_gear_teeth > 0
        && c.turret_gear_teeth > 0
        && c.servo_sweep_deg > 0
        && c.servo_sweep_deg <= 360
        && c.left_volts_per_position > 0
        && Double.isFinite(c.left_volts_per_position)
        && c.right_volts_per_position > 0
        && Double.isFinite(c.right_volts_per_position)
        && c.left_reference_volts > 0
        && c.left_reference_volts < 3.3
        && c.right_reference_volts > 0
        && c.right_reference_volts < 3.3
        && c.manual_position_per_sec > 0
        && c.manual_position_per_sec <= 1
        && c.feedback_startup_sec >= 0
        && c.feedback_startup_sec <= 2
        && c.max_update_gap_sec > 0
        && c.max_update_gap_sec <= 0.5)) {
      return "Invalid dual positional turret calibration or timing";
    }
    return DualAxonTurretTest.configurationProblem();
  }

  static boolean calibrated() {
    return config.turret.dual_position.angle_calibrated
        && config.turret_servo_test.pair_verified
        && configurationProblem() == null;
  }

  double rawVoltage() {
    return servos.leftVoltage();
  }

  @Nullable
  String blockedReason() {
    return blocked;
  }

  @Nullable
  String faultReason() {
    return fault;
  }

  void clearFault() {
    fault = null;
    manualReleaseRequired = true;
  }

  private static boolean validVoltage(double volts) {
    return volts > 0 && volts < 3.3;
  }

  private double angle(boolean left, double volts) {
    var c = config.turret.dual_position;
    var s = config.turret_servo_test;
    double reference = left ? c.left_reference_volts : c.right_reference_volts;
    double slope = left ? c.left_volts_per_position : c.right_volts_per_position;
    boolean reversed = left ? s.left_reversed : s.right_reversed;
    return (volts - reference) / slope * (reversed ? -1 : 1) * degreesPerPosition();
  }

  double currentAngle() {
    if (!active || !calibrated()) return Double.NaN;
    double left = servos.leftVoltage();
    double right = servos.rightVoltage();
    if (!validVoltage(left) || !validVoltage(right)) return Double.NaN;
    return (angle(true, left) + angle(false, right)) / 2;
  }

  boolean reachable(double degrees) {
    return calibrated()
        && Double.isFinite(degrees)
        && Math.abs(degrees / degreesPerPosition()) <= config.turret_servo_test.max_jog_offset
        && degrees >= config.turret.travel.min_angle
        && degrees <= config.turret.travel.max_angle;
  }

  boolean aimed(double desired, double tolerance) {
    if (!active || fault != null || !reachable(desired)) return false;
    double left = servos.leftVoltage();
    double right = servos.rightVoltage();
    // Both shafts must agree; averaging opposite errors must never open the feed gate.
    return validVoltage(left)
        && validVoltage(right)
        && Math.abs(desired - angle(true, left)) < tolerance
        && Math.abs(desired - angle(false, right)) < tolerance;
  }

  void periodic(boolean enabled, Turret.AimMode mode, double desired, double manualPower) {
    boolean use = false;
    long now = clock.getAsLong();
    double dt = lastUpdate == Long.MIN_VALUE ? 0 : (now - lastUpdate) / 1e9;
    lastUpdate = now;
    try {
      var c = config.turret.dual_position;
      var s = config.turret_servo_test;
      String problem = configurationProblem();
      double left = servos.leftVoltage();
      double right = servos.rightVoltage();
      if (!Double.isFinite(left) || !Double.isFinite(right)) fault = "Invalid Axon feedback";
      if (active
          && (now - enabledAt) / 1e9 >= c.feedback_startup_sec
          && (!validVoltage(left) || !validVoltage(right)))
        fault = "Axon feedback missing after PWM enable";
      if (mode != Turret.AimMode.MANUAL || manualPower == 0 || !enabled)
        manualReleaseRequired = false;
      if (!enabled || mode == Turret.AimMode.IDLE) {
        blocked = "PWM disabled";
      } else if (problem != null || fault != null) {
        blocked = fault != null ? fault : problem;
      } else if (mode == Turret.AimMode.MANUAL) {
        if (!Double.isFinite(manualPower)) {
          manualReleaseRequired = true;
          blocked = "Invalid manual request; return input to neutral";
        } else if (manualPower == 0) {
          blocked = "Manual input neutral: PWM disabled";
        } else if (manualReleaseRequired) {
          blocked = "Return manual input to neutral to re-arm";
        } else if (active
            && (dt < 0
                || dt > c.max_update_gap_sec
                || (now - enabledAt) / 1e9 >= s.max_enable_sec)) {
          manualReleaseRequired = true;
          blocked = "Manual timeout: return input to neutral";
        } else {
          if (!active) {
            enabledAt = now;
            dt = 0;
          }
          offset =
              Range.clip(
                  offset + Range.clip(manualPower, -1, 1) * c.manual_position_per_sec * dt,
                  -s.max_jog_offset,
                  s.max_jog_offset);
          use = true;
          blocked = null;
        }
      } else if (!calibrated()) {
        blocked = "Automatic aim locked: verify pair, forward zero, sweep and CCW direction";
      } else if (!Double.isFinite(desired)) {
        blocked = "Invalid aim input";
      } else {
        if (!active) enabledAt = now;
        double target =
            Range.clip(desired, config.turret.travel.min_angle, config.turret.travel.max_angle);
        offset = Range.clip(target / degreesPerPosition(), -s.max_jog_offset, s.max_jog_offset);
        use = true;
        blocked = reachable(desired) ? null : "Target beyond verified positional travel";
      }
      active = use;
      servos.write(use, use, offset);
    } catch (RuntimeException e) {
      active = false;
      fault = "Axon hardware read/write failure";
      servos.write(false, false, 0);
      throw e;
    }
  }
}
