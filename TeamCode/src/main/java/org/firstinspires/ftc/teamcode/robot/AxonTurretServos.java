package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/** Shared hardware adapter; constructed by exactly one turret controller per OpMode. */
final class AxonTurretServos {
  private final ServoImplEx left;
  private final ServoImplEx right;
  private final AnalogInput leftAnalog;
  private final AnalogInput rightAnalog;
  private boolean configured;

  AxonTurretServos(HardwareMap hw) {
    var c = config.turret_servo_test;
    left = hw.get(ServoImplEx.class, c.left_servo_name);
    right = hw.get(ServoImplEx.class, c.right_servo_name);
    leftAnalog = hw.get(AnalogInput.class, c.left_analog_name);
    rightAnalog = hw.get(AnalogInput.class, c.right_analog_name);
  }

  double leftVoltage() {
    return leftAnalog.getVoltage();
  }

  double rightVoltage() {
    return rightAnalog.getVoltage();
  }

  /** Sole hardware writer, called only by the owning controller's periodic update. */
  void write(boolean useLeft, boolean useRight, double offset) {
    if (!Double.isFinite(offset) || DualAxonTurretTest.configurationProblem() != null) {
      left.setPwmDisable();
      right.setPwmDisable();
      return;
    }
    try {
      var c = config.turret_servo_test;
      if (!configured) {
        left.setPwmDisable();
        right.setPwmDisable();
        var range = new PwmControl.PwmRange(c.pwm_lower_us, c.pwm_upper_us);
        left.setPwmRange(range);
        right.setPwmRange(range);
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
        configured = true;
      }
      offset = com.qualcomm.robotcore.util.Range.clip(offset, -c.max_jog_offset, c.max_jog_offset);
      if (!useLeft) left.setPwmDisable();
      if (!useRight) right.setPwmDisable();
      if (useLeft) {
        left.setPosition(c.left_center_position + (c.left_reversed ? -offset : offset));
        left.setPwmEnable();
      }
      if (useRight) {
        right.setPosition(c.right_center_position + (c.right_reversed ? -offset : offset));
        right.setPwmEnable();
      }
    } catch (RuntimeException e) {
      try {
        left.setPwmDisable();
      } finally {
        right.setPwmDisable();
      }
      throw e;
    }
  }
}
