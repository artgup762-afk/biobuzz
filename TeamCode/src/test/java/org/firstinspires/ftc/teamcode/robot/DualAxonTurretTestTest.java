package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import java.util.List;
import org.firstinspires.ftc.teamcode.robot.DualAxonTurretTest.Selection;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DualAxonTurretTestTest {
  private final ServoImplEx left = mock(ServoImplEx.class);
  private final ServoImplEx right = mock(ServoImplEx.class);
  private final AnalogInput leftAnalog = mock(AnalogInput.class);
  private final AnalogInput rightAnalog = mock(AnalogInput.class);
  private long now;

  @BeforeEach
  void useFixedFixtureCenters() {
    // Hardware calibration can change without changing these control-behavior examples.
    config.turret_servo_test.left_center_position = 0.5;
    config.turret_servo_test.right_center_position = 0.5;
    config.turret_servo_test.max_jog_offset = 0.02;
    config.turret_servo_test.jog_step_position = 0.002;
  }

  private HardwareMap hardware() {
    HardwareMap hw = mock(HardwareMap.class);
    var c = config.turret_servo_test;
    when(hw.get(ServoImplEx.class, c.left_servo_name)).thenReturn(left);
    when(hw.get(ServoImplEx.class, c.right_servo_name)).thenReturn(right);
    when(hw.get(AnalogInput.class, c.left_analog_name)).thenReturn(leftAnalog);
    when(hw.get(AnalogInput.class, c.right_analog_name)).thenReturn(rightAnalog);
    when(leftAnalog.getVoltage()).thenReturn(1.5);
    when(rightAnalog.getVoltage()).thenReturn(1.6);
    return hw;
  }

  private DualAxonTurretTest build() {
    return new DualAxonTurretTest(hardware(), () -> now);
  }

  @Test
  void rigRefreshesBothHubsBeforeEveryFeedbackReadAndDisablesOnShutdown() {
    var hw = hardware();
    LynxModule expansion = mock(LynxModule.class);
    LynxModule control = mock(LynxModule.class);
    when(hw.getAll(LynxModule.class)).thenReturn(List.of(expansion, control));
    var rig = new Robot.TurretTestRig(hw);
    var order = inOrder(expansion, control, leftAnalog, rightAnalog);
    for (int i = 0; i < 2; i++) {
      rig.update();
      order.verify(expansion).clearBulkCache();
      order.verify(control).clearBulkCache();
      order.verify(leftAnalog).getVoltage();
      order.verify(rightAnalog).getVoltage();
    }
    rig.turret.select(Selection.RIGHT);
    rig.update();
    rig.turret.setEnableRequested(true);
    rig.update();
    verify(right).setPosition(0.5);
    clearInvocations(left, right);
    rig.shutdown();
    rig.shutdown();
    rig.update();
    verify(left).setPwmDisable();
    verify(right).setPwmDisable();
    verify(right, never()).setPosition(anyDouble());
  }

  private void arm(DualAxonTurretTest turret, Selection selection) {
    turret.select(selection);
    turret.setEnableRequested(false);
    turret.periodic();
    turret.setEnableRequested(true);
    turret.periodic();
  }

  @Test
  void initDoesNotCommandACenterAndReadsBothFeedbacks() {
    var turret = build();
    verifyNoInteractions(left, right);
    turret.periodic();
    verify(left, never()).setPosition(anyDouble());
    verify(right, never()).setPosition(anyDouble());
    verify(left, atLeastOnce()).setPwmDisable();
    verify(right, atLeastOnce()).setPwmDisable();
    verify(leftAnalog).getVoltage();
    verify(rightAnalog).getVoltage();
  }

  @Test
  void eachServoCanBeTestedWithoutPoweringTheOther() {
    for (var selection : new Selection[] {Selection.LEFT, Selection.RIGHT}) {
      reset(left, right);
      var turret = build();
      arm(turret, selection);
      ServoImplEx selected = selection == Selection.LEFT ? left : right;
      ServoImplEx other = selection == Selection.LEFT ? right : left;
      verify(selected).setPosition(0.5);
      verify(other, never()).setPosition(anyDouble());
      clearInvocations(left, right);
      turret.setEnableRequested(false);
      turret.periodic();
      verify(left).setPwmDisable();
      verify(right).setPwmDisable();
      verify(selected, never()).setPosition(anyDouble());
    }
  }

  @Test
  void bothCanBeCommissionedBeforeAutomaticPairVerification() {
    config.turret_servo_test.pair_verified = false;
    var turret = build();
    arm(turret, Selection.BOTH);
    verify(left).setPosition(0.5);
    verify(right).setPosition(0.5);
    clearInvocations(left, right);
    turret.jog(true);
    turret.periodic();
    verify(left).setPosition(0.502);
    verify(right).setPosition(0.502);
    assertFalse(config.turret_servo_test.pair_verified);
    turret.setEnableRequested(false);
    clearInvocations(left, right);
    turret.periodic();
    verify(left).setPwmDisable();
    verify(right).setPwmDisable();
  }

  @Test
  void timeoutLatchesUntilReleaseAndEmergencyStopDisablesBoth() {
    var turret = build();
    arm(turret, Selection.LEFT);
    clearInvocations(left, right);
    now = (long) (config.turret_servo_test.max_enable_sec * 1e9);
    turret.periodic();
    now += 1_000_000_000L;
    turret.periodic();
    verify(left, never()).setPosition(anyDouble());
    arm(turret, Selection.LEFT);
    verify(left).setPosition(0.5);
    clearInvocations(left, right);
    turret.stop();
    turret.periodic();
    verify(left).setPwmDisable();
    verify(right).setPwmDisable();
    verify(left, never()).setPosition(anyDouble());
  }

  @Test
  void changingSelectionWhileEnabledRequiresRelease() {
    var turret = build();
    arm(turret, Selection.LEFT);
    clearInvocations(left, right);
    turret.select(Selection.RIGHT);
    turret.periodic();
    verify(left).setPwmDisable();
    verify(right).setPwmDisable();
    verify(right, never()).setPosition(anyDouble());
  }

  @Test
  void jogLimitsAndIndependentReversalsKeepPairWithinTheCommandWindow() {
    var c = config.turret_servo_test;
    c.pair_verified = true;
    c.left_center_position = 0.4;
    c.right_center_position = 0.6;
    c.right_reversed = true;
    var turret = build();
    arm(turret, Selection.BOTH);
    for (int i = 0; i < 50; i++) turret.jog(true);
    clearInvocations(left, right);
    turret.periodic();
    verify(left).setPosition(doubleThat(p -> Math.abs(p - 0.42) < 1e-12));
    verify(right).setPosition(doubleThat(p -> Math.abs(p - 0.58) < 1e-12));
    for (int i = 0; i < 100; i++) turret.jog(false);
    clearInvocations(left, right);
    turret.periodic();
    verify(left).setPosition(doubleThat(p -> Math.abs(p - 0.38) < 1e-12));
    verify(right).setPosition(doubleThat(p -> Math.abs(p - 0.62) < 1e-12));
    c.left_reversed = true;
    c.right_reversed = false;
    clearInvocations(left, right);
    turret.periodic();
    verify(left).setPosition(doubleThat(p -> Math.abs(p - 0.42) < 1e-12));
    verify(right).setPosition(doubleThat(p -> Math.abs(p - 0.58) < 1e-12));
  }

  @Test
  void badFeedbackAndReadExceptionsDisableBoth() {
    var turret = build();
    arm(turret, Selection.LEFT);
    when(leftAnalog.getVoltage()).thenReturn(Double.NaN);
    clearInvocations(left, right);
    turret.periodic();
    verify(left).setPwmDisable();
    verify(left, never()).setPosition(anyDouble());
    when(leftAnalog.getVoltage()).thenReturn(1.5);
    arm(turret, Selection.RIGHT);
    when(rightAnalog.getVoltage()).thenThrow(new IllegalStateException("disconnected"));
    clearInvocations(left, right);
    assertThrows(IllegalStateException.class, turret::periodic);
    verify(left).setPwmDisable();
    verify(right).setPwmDisable();
  }

  @Test
  void invalidConfigurationNeverEnablesAServo() {
    var c = config.turret_servo_test;
    assertNull(DualAxonTurretTest.configurationProblem());
    c.left_center_position = Double.NaN;
    var turret = build();
    arm(turret, Selection.LEFT);
    verify(left, never()).setPosition(anyDouble());
    assertNotNull(DualAxonTurretTest.configurationProblem());
    c.left_center_position = 0.99;
    assertNotNull(DualAxonTurretTest.configurationProblem());
    c.left_center_position = 0.5;
    c.right_center_position = 0;
    assertNotNull(DualAxonTurretTest.configurationProblem());
    c.right_center_position = 0.5;
    c.pwm_lower_us = c.pwm_upper_us;
    assertNotNull(DualAxonTurretTest.configurationProblem());
    c.pwm_lower_us = 500;
    c.jog_step_position = c.max_jog_offset * 2;
    assertNotNull(DualAxonTurretTest.configurationProblem());
    c.jog_step_position = 0.002;
    c.max_enable_sec = Double.POSITIVE_INFINITY;
    assertNotNull(DualAxonTurretTest.configurationProblem());
    c.max_enable_sec = 3;
    c.right_servo_name = c.left_servo_name;
    assertNotNull(DualAxonTurretTest.configurationProblem());
    c.right_servo_name = "right";
    c.right_analog_name = c.left_analog_name;
    assertNotNull(DualAxonTurretTest.configurationProblem());
  }
}
