package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DualPositionTurretTest {
  private final HardwareMap hw = mock(HardwareMap.class);
  private final ServoImplEx left = mock(ServoImplEx.class);
  private final ServoImplEx right = mock(ServoImplEx.class);
  private final AnalogInput leftAnalog = mock(AnalogInput.class);
  private final AnalogInput rightAnalog = mock(AnalogInput.class);
  private final Pose pose = new Pose(0, 0, 0);
  private long now;

  @BeforeEach
  void useFixedControlFixtureWindow() {
    // These timeout and clamp examples use a fixed window independent of hardware commissioning.
    config.turret_servo_test.max_jog_offset = 0.02;
    config.turret_servo_test.jog_step_position = 0.002;
  }

  private Turret build() {
    config.turret.dual_position.enabled = true;
    var c = config.turret_servo_test;
    when(hw.get(ServoImplEx.class, c.left_servo_name)).thenReturn(left);
    when(hw.get(ServoImplEx.class, c.right_servo_name)).thenReturn(right);
    when(hw.get(AnalogInput.class, c.left_analog_name)).thenReturn(leftAnalog);
    when(hw.get(AnalogInput.class, c.right_analog_name)).thenReturn(rightAnalog);
    feedback(0, 0);
    return new Turret(hw, null, () -> pose, Velocity::zero, () -> now);
  }

  private void calibrate() {
    config.turret.dual_position.angle_calibrated = true;
    config.turret_servo_test.pair_verified = true;
  }

  private void feedback(double leftOffset, double rightOffset) {
    var c = config.turret.dual_position;
    when(leftAnalog.getVoltage())
        .thenReturn(c.left_reference_volts + leftOffset * c.left_volts_per_position);
    when(rightAnalog.getVoltage())
        .thenReturn(c.right_reference_volts + rightOffset * c.right_volts_per_position);
  }

  @Test
  void mainSubsystemMapsBothConfiguredServosWithoutLegacyDevicesOrInitMovement() {
    var t = build();
    verify(hw).get(ServoImplEx.class, "turnservo1");
    verify(hw).get(ServoImplEx.class, "turnservo2");
    verify(hw, never()).get(eq(CRServo.class), anyString());
    verify(hw, never()).get(AnalogInput.class, "turnanalog");
    verifyNoInteractions(left, right);
    t.periodic();
    verify(left, never()).setPosition(anyDouble());
    verify(right, never()).setPosition(anyDouble());
    assertTrue(Double.isNaN(t.getCurrentTurnAngle()));
  }

  @Test
  void automaticAimRequiresBothMechanicalAndAngleVerification() {
    var t = build();
    t.setAimMode(Turret.AimMode.HOLD);
    t.periodic();
    assertFalse(t.isAimed(pose));
    assertFalse(t.isTurnDone());
    assertTrue(t.getPowerBlockedReason().contains("locked"));
    assertNotNull(t.getEncoderRangeWarning());
    config.turret.dual_position.angle_calibrated = true;
    t.periodic();
    verify(left, never()).setPosition(anyDouble());
    calibrate();
    t.periodic();
    assertTrue(t.isAimed(pose));
    assertNull(t.getEncoderRangeWarning());
    assertEquals(0, t.getCurrentTurnAngle(), 1e-9);
  }

  @Test
  void gearRatioAndReversalsMapTargetAndBothFeedbacksConsistently() {
    var t = build();
    calibrate();
    // One quarter shaft turn is 90 * 64/150 = 38.4 turret degrees.
    assertEquals(153.6, DualPositionTurret.degreesPerPosition(), 1e-9);
    for (boolean ccw : new boolean[] {true, false}) {
      config.turret.dual_position.positive_offset_ccw = ccw;
      for (boolean reverse : new boolean[] {true, false}) {
        config.turret_servo_test.right_reversed = reverse;
        config.turret_servo_test.left_reversed = !reverse;
        double commandOffset = ccw ? 0.01 : -0.01;
        double leftOffset = reverse ? commandOffset : -commandOffset;
        double rightOffset = reverse ? -commandOffset : commandOffset;
        feedback(leftOffset, rightOffset);
        t.setHoldAngle(1.536);
        t.setAimMode(Turret.AimMode.HOLD);
        clearInvocations(left, right);
        t.periodic();
        verify(left).setPosition(config.turret_servo_test.left_center_position + leftOffset);
        verify(right).setPosition(config.turret_servo_test.right_center_position + rightOffset);
        assertEquals(1.536, t.getCurrentTurnAngle(), 1e-9);
        assertTrue(t.isAimed(pose));
      }
    }
  }

  @Test
  void oppositeEncoderErrorsDoNotCancelIntoAFakeAimClaim() {
    var t = build();
    calibrate();
    config.turret.tolerance.min_deg = 0.5;
    config.turret.tolerance.max_deg = 0.5;
    t.setAimMode(Turret.AimMode.HOLD);
    feedback(0.01, -0.01);
    t.periodic();
    assertEquals(0, t.getCurrentTurnAngle(), 1e-9);
    assertFalse(t.isAimed(pose));
  }

  @Test
  void outsideVerifiedWindowClampsCommandsButNeverClaimsAim() {
    var t = build();
    calibrate();
    t.setHoldAngle(20);
    t.setAimMode(Turret.AimMode.HOLD);
    t.periodic();
    verify(left)
        .setPosition(
            config.turret_servo_test.left_center_position
                + config.turret_servo_test.max_jog_offset);
    assertTrue(t.isTargetOutsideTravel());
    assertFalse(t.isAimed(pose));
    assertArrayEquals(new double[] {-3.072, 3.072}, t.getReachableAngleRange(), 1e-9);
  }

  @Test
  void manualCommandsMoveBothDirectionsOnlyFromPeriodicAndRespectBounds() {
    var t = build();
    t.setAimMode(Turret.AimMode.MANUAL);
    t.setManualPower(1);
    verifyNoInteractions(left, right);
    t.periodic();
    for (int i = 0; i < 20; i++) {
      now += 100_000_000;
      t.periodic();
    }
    verify(left, atLeastOnce()).setPosition(config.turret_servo_test.left_center_position + 0.02);
    t.setManualPower(0);
    t.periodic();
    t.setManualPower(-1);
    t.periodic();
    for (int i = 0; i < 25; i++) {
      now += 100_000_000;
      t.periodic();
    }
    verify(right, atLeastOnce()).setPosition(config.turret_servo_test.right_center_position - 0.02);
    assertFalse(t.isAimed(pose));
    clearInvocations(left, right);
    t.setEnabled(false);
    t.periodic();
    verify(left).setPwmDisable();
    verify(right).setPwmDisable();
  }

  @Test
  void manualTimeoutOrLoopGapRequiresNeutralToRearm() {
    var t = build();
    t.setAimMode(Turret.AimMode.MANUAL);
    t.setManualPower(1);
    t.periodic();
    now += 4_000_000_000L;
    t.periodic();
    clearInvocations(left, right);
    now += 20_000_000L;
    t.periodic();
    verify(left, never()).setPosition(anyDouble());
    t.setManualPower(0);
    t.periodic();
    t.setManualPower(-1);
    t.periodic();
    verify(left).setPosition(anyDouble());
  }

  @Test
  void zeroVoltageMayWakeButMissingOrInvalidFeedbackCannotAimAndLatchesOff() {
    var t = build();
    calibrate();
    when(leftAnalog.getVoltage()).thenReturn(0.0);
    when(rightAnalog.getVoltage()).thenReturn(0.0);
    t.setAimMode(Turret.AimMode.HOLD);
    t.periodic();
    assertFalse(t.isFaulted());
    assertFalse(t.isAimed(pose));
    assertTrue(Double.isNaN(t.getCurrentTurnAngle()));
    now = 600_000_000;
    t.periodic();
    assertTrue(t.isFaulted());
    feedback(0, 0);
    clearInvocations(left, right);
    t.periodic();
    verify(left, never()).setPosition(anyDouble());
    t.clearFault();
    t.periodic();
    assertTrue(t.isAimed(pose));
    when(rightAnalog.getVoltage()).thenReturn(Double.NaN);
    t.periodic();
    assertFalse(t.isAimed(pose));
    assertTrue(t.isFaulted());
  }

  @Test
  void hardwareExceptionsDisableBothOutputs() {
    var t = build();
    t.setAimMode(Turret.AimMode.MANUAL);
    t.setManualPower(1);
    t.periodic();
    when(leftAnalog.getVoltage()).thenThrow(new IllegalStateException("lost sensor"));
    clearInvocations(left, right);
    assertThrows(IllegalStateException.class, t::periodic);
    verify(left).setPwmDisable();
    verify(right).setPwmDisable();
  }

  @Test
  void invalidConfigAndManualRequestsNeverProducePositionWrites() {
    var t = build();
    t.setAimMode(Turret.AimMode.MANUAL);
    t.setManualPower(Double.NaN);
    t.periodic();
    verify(left, never()).setPosition(anyDouble());
    config.turret.dual_position.turret_gear_teeth = 0;
    assertNotNull(Turret.positionalConfigurationProblem());
    t.setManualPower(1);
    t.periodic();
    verify(right, never()).setPosition(anyDouble());
  }
}
