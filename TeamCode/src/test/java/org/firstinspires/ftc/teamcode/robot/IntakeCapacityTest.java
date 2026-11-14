package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;

public class IntakeCapacityTest {
  @Test
  public void knownFullPreloadCannotBecomeUnknownJustByRequestingCollection() {
    HardwareMap map = mock(HardwareMap.class);
    DcMotorEx motor = mock(DcMotorEx.class);
    config.inventory.entry_sensor_enabled = false;
    when(map.get(DcMotorEx.class, "rightIntake")).thenReturn(motor);
    Intake intake = new Intake(map);
    verify(map).get(DcMotorEx.class, "rightIntake");
    verifyNoMoreInteractions(map);
    intake.inventory().set(config.inventory.capacity_count);
    intake.run(1.0);
    verify(motor, never()).setPower(anyDouble());
    intake.periodic();
    assertTrue(intake.isBlockedByCapacity());
    assertTrue(intake.inventory().isKnown());
    verify(motor).setPower(0.0);
    intake.inventory().onShot();
    intake.periodic();
    assertFalse(intake.isBlockedByCapacity());
    assertFalse(intake.inventory().isKnown());
    verify(motor).setPower(1.0);
  }

  @Test
  public void fullInventoryStillAllowsFeedingAndEjectingThroughTheSingleWriter() {
    HardwareMap map = mock(HardwareMap.class);
    DcMotorEx motor = mock(DcMotorEx.class);
    config.inventory.entry_sensor_enabled = false;
    when(map.get(DcMotorEx.class, "rightIntake")).thenReturn(motor);
    Intake intake = new Intake(map);
    intake.inventory().set(config.inventory.capacity_count);

    intake.feed(0.7);
    verify(motor, never()).setPower(anyDouble());
    intake.periodic();
    assertFalse(intake.isBlockedByCapacity());
    assertEquals(Intake.Mode.FEED, intake.mode());
    verify(motor).setPower(0.7);

    clearInvocations(motor);
    intake.eject(0.6);
    verifyNoInteractions(motor);
    intake.periodic();
    assertFalse(intake.isBlockedByCapacity());
    assertEquals(Intake.Mode.EJECT, intake.mode());
    verify(motor).setPower(-0.6);

    clearInvocations(motor);
    intake.stop();
    verifyNoInteractions(motor);
    intake.periodic();
    assertFalse(intake.isRunning());
    verify(motor).setPower(0.0);
  }
}
