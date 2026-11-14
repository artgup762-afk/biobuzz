package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class ShotTimingFeedTest {
  @ParameterizedTest
  @ValueSource(booleans = {false, true})
  void benchFeedsAtSpeedWhileAutomaticShotsStillRequireCalibration(boolean bench) {
    config.shooter.shot_table.calibrated = false;
    HardwareMap map = mock(HardwareMap.class);
    DcMotorEx motor = mock(DcMotorEx.class);
    when(map.get(DcMotorEx.class, "rightIntake")).thenReturn(motor);
    Intake intake = new Intake(map);
    Shooter shooter = mock(Shooter.class);
    when(shooter.getTargetPower()).thenReturn(Shooter.constantPower());
    Turret turret = mock(Turret.class);
    when(turret.isEnabled()).thenReturn(true);
    when(turret.isAimed(any())).thenReturn(false);
    ShotController controller =
        new ShotController(
            shooter,
            turret,
            intake,
            () -> new Pose(30, 30, 0),
            Velocity::zero,
            null,
            new FieldModel(Alliance.RED),
            new HiveTracker(3, 1000, 500, () -> 0),
            null,
            (inputs, previous) -> ShotSolution.invalid("table not calibrated"));
    try {
      controller.startShot(Shooter.constantPower(), !bench, !bench, !bench);
      controller.periodic();
      intake.periodic();
      verify(motor).setPower(0);
      assertFalse(controller.isFeedCommanded());

      when(shooter.getShooterVelocity()).thenReturn(config.shooter.constant_rpm);
      clearInvocations(motor);
      controller.periodic();
      intake.periodic();
      assertEquals(bench, controller.isFeedCommanded());
      verify(motor).setPower(bench ? config.shooter.feed_intake_power : 0);
      if (bench) {
        assertEquals(Intake.Mode.FEED, intake.mode());
        verify(turret, never()).setAimMode(Turret.AimMode.AIM_AT_TARGET);
      }

      when(shooter.getShooterVelocity()).thenReturn(0.0);
      clearInvocations(motor);
      controller.periodic();
      intake.periodic();
      verify(motor).setPower(0);
      assertFalse(controller.isFeedCommanded());

      controller.stopShot();
      when(shooter.getShooterVelocity()).thenReturn(config.shooter.constant_rpm);
      clearInvocations(motor);
      controller.periodic();
      intake.periodic();
      verify(motor).setPower(0);
      verify(shooter).setTargetPower(0);
    } finally {
      controller.shutdown();
    }
  }
}
