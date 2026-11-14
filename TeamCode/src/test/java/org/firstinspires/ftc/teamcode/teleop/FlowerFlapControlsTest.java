package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.FlowerFlap;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class FlowerFlapControlsTest {
  @AfterEach
  void resetScheduler() {
    Scheduler.reset();
  }

  @Test
  void operatorLeftDpadTogglesOncePerPressAndCancelRemovesTheTarget() {
    Scheduler.reset();
    TeleOp op = new TeleOp();
    op.driver = new Gamepad();
    op.flap = new FlowerFlap(mock(HardwareMap.class), false);
    op.intake = mock(Intake.class);
    when(op.intake.runIntakeCommand(anyDouble())).thenReturn(Command.build());
    when(op.intake.ejectCommand(anyDouble())).thenReturn(Command.build());
    op.shooter = mock(Shooter.class);
    op.turret = mock(Turret.class);
    op.follower = mock(Follower.class);
    when(op.follower.pose()).thenReturn(new Pose(30, 30, 0));
    op.casablanca = mock(Casablanca.class);
    op.shotController = mock(ShotController.class);
    when(op.shotController.currentTarget())
        .thenReturn(new CellTarget(HiveCell.A, 50, 50, 60, 0, 0));
    when(op.shotController.aimAndShootCommand(op.follower)).thenReturn(Command.build());
    op.buildCommands();

    op.operatorDpadLeft.update(true);
    op.onLoop();
    Scheduler.execute();
    assertEquals(FlowerFlap.Target.DOWN, op.flap.target());
    for (int i = 0; i < 5; i++) {
      op.operatorDpadLeft.update(true);
      op.onLoop();
      Scheduler.execute();
      assertEquals(FlowerFlap.Target.DOWN, op.flap.target());
    }
    op.operatorDpadLeft.update(false);
    op.onLoop();
    Scheduler.execute();
    op.operatorDpadLeft.update(true);
    op.onLoop();
    Scheduler.execute();
    assertEquals(FlowerFlap.Target.UP, op.flap.target());
    op.operatorDpadLeft.update(false);
    op.operatorLeftTrigger.update(true);
    op.onLoop();
    Scheduler.execute();
    assertEquals(FlowerFlap.Target.NONE, op.flap.target());
    verify(op.follower, never()).hold(any());
  }
}
