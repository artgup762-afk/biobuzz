package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.DualAxonTurretTest.Selection;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/** Manual checkout for two Axon MK2 servos programmed in positional Servo Mode. */
@TeleOp(name = "Dual Axon Turret Test", group = "Test")
public class DualAxonTurretTestOpMode extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry);
    Robot.TurretTestRig rig = new Robot.TurretTestRig(hardwareMap);
    var turret = rig.turret;
    Command enable = turret.enableCommand();
    try {
      while (opModeInInit()) {
        rig.update();
        turret.report(telemetry);
        telemetry.update();
        idle();
      }
      waitForStart();
      while (opModeIsActive()) {
        // SDK edge queries consume latched events: poll every button each loop, even when a
        // selection is disallowed, so a press while LB is held cannot be replayed after release.
        boolean selectLeft = gamepad1.aWasPressed();
        boolean selectRight = gamepad1.bWasPressed();
        boolean selectBoth = gamepad1.xWasPressed();
        boolean enablePressed = gamepad1.leftBumperWasPressed();
        boolean decrease = gamepad1.dpadLeftWasPressed();
        boolean increase = gamepad1.dpadRightWasPressed();
        if (gamepad1.y) {
          enable.cancel();
          turret.stop();
        } else {
          if (!gamepad1.left_bumper) {
            enable.cancel();
            if (selectLeft) turret.select(Selection.LEFT);
            else if (selectRight) turret.select(Selection.RIGHT);
            else if (selectBoth) turret.select(Selection.BOTH);
          } else if (enablePressed) {
            enable.schedule();
          }
          if (decrease != increase) turret.jog(increase);
        }
        rig.update();
        turret.report(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      enable.cancel();
      rig.shutdown();
    }
  }
}
