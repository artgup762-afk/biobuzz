package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.FlowerFlap.Target;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/** Endpoint checkout with no other mechanisms or camera constructed. */
@TeleOp(name = "Flower Flap Test", group = "Test")
public class FlowerFlapTestOpMode extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry);
    var rig = new Robot.FlapTestRig(hardwareMap);
    try {
      while (opModeInInit()) {
        rig.update();
        telemetry.addLine(
            "No movement at INIT/START. Hold A: backward/up; B: forward/down; release: off.");
        telemetry.addData("Flap", rig.flap.status());
        telemetry.update();
        idle();
      }
      waitForStart();
      while (opModeIsActive()) {
        boolean up = gamepad1.a || gamepad2.a;
        boolean down = gamepad1.b || gamepad2.b;
        rig.flap.setTarget(up == down ? Target.NONE : down ? Target.DOWN : Target.UP);
        rig.update();
        telemetry.addData("Flap", rig.flap.status());
        telemetry.addData("Command readback", rig.flap.outputStatus());
        telemetry.addLine(
            "Hold A: backward/up; B: forward/down; release: off. Physical angle is not measured.");
        telemetry.update();
        idle();
      }
    } finally {
      rig.shutdown();
    }
  }
}
