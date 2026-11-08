package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Runs one or both flywheel motors at {@link #targetVelocity} (encoder ticks/second) on the hub's
 * velocity loop and reports the flywheel's velocity, peak velocity, and time to reach 95% of
 * target. Each enabled motor closes the loop on its own encoder. Both motors turn one flywheel, so
 * the flywheel speed is the faster of the two encoder readings.
 *
 * <p>The hub keeps whatever velocity PIDF was last written to it (e.g. {@code shooter.motor_pidf})
 * until it is power-cycled; this test does not set one.
 *
 * <p>Controls: X stops the motors (latched), A restarts the run, Y resets the peak/timing stats.
 */
@TeleOp(name = "Shooter Max Velocity Test", group = "Test")
@Configurable
public class MaxVelocityTest extends LinearOpMode {
  public enum EnabledMotors {
    SHOOTER1,
    SHOOTER2,
    BOTH
  }

  public static String name = "shooter";
  public static String name2 = "shooter2";

  public static EnabledMotors enabled = EnabledMotors.BOTH;

  // Both motors drive the same flywheel from opposite sides, so they must spin in opposite
  // directions — these defaults match Shooter (shooter FORWARD, shooter2 REVERSE).
  public static boolean shooter1forward = true;
  public static boolean shooter2forward = false;

  /** Velocity command for each enabled motor, encoder ticks/second. */
  public static double targetVelocity = 1000.0;

  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry);
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, name);
    DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, name2);
    for (DcMotorEx motor : new DcMotorEx[] {motor1, motor2}) {
      // FLOAT so a disabled motor coasts with the flywheel instead of braking against the other.
      motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    ElapsedTime runtime = new ElapsedTime();
    boolean running = true;
    boolean timeRecorded = false;
    String timeTaken = "-";
    double peakVelocity = 0;

    waitForStart();
    if (isStopRequested()) return;
    runtime.reset();

    while (opModeIsActive()) {
      if (gamepad1.xWasPressed()) {
        running = false;
      }
      if (gamepad1.aWasPressed() && !running) {
        running = true;
        runtime.reset();
        timeRecorded = false;
        timeTaken = "-";
      }
      if (gamepad1.yWasPressed()) {
        runtime.reset();
        peakVelocity = 0;
        timeRecorded = false;
        timeTaken = "-";
      }

      motor1.setDirection(
          shooter1forward ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
      motor2.setDirection(
          shooter2forward ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);

      boolean motor1Enabled = running && enabled != EnabledMotors.SHOOTER2;
      boolean motor2Enabled = running && enabled != EnabledMotors.SHOOTER1;
      drive(motor1, motor1Enabled);
      drive(motor2, motor2Enabled);

      double velocity1 = motor1.getVelocity();
      double velocity2 = motor2.getVelocity();
      double flywheelVelocity = Math.max(Math.abs(velocity1), Math.abs(velocity2));
      peakVelocity = Math.max(peakVelocity, flywheelVelocity);

      if (running && !timeRecorded && flywheelVelocity >= 0.95 * targetVelocity) {
        timeTaken = runtime.toString();
        timeRecorded = true;
      }

      telemetryM.addData("Enabled", enabled);
      telemetryM.addData("Running", running);
      telemetryM.addData("Target Velocity", targetVelocity);
      telemetryM.addData("Flywheel Velocity", flywheelVelocity);
      telemetryM.addData("Peak Velocity", peakVelocity);
      telemetryM.addData("Time to 95%", timeTaken);
      telemetryM.addData(name + " encoder", velocity1);
      telemetryM.addData(name2 + " encoder", velocity2);
      telemetryM.addData(name + " power", motor1.getPower());
      telemetryM.addData(name2 + " power", motor2.getPower());
      telemetryM.update();
    }
  }

  /**
   * Runs the motor at {@link #targetVelocity}, or cuts power and lets it coast. A disabled motor is
   * switched to RUN_WITHOUT_ENCODER because in RUN_USING_ENCODER even power 0 is a velocity-0
   * command that would hold the shared flywheel back.
   */
  private static void drive(DcMotorEx motor, boolean spin) {
    if (spin) {
      if (motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      }
      motor.setVelocity(targetVelocity);
    } else {
      if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      }
      motor.setPower(0);
    }
  }
}
