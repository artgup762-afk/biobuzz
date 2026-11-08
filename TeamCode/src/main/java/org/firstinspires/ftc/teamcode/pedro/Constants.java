package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/** Builds Pedro 3 from the robot's existing motor and Pinpoint setup. */
public final class Constants {
  private Constants() {}

  public static MecanumConfig drivetrainConfig() {
    var d = config.pedro.drivetrain;
    return new MecanumConfig(
        c -> {
          c.frontLeftName.set(d.front_left_motor);
          c.frontRightName.set(d.front_right_motor);
          c.backLeftName.set(d.back_left_motor);
          c.backRightName.set(d.back_right_motor);
          c.frontLeftDirection.set(DcMotorSimple.Direction.valueOf(d.front_left_direction));
          c.frontRightDirection.set(DcMotorSimple.Direction.valueOf(d.front_right_direction));
          c.backLeftDirection.set(DcMotorSimple.Direction.valueOf(d.back_left_direction));
          c.backRightDirection.set(DcMotorSimple.Direction.valueOf(d.back_right_direction));
        });
  }

  public static PinpointConfig localizerConfig() {
    var l = config.pedro.localization;
    return new PinpointConfig(
        c -> {
          c.name.set(l.pinpoint_name);
          c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.valueOf(l.pod_type));
          c.xPodOffset.set(l.x_pod_offset);
          c.yPodOffset.set(l.y_pod_offset);
          c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.valueOf(l.x_pod_direction));
          c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.valueOf(l.y_pod_direction));
          c.offsetUnits.set(DistanceUnit.INCH);
          c.globalDistanceUnit.set(DistanceUnit.INCH);
        });
  }

  public static ForesightConfig foresightConfig() {
    var p = config.pedro;
    return new ForesightConfig(
        c -> {
          c.headingFeedback.set(pid(p.heading.pid));
          c.headingStaticFF.set(Controller.staticFeedforward(p.heading.ks));
          c.forwardTranslational.set(
              Controller.piecewise(pid(p.translation.forward.small_error_pid))
                  .put(p.translation.gain_threshold, pid(p.translation.forward.large_error_pid)));
          c.strafeTranslational.set(
              Controller.piecewise(pid(p.translation.strafe.small_error_pid))
                  .put(p.translation.gain_threshold, pid(p.translation.strafe.large_error_pid)));
          c.coast.set(Controller.proportionalFeedforward(p.feedforward.coast_kv));
          c.brake.set(Controller.proportionalFeedforward(p.feedforward.brake_kv));
          c.linearBrakeCoefficients.set(
              Matrix.diag(p.braking.forward.linear, p.braking.strafe.linear));
          c.quadraticBrakeCoefficients.set(
              Matrix.diag(p.braking.forward.quadratic, p.braking.strafe.quadratic));
          c.headingBrakeCoefficients.set(
              Vector2D.cartesian(p.braking.heading.linear, p.braking.heading.quadratic));
          c.maxAchievableForwardVelocity.set(p.motion.max_forward_velocity);
          c.maxAchievableStrafeVelocity.set(p.motion.max_strafe_velocity);
          c.naturalForwardDeceleration.set(p.motion.natural_forward_deceleration);
          c.naturalStrafeDeceleration.set(p.motion.natural_strafe_deceleration);
          c.maxPathSpeed.set(p.motion.max_path_speed);
        });
  }

  private static Controller pid(PIDFCoefficients gains) {
    return Controller.pid(gains.p, gains.i, gains.d);
  }

  public static Follower create(HardwareMap hardwareMap) {
    return new Follower(
        new PinpointLocalizer(hardwareMap, localizerConfig()),
        new SlewLimitedDrivetrain(new Mecanum(hardwareMap, drivetrainConfig())),
        new Foresight(foresightConfig()));
  }
}
