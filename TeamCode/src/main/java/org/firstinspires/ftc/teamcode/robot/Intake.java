package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;

/**
 * Single-motor intake and feed mechanism, plus the G407 four-element inventory.
 *
 * <p>Single writer: commands, {@link ShotController} and OpModes set a target with {@link #run},
 * {@link #feed}, {@link #eject} or {@link #stop}; only {@link #periodic()} writes motor power. That
 * is where the inventory cap is applied, so no caller can drive a fifth SCORING ELEMENT in by
 * accident.
 */
public class Intake {

  /** What the rollers are doing this loop. FEED and EJECT ignore the inventory cap. */
  public enum Mode {
    STOPPED,
    COLLECT,
    FEED,
    EJECT
  }

  private final DcMotorEx rightIntake;
  private final @Nullable DigitalChannel entrySensor;
  private final BallInventory inventory;

  private Mode mode = Mode.STOPPED;
  private double targetPower = 0.0;
  private boolean blockedByCapacity = false;

  public Intake(HardwareMap hardwareMap) {
    rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");

    rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

    var inv = config.inventory;
    DigitalChannel sensor =
        inv.entry_sensor_enabled
            ? hardwareMap.tryGet(DigitalChannel.class, inv.entry_sensor_name)
            : null;
    if (sensor != null) {
      sensor.setMode(DigitalChannel.Mode.INPUT);
    }
    entrySensor = sensor;
    inventory = new BallInventory(inv.capacity_count, sensor != null);
  }

  /** Collect from the floor or a FLOWER: stops itself once the inventory is full. */
  public void run(double power) {
    setTarget(Mode.COLLECT, power);
  }

  /** Push SCORING ELEMENTS into the shooter. Not capped: feeding only ever empties the robot. */
  public void feed(double power) {
    setTarget(Mode.FEED, power);
  }

  /** Reverse everything to spit out a fifth element (G407) or opponent NECTAR (G408). */
  public void eject(double power) {
    double reverse = -Math.abs(power);
    setTarget(Mode.EJECT, reverse);
  }

  public void stop() {
    setTarget(Mode.STOPPED, 0.0);
  }

  private void setTarget(Mode newMode, double power) {
    mode = newMode;
    targetPower = power;
  }

  public Mode mode() {
    return mode;
  }

  /** Read the actual intake controller, independently of the bridge's hub-index mapping. */
  public String getMotorStatus() {
    return String.format(
        Locale.ROOT,
        "%s; %.0f ticks/s; command %.2f; port %d",
        mode,
        rightIntake.getVelocity(),
        rightIntake.getPower(),
        rightIntake.getPortNumber());
  }

  public boolean isRunning() {
    return mode != Mode.STOPPED;
  }

  /** True when the last {@link #periodic()} held the intake off because the robot was full. */
  public boolean isBlockedByCapacity() {
    return blockedByCapacity;
  }

  public BallInventory inventory() {
    return inventory;
  }

  /** G407: the proven predicate {@link BallInventory#mayCollect}; unknown never blocks. */
  private boolean mayCollect() {
    return BallInventory.mayCollect(inventory.isKnown(), inventory.count(), inventory.capacity());
  }

  /** The only place intake motor power is written. Called once per loop by Robot.update(). */
  public void periodic() {
    if (entrySensor != null) {
      boolean raw = entrySensor.getState();
      boolean blocked = config.inventory.entry_sensor_active_low ? !raw : raw;
      inventory.updateEntrySensor(
          blocked,
          System.nanoTime() / 1_000_000L,
          config.inventory.entry_debounce_ms,
          mode == Mode.EJECT);
    } else if ((mode == Mode.COLLECT && mayCollect()) || mode == Mode.EJECT) {
      inventory.markUnknown();
    }

    blockedByCapacity = mode == Mode.COLLECT && !mayCollect();
    boolean powered = mode != Mode.STOPPED && !blockedByCapacity;
    rightIntake.setPower(powered ? targetPower : 0.0);
  }

  public Command runIntakeCommand(double power) {
    return Command.build().setStart(() -> run(power)).setEnd(interrupted -> stop()).requiring(this);
  }

  public Command ejectCommand(double power) {
    return Command.build()
        .setStart(() -> eject(power))
        .setEnd(interrupted -> stop())
        .requiring(this);
  }

  public Command stopIntakeCommand() {
    return Command.build().setStart(this::stop).setDone(() -> true).requiring(this);
  }
}
