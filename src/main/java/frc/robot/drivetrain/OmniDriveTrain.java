package frc.robot.drivetrain;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Represents the omni drive train. This can be sent over Shuffleboard.
 */
public class OmniDriveTrain implements Sendable {
  private final MotorController left;
  private final MotorController top;
  private final MotorController right;
  private final MotorController bottom;

  private double heading;

  /**
   * Creates a new omni drive train with the 4 side motors
   * @param left The motor on the left side
   * @param top The motor on the top side
   * @param right The motor on the right side
   * @param bottom The motor on the bottom side
   */
  public OmniDriveTrain(MotorController left, MotorController top, MotorController right, MotorController bottom) {
    this.left = left;
    this.top = top;
    this.right = right;
    this.bottom = bottom;
  }

  /**
   * Drives this drive train with the speeds of {@code speeds}
   * @param speeds The speeds of the drive train
   */
  public void drive(OmniSpeeds speeds) {
    heading = speeds.heading();
    left.set(speeds.leftPower());
    right.set(-speeds.rightPower());
    top.set(speeds.topPower());
    bottom.set(-speeds.bottomPower());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("OMNI_DRIVETRAIN");
    builder.addDoubleProperty("heading", () -> heading, null);
    builder.addDoubleProperty("leftPower", left::get, null);
    builder.addDoubleProperty("topPower", top::get, null);
    builder.addDoubleProperty("rightPower", () -> -right.get(), null);
    builder.addDoubleProperty("bottomPower", () -> -bottom.get(), null);
  }
}