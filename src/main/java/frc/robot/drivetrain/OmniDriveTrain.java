package frc.robot.drivetrain;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class OmniDriveTrain implements Sendable {
  private final MotorController left;
  private final MotorController top;
  private final MotorController right;
  private final MotorController bottom;

  private double heading;

  public OmniDriveTrain(MotorController left, MotorController top, MotorController right, MotorController bottom) {
    this.left = left;
    this.top = top;
    this.right = right;
    this.bottom = bottom;
  }

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