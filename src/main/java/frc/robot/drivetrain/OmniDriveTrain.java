package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class OmniDriveTrain {
  private final MotorControllerGroup left;
  private final MotorControllerGroup top;
  private final MotorControllerGroup right;
  private final MotorControllerGroup bottom;

  public OmniDriveTrain(MotorController left, MotorController top, MotorController right, MotorController bottom) {
    this(new MotorControllerGroup(left), new MotorControllerGroup(top), new MotorControllerGroup(right), new MotorControllerGroup(bottom));
  }
  public OmniDriveTrain(MotorControllerGroup left, MotorControllerGroup top, MotorControllerGroup right, MotorControllerGroup bottom) {
    this.left = left;
    this.top = top;
    this.right = right;
    this.bottom = bottom;
  }

  public void drive(OmniSpeeds speeds) {
    left.set(speeds.getLeftPower());
    right.set(-speeds.getRightPower());
    top.set(speeds.getTopPower());
    bottom.set(-speeds.getBottomPower());
  }
}