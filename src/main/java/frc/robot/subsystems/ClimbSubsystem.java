package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private final MotorController motorController;

  public ClimbSubsystem(MotorController motorController) {
    this.motorController = motorController;
  }

  public Command climbCommand(double speed) {
    return startEnd(() -> motorController.set(speed), () -> motorController.set(0));
  }

  public MotorController getMotorController() {
    return motorController;
  }
}

