package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private final MotorController motorController;

  private double extendSpeed = -0.5;
  private double retractSpeed = 0.9;

  public ClimbSubsystem(MotorController motorController) {
    this.motorController = motorController;
  }

  public Command extendCommand() {
    return startEnd(() -> motorController.set(extendSpeed), () -> motorController.set(0));
  }

  public Command retractCommand() {
    return startEnd(() -> motorController.set(retractSpeed), () -> motorController.set(0));
  }

  public Command climbCommand(double speed) {
    return startEnd(() -> motorController.set(speed), () -> motorController.set(0));
  }

  public double getExtendSpeed() {
    return extendSpeed;
  }

  public void setExtendSpeed(double extendSpeed) {
    this.extendSpeed = extendSpeed;
  }

  public double getRetractSpeed() {
    return retractSpeed;
  }

  public void setRetractSpeed(double retractSpeed) {
    this.retractSpeed = retractSpeed;
  }

  public MotorController getMotorController() {
    return motorController;
  }

  public void addShuffleboardData(ShuffleboardTab tab) {
    tab.addDouble("motor power", motorController::get)
            .withPosition(2, 0)
            .withSize(2, 1);

    Command climbExtendCommand = extendCommand();
    climbExtendCommand.setName("extend");
    tab.add(climbExtendCommand)
            .withPosition(2, 1);

    Command climbRetractCommand = retractCommand();
    climbRetractCommand.setName("retract");
    tab.add(climbRetractCommand)
            .withPosition(3, 1);
  }
}

