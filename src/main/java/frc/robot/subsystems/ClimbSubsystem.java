package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private final MotorController motorController;
  private final MotorController otherMotorController;

  private double extendSpeed = 0.9;
  private double retractSpeed = -0.9;

  public ClimbSubsystem(MotorController motorController, MotorController otherMotorController) {
    this.motorController = motorController;
    this.otherMotorController = otherMotorController;
  }

  public Command extendCommand() {
    return startEnd(() -> {
      motorController.set(extendSpeed);
      otherMotorController.set(extendSpeed);
    }, () -> {
      motorController.set(0);
      otherMotorController.set(0);
    });
  }

  public Command retractCommand() {
    return startEnd(() -> {
      motorController.set(retractSpeed);
      otherMotorController.set(retractSpeed);
    }, () -> {
      motorController.set(0);
      otherMotorController.set(0);
    });
  }

  public Command climbCommand(double speed) {
    return startEnd(() -> {
      motorController.set(speed);
      otherMotorController.set(speed);
    }, () -> {
      motorController.set(0);
      otherMotorController.set(0);
    });
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

    Command leftExtendCommand = startEnd(() -> otherMotorController.set(extendSpeed), otherMotorController::stopMotor);
    leftExtendCommand.setName("left extend");
    tab.add(leftExtendCommand)
            .withPosition(1, 2);

    Command leftRetractCommand = startEnd(() -> otherMotorController.set(retractSpeed), otherMotorController::stopMotor);
    leftRetractCommand.setName("left retract");
    tab.add(leftRetractCommand)
            .withPosition(2, 2);

    Command rightExtendCommand = startEnd(() -> motorController.set(extendSpeed), motorController::stopMotor);
    rightExtendCommand.setName("right extend");
    tab.add(rightExtendCommand)
            .withPosition(3, 2);

    Command rightRetractCommand = startEnd(() -> motorController.set(retractSpeed), motorController::stopMotor);
    rightRetractCommand.setName("right retract");
    tab.add(rightRetractCommand)
            .withPosition(4, 2);
  }
}

