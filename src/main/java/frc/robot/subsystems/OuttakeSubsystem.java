// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Represents the outtake of the shooter on the robot.
 * Note that this is different than the intake, which is the {@link IntakeSubsystem}
 */
public class OuttakeSubsystem extends SubsystemBase {
  private final MotorController motorController;

  /**
   * Creates a new {@code OuttakeSubsystem} with a motor controller
   * @param motorController The motor controller that outtakes
   */
  public OuttakeSubsystem(MotorController motorController) {
    this.motorController = motorController;
  }

  /**
   * Returns a command that outtakes when it's started, and then stops the motor when it's stopped.
   * <p>
   * If you are binding this to a trigger, make sure you use {@link Trigger#whileTrue(Command)} rather than {@link Trigger#onTrue(Command)}.
   * @param speed The speed to outtake at.
   * Negative numbers shoot the note, while positive numbers suck up a note. (this doesn't actually work because there is not enough torque on the cims.)
   * @return
   */
  public Command outtakeCommand(double speed) {
    return startEnd(() -> motorController.set(speed), () -> motorController.set(0));
  }

  public MotorController getMotorController() {
      return motorController;
  }
}
