// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Represents the intake part of the shooter on the robot.
 * Note that this is different than the outtake, which is the {@link OuttakeSubsystem}
 */
public class IntakeSubsystem extends SubsystemBase {
  private final MotorController motorController;

  /**
   * Creates a new {@code IntakeSubsystem} with a motor controller
   * @param motorController The motor controller that intakes
   */
  public IntakeSubsystem(MotorController motorController) {
    this.motorController = motorController;
  }

  /**
   * Returns a command that intakes when it's started, and then stops the motor when it's stopped.
   * <p>
   * If you are binding this to a trigger, make sure you use {@link Trigger#whileTrue(Command)} rather than {@link Trigger#onTrue(Command)}.
   * @param speed The speed to intake at. Negative numbers intake notes, while positive numbers eject the note.
   * @return The newly created command that requires this subsystem
   */
  public Command intakeCommand(double speed) {
    return startEnd(() -> motorController.set(speed), motorController::stopMotor);
  }

  public MotorController getMotorController() {
    return motorController;
  }
}
