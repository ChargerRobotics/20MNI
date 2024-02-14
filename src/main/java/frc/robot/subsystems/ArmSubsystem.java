// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the arm on the robot
 */
public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax motorController;
  private final PIDController pidController;

  private double setPoint;

  /**
   * Creates a new {@code ArmSubsystem} with a motor controller and pid controller
   * @param motorController The motor controller that moves the arm
   * @param pidController The pid controller to control the motor controller
   * @param armPositionSupplier A supplier that supplies how much to move the arm. Positive values move the arm down, while negative values move it up.
   */
  public ArmSubsystem(CANSparkMax motorController, PIDController pidController, DoubleSupplier armPositionSupplier) {
    this.motorController = motorController;
    this.pidController = pidController;

    setDefaultCommand(run(() -> {
      setPoint += armPositionSupplier.getAsDouble();

      double speed = MathUtil.clamp(pidController.calculate(motorController.getEncoder().getPosition(), setPoint), -1, 1);
      motorController.set(speed);
    }));
  }

  public void reset() {
    pidController.reset();
    setPoint = 0;
  }

  public CANSparkMax getMotorController() {
      return motorController;
  }

  public PIDController getPidController() {
      return pidController;
  }
}
