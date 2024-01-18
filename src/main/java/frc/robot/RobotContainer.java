// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.OmniDriveTrain;
import frc.robot.drivetrain.OmniSpeeds;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RobotContainer {
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final XboxController controller = new XboxController(0);
  private final Gyro gyro = new ADXRS450_Gyro();

  public RobotContainer() {
    OmniDriveTrain driveTrain = new OmniDriveTrain(
      new PWMSparkMax(Ports.LEFT_MOTOR_CHANNEL),
      new PWMSparkMax(Ports.TOP_MOTOR_CHANNEL),
      new PWMSparkMax(Ports.RIGHT_MOTOR_CHANNEL),
      new PWMSparkMax(Ports.BOTTOM_MOTOR_CHANNEL)
    );
    this.driveTrainSubsystem = new DriveTrainSubsystem(driveTrain, () -> {
      double magnitude = Math.hypot(controller.getLeftX(), controller.getLeftY());
      double angleRadians = getAngle(controller.getLeftX(), controller.getLeftY());
      return OmniSpeeds.fromRelative(magnitude * 0.75, angleRadians, controller.getRightX() * 0.25, gyro.getAngle());
    });
    gyro.calibrate();

    configureBindings();
  }

  public void reset() {
    gyro.reset();
  }

  public void periodic() {
    double magnitude = Math.hypot(controller.getLeftX(), controller.getLeftY());
    double angleRadians = getAngle(controller.getLeftX(), controller.getLeftY());
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    SmartDashboard.putNumber("Controller x", controller.getLeftX());
    SmartDashboard.putNumber("Controller y", controller.getLeftY());
    SmartDashboard.putNumber("Controller mag", magnitude);
    SmartDashboard.putNumber("Controller angle", Math.toDegrees(angleRadians));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configureBindings() {}

  private static double getAngle(double x, double y) {
    if (y == 0 && x == 0) return 0;
    if (y == 0) {
      if (x > 0) return 90;
      else if (x < 0) return 270;
    }
    if (x == 0) {
      if (y > 0) return 0;
      else if (y < 0) return 180;
    }
    double angle = Math.atan(x / -y);
    if (y > 0) return angle + 180;
    if (x < 0) return angle + 360;
    return angle;
  }
}