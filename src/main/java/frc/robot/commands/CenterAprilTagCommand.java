// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.drivetrain.OmniSpeeds;
import frc.robot.subsystems.DriveTrainSubsystem;

public class CenterAprilTagCommand extends Command {
  private final RobotContainer robot;
  private final PIDController pid;
  private final int id;

  public CenterAprilTagCommand(RobotContainer robot, PIDController pid, int id) {
    this.robot = robot;
    this.pid = pid;
    this.id = id;

    addRequirements(robot.getDriveTrainSubsystem());
  }

  @Override
  public void execute() {
    DriveTrainSubsystem driveTrainSubsystem = robot.getDriveTrainSubsystem();
    double x = LimelightHelpers.getTX("") - 12;
    double heading = Math.toRadians(robot.getGyro().getAngle());

    driveTrainSubsystem.drive(OmniSpeeds.from(MathUtil.clamp(pid.calculate(-x, 0), -1, 1), Math.PI / 2, 0, heading));
  }

  @Override
  public void end(boolean interrupted) {
    robot.getDriveTrainSubsystem().drive(OmniSpeeds.from(0, 0, 0, Math.toRadians(robot.getGyro().getAngle())));
    pid.reset();
  }

  @Override
  public boolean isFinished() {
    return LimelightHelpers.getFiducialID("") != id || pid.atSetpoint();
  }
}
