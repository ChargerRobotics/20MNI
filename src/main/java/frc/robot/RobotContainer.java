// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.datasiqn.robotutils.controlcurve.ControlCurve;
import com.datasiqn.robotutils.controlcurve.ControlCurves;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.OmniDriveTrain;
import frc.robot.drivetrain.OmniSpeeds;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RobotContainer {
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final XboxController controller = new XboxController(0);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final ControlCurve driveSpeedCurve = ControlCurves.power(3)
          .withDeadZone(0.05)
          .withPowerMultiplier(0.5)
          .build();

  private GenericEntry fieldCentricEntry;

  public RobotContainer(ShuffleboardTab robotTab, ShuffleboardTab debugTab) {
    OmniDriveTrain driveTrain = new OmniDriveTrain(
      new PWMSparkMax(Ports.LEFT_MOTOR_CHANNEL),
      new PWMSparkMax(Ports.TOP_MOTOR_CHANNEL),
      new PWMSparkMax(Ports.RIGHT_MOTOR_CHANNEL),
      new PWMSparkMax(Ports.BOTTOM_MOTOR_CHANNEL)
    );
    this.driveTrainSubsystem = new DriveTrainSubsystem(driveTrain, () -> {
      double magnitude = Math.hypot(controller.getLeftX(), controller.getLeftY());
      magnitude = driveSpeedCurve.get(MathUtil.clamp(magnitude, -1, 1));

      double angleRadians = getAngle(controller.getLeftX(), -controller.getLeftY());

      double rotatePower = controller.getRightX() * 0.25;
      double heading = Math.toRadians(gyro.getAngle());

      if (fieldCentricEntry.getBoolean(true)) {
        return OmniSpeeds.fromRelative(magnitude, angleRadians, rotatePower, heading);
      } else {
        return OmniSpeeds.from(magnitude, angleRadians, rotatePower, heading);
      }
    });
    gyro.calibrate();

    configureBindings();

    addRobotData(robotTab);
    addDebugData(debugTab);
  }

  public void reset() {
    gyro.reset();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configureBindings() {}

  private void addRobotData(ShuffleboardTab tab) {
    tab.add("drivetrain", driveTrainSubsystem.getDriveTrain())
            .withPosition(3, 1)
            .withSize(4, 3);

    fieldCentricEntry = tab.add("field centric", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(2, 1)
            .withSize(1, 1)
            .getEntry();

    Button resetGyroCommand = new Button("reset", gyro::reset);
    Button calibrateGyroCommand = new Button("calibrate", gyro::calibrate);

    ShuffleboardLayout gyroList = tab.getLayout("Gyro", BuiltInLayouts.kList)
            .withPosition(7, 1)
            .withSize(2, 3);
    gyroList.add("reset", resetGyroCommand);
    gyroList.add("calibrate", calibrateGyroCommand);
  }

  private void addDebugData(ShuffleboardTab tab) {
    tab.add("gyro", gyro)
            .withPosition(5, 0)
            .withSize(2, 2);

    tab.addDouble("controller mag", () -> Math.hypot(controller.getLeftX(), controller.getLeftY()))
            .withPosition(3, 1)
            .withSize(2, 1);

    tab.addDouble("controller angle", () -> Math.toDegrees(RobotContainer.getAngle(controller.getLeftX(), -controller.getLeftY())))
            .withPosition(3, 0)
            .withSize(2, 1);
  }

  private static double getAngle(double x, double y) {
    if (y == 0 && x == 0) return 0;
    if (y == 0) {
      if (x > 0) return Math.PI / 2; // 90 deg
      else if (x < 0) return 3 * Math.PI / 2; // 270 deg
    }
    if (x == 0) {
      if (y > 0) return 0;
      else if (y < 0) return Math.PI; // 180 deg
    }
    double angle = Math.atan(x / y);
    if (y < 0) return angle + Math.PI;
    if (x < 0) return angle + Math.PI * 2;
    return angle;
  }
}
