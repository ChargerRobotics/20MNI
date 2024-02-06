// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.DriveSpeedControlCurve;
import frc.robot.drivetrain.OmniDriveTrain;
import frc.robot.drivetrain.OmniSpeeds;
import frc.robot.subsystems.DriveTrainSubsystem;

import java.util.Map;

public class RobotContainer {
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final CommandXboxController controller = new CommandXboxController(0);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private final DriveSpeedControlCurve driveSpeedCurve = new DriveSpeedControlCurve(0.05, 0);

  private final SendableChooser<DriveSpeedControlCurve.SpeedType> speedTypeChooser = new SendableChooser<>();
  {
    speedTypeChooser.setDefaultOption(DriveSpeedControlCurve.DEFAULT_SPEED_TYPE.toString(), DriveSpeedControlCurve.DEFAULT_SPEED_TYPE);
    speedTypeChooser.addOption("Linear", DriveSpeedControlCurve.SpeedType.LINEAR);
    speedTypeChooser.onChange(driveSpeedCurve::setSpeedType);
  }

  private GenericEntry maxSpeedEntry;
  private GenericEntry fieldCentricEntry;
  private GenericEntry dPadEntry;

  public RobotContainer(ShuffleboardTab robotTab, ShuffleboardTab debugTab) {
    OmniDriveTrain driveTrain = new OmniDriveTrain(
      new PWMSparkMax(Ports.LEFT_MOTOR_CHANNEL),
      new PWMSparkMax(Ports.TOP_MOTOR_CHANNEL),
      new PWMSparkMax(Ports.RIGHT_MOTOR_CHANNEL),
      new PWMSparkMax(Ports.BOTTOM_MOTOR_CHANNEL)
    );
    this.driveTrainSubsystem = new DriveTrainSubsystem(driveTrain, () -> {
      driveSpeedCurve.setMaxSpeed(maxSpeedEntry.getDouble(DriveSpeedControlCurve.DEFAULT_MAX_SPEED));

      double magnitude;
      double angleRadians;

      if (!dPadEntry.getBoolean(false)) {
        magnitude = Math.hypot(controller.getLeftX(), controller.getLeftY());
        magnitude = driveSpeedCurve.getControlCurve().get(MathUtil.clamp(magnitude, -1, 1));

        angleRadians = getAngle(controller.getLeftX(), -controller.getLeftY());
      } else {
        int pov = controller.getHID().getPOV();
        magnitude = pov == -1 ? 0 : 1;
        angleRadians = Math.toRadians(pov);
      }

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

    tab.add("Speed Type", speedTypeChooser)
            .withPosition(2, 0)
            .withSize(2, 1);

    maxSpeedEntry = tab.add("Max Speed", DriveSpeedControlCurve.DEFAULT_MAX_SPEED)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", 0, "Max", 1))
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();

    fieldCentricEntry = tab.add("field centric", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(2, 1)
            .withSize(1, 1)
            .getEntry();

    dPadEntry = tab.add("use dpad", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(2, 2)
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
