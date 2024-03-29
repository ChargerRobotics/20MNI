// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.DriveSpeedControlCurve;
import frc.robot.drivetrain.OmniDriveTrain;
import frc.robot.drivetrain.OmniSpeeds;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.Setpoint;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class RobotContainer {
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final ArmSubsystem armSubsystem;
  private final Shooter shooter;
  private final ClimbSubsystem climbSubsystem;

  private final CommandXboxController controller = new CommandXboxController(0);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private final DriveSpeedControlCurve driveSpeedCurve = new DriveSpeedControlCurve(0.05, 0.025);

  private final PIDController centerPidController = new PIDController(0.01, 0.015, 0);
  {
    centerPidController.setTolerance(0.1);
    centerPidController.setIntegratorRange(-0.2, 0.2);
  }

  private final SendableChooser<DriveSpeedControlCurve.SpeedType> speedTypeChooser = new SendableChooser<>();
  {
    speedTypeChooser.setDefaultOption(DriveSpeedControlCurve.DEFAULT_SPEED_TYPE.toString(), DriveSpeedControlCurve.DEFAULT_SPEED_TYPE);
    speedTypeChooser.addOption("Linear", DriveSpeedControlCurve.SpeedType.LINEAR);
    speedTypeChooser.onChange(driveSpeedCurve::setSpeedType);
  }

  private final SendableChooser<AutonomousMode> autonomousChooser = new SendableChooser<>();
  {
    autonomousChooser.setDefaultOption(AutonomousMode.MIDDLE_SPEAKER.toString(), AutonomousMode.MIDDLE_SPEAKER);
    for (AutonomousMode mode : AutonomousMode.values()) {
      if (mode == AutonomousMode.MIDDLE_SPEAKER) continue;
      autonomousChooser.addOption(mode.toString(), mode);
    }
  }
  private GenericEntry maxSpeedEntry;
  private GenericEntry fieldCentricEntry;
  private GenericEntry dPadEntry;

  /**
   * Creates a new {@code RobotContainer} with the robot tab of {@code robotTab} and a debug tab of {@code debugTab}
   */
  public RobotContainer() {
    this.driveTrainSubsystem = createDriveTrainSubsystem();
    this.armSubsystem = createArmSubsystem();
    this.shooter = new Shooter(createIntakeSubsystem(), createOuttakeSubsystem());
    this.climbSubsystem = createClimbSubsystem();

    gyro.calibrate();

    configureBindings();

    addRobotData(Shuffleboard.getTab("robot"));
    shooter.addShuffleboardData(Shuffleboard.getTab("shooter"));
    armSubsystem.addShuffleboardData(Shuffleboard.getTab("arm"));
    climbSubsystem.addShuffleboardData(Shuffleboard.getTab("climb"));
  }

  public void reset() {
    gyro.reset();
    armSubsystem.reset();
  }

  public Command getAutonomousCommand() {
    AutonomousMode selected = autonomousChooser.getSelected();
    Command shootCommand = Commands.sequence(
      armSubsystem.unstuckCommand(),
      Commands.waitUntil(() -> armSubsystem.getSetpoint().getPidController().atSetpoint()),
      armSubsystem.setSetpointCommand(Setpoint.DOWN_SPEAKER),
      Commands.waitUntil(() -> armSubsystem.getSetpoint().getPidController().atSetpoint()),
      armSubsystem.setSetpointCommand(Setpoint.SPEAKER),
      Commands.waitSeconds(0.5),
      shooter.outtakeNoteCommand()
    );

    // Optional<Alliance> alliance = DriverStation.getAlliance();
    // if (alliance.isEmpty()) throw new IllegalStateException("invalid alliance");

    return switch (selected) {
      case MIDDLE_SPEAKER -> shootCommand.andThen(driveTrainSubsystem.driveSecondsCommand(OmniSpeeds.from(0.5, 180, 0, gyro.getAngle()), gyro::getAngle, 1));

      case LEFT_SPEAKER, RIGHT_SPEAKER -> shootCommand;

      case AMP -> Commands.print("no amp");

      case MOVE_FOWARD -> driveTrainSubsystem.driveSecondsCommand(OmniSpeeds.from(0.5, 0, 0, gyro.getAngle()), gyro::getAngle, 3);

      case NONE -> Commands.none();
    };
  }

  public DriveTrainSubsystem getDriveTrainSubsystem() {
    return driveTrainSubsystem;
  }

  public ADXRS450_Gyro getGyro() {
    return gyro;
  }

  /**
   * Creates a new {@link DriveTrainSubsystem} that moves based off the xbox controller's joysticks.
   * <p>
   * The left joystick moves the robot (unless dpad mode is enabled. in that case, the dpad moves the robot), while the right joystick rotates it.
   * @return The newly created {@link DriveTrainSubsystem}
   */
  private DriveTrainSubsystem createDriveTrainSubsystem() {
    OmniDriveTrain driveTrain = new OmniDriveTrain(
            new PWMSparkMax(Ports.LEFT_MOTOR_CHANNEL),
            new PWMSparkMax(Ports.TOP_MOTOR_CHANNEL),
            new PWMSparkMax(Ports.RIGHT_MOTOR_CHANNEL),
            new PWMSparkMax(Ports.BOTTOM_MOTOR_CHANNEL)
    );
    return new DriveTrainSubsystem(driveTrain, () -> {
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

      double rotatePower = controller.getRightX() * 0.3;
      double heading = Math.toRadians(gyro.getAngle());

      if (fieldCentricEntry.getBoolean(true)) {
        return OmniSpeeds.fromRelative(magnitude, angleRadians, rotatePower, heading);
      } else {
        return OmniSpeeds.from(magnitude, angleRadians, rotatePower, heading);
      }
    });
  }

  /**
   * Creates a new {@link ArmSubsystem} that moves based off the xbox controller triggers.
   * Left trigger should raise the arm, while the right trigger should lower it.
   * @return The newly created {@link ArmSubsystem}
   */
  private ArmSubsystem createArmSubsystem() {
    CANSparkMax leftArmController = new CANSparkMax(Ports.LEFT_ARM_MOTOR_ID, MotorType.kBrushless);
    CANSparkMax rightArmController = new CANSparkMax(Ports.RIGHT_ARM_MOTOR_ID, MotorType.kBrushless);

    return new ArmSubsystem(leftArmController, rightArmController, () -> controller.getLeftTriggerAxis() - controller.getRightTriggerAxis());
  }

  /**
   * Creates a new {@link OuttakeSubsystem} that outtakes based on the left bumper
   * @return The newly created {@link OuttakeSubsystem}
   */
  private OuttakeSubsystem createOuttakeSubsystem() {
    PWMSparkMax leftMotorController = new PWMSparkMax(Ports.LEFT_OUTTAKE_MOTOR_CHANNEL);
    PWMSparkMax rightMotorController = new PWMSparkMax(Ports.RIGHT_OUTTAKE_MOTOR_CHANNEL);

    leftMotorController.addFollower(rightMotorController);

    return new OuttakeSubsystem(leftMotorController);
  }

  /**
   * Creates a new {@link IntakeSubsystem} that intakes based on the right bumper
   * @return The newly created {@link IntakeSubsystem}
   */
  private IntakeSubsystem createIntakeSubsystem() {
    return new IntakeSubsystem(new PWMSparkMax(Ports.INTAKE_MOTOR_CHANNEL));
  }

  private ClimbSubsystem createClimbSubsystem() {
    PWMVictorSPX rightMotorController = new PWMVictorSPX(Ports.RIGHT_CLIMB_MOTOR_CHANNEL);
    PWMVictorSPX leftMotorController = new PWMVictorSPX(Ports.LEFT_CLIMB_MOTOR_CHANNEL);

    return new ClimbSubsystem(rightMotorController, leftMotorController);
  }

  private void configureBindings() {
    controller.leftBumper()
            .whileTrue(shooter.intakeCommand(0.9))
            .onFalse(shooter.intakeCommand(-0.5).withTimeout(0.1));

    controller.leftTrigger()
            .onTrue(armSubsystem.setSetpointCommand(ArmSubsystem.Setpoint.INTAKE));

    controller.rightBumper()
            .onTrue(shooter.outtakeNoteCommand());

    controller.rightTrigger()
            .onTrue(armSubsystem.setSetpointCommand(ArmSubsystem.Setpoint.SPEAKER));

    controller.b()
            .onTrue(armSubsystem.setSetpointCommand(ArmSubsystem.Setpoint.AMP));

    controller.y()
            .whileTrue(climbSubsystem.retractCommand());

    controller.x()
            .whileTrue(climbSubsystem.extendCommand());

    // controller.b()
    //       .onTrue(new CenterAprilTagCommand(this, centerPidController, 2));
  }

  /**
   * Adds robot data onto shuffleboard tab {@code tab}
   * @param tab The shuffleboard tab to add robot data to
   */
  private void addRobotData(ShuffleboardTab tab) {
    tab.add("drivetrain", driveTrainSubsystem.getDriveTrain())
            .withPosition(3, 1)
            .withSize(4, 3);

    tab.add("Speed Type", speedTypeChooser)
            .withPosition(2, 0)
            .withSize(2, 1);

    tab.add("auto mode", autonomousChooser)
            .withPosition(6, 0)
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

  /**
   * Returns the angle given a coordinate. This is returned in radians.
   * @param x The x value of the coordinate
   * @param y The y value of the coordinate
   * @return The angle in radians
   */
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

  public enum AutonomousMode {
    LEFT_SPEAKER,
    RIGHT_SPEAKER,
    MIDDLE_SPEAKER,
    AMP,
    NONE,
    MOVE_FOWARD,
    ;
  }
}
