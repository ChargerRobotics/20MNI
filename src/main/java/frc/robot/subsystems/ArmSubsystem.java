// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Button;

import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Represents the arm on the robot
 */
public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax motorController;
  private final CANSparkMax otherMotorController;

  private Setpoint setpoint = Setpoint.INTAKE;

  private GenericEntry manualMovementEntry;

  /**
   * Creates a new {@code ArmSubsystem} with a motor controller and pid controller
   * @param motorController The motor controller that moves the arm
   */
  public ArmSubsystem(CANSparkMax motorController, CANSparkMax otherMotorController, DoubleSupplier manualSpeedSupplier) {
    this.motorController = motorController;
    this.otherMotorController = otherMotorController;

    setDefaultCommand(run(() -> {
      if (manualMovementEntry.getBoolean(false)) {
        motorController.set(manualSpeedSupplier.getAsDouble());
        otherMotorController.set(-manualSpeedSupplier.getAsDouble());
        return;
      }

      if (setpoint.rest && setpoint.pidController.atSetpoint()) {
        motorController.stopMotor();
        otherMotorController.stopMotor();
        return;
      }

      double speed = MathUtil.clamp(setpoint.pidController.calculate(motorController.getEncoder().getPosition()), -1, 1);
      // double otherSpeed = MathUtil.clamp(setpoint.pidController.calculate(otherMotorController.getEncoder().getPosition()), -1, 1);
      motorController.set(speed);
      // otherMotorController.set(otherSpeed);
      otherMotorController.set(-speed);
    }));
  }

  public void reset() {
    motorController.stopMotor();
    otherMotorController.stopMotor();

    Setpoint.reset();
  }

  public Command setSetpointCommand(Setpoint setpoint) {
    return runOnce(() -> {
      if (this.setpoint == Setpoint.AMP && setpoint == Setpoint.SPEAKER) return;
      Setpoint.reset();
      this.setpoint = setpoint;
    });
  }

  public void moveMotor(double speed) {
    motorController.set(speed);
    otherMotorController.set(-speed);
  }

  public Command unstuckCommand() {
    return Commands.sequence(
      startEnd(() -> moveMotor(0.3), () -> moveMotor(0)).withTimeout(0.1),
      startEnd(() -> motorController.set(-0.3), motorController::stopMotor).withTimeout(0.1),
      startEnd(() -> moveMotor(-0.3), () -> moveMotor(0)).withTimeout(0.1)
    );
  }

  public Setpoint getSetpoint() {
    return setpoint;
  }

  public void addShuffleboardData(ShuffleboardTab tab) {
    Setpoint[] setpoints = Setpoint.values();

    tab.addString("setpoint", () -> setpoint.name())
            .withPosition(2, 0)
            .withSize(3, 1);

    tab.addDouble("encoder", motorController.getEncoder()::getPosition)
            .withPosition(3, 1)
            .withSize(1, 1);

    tab.addDouble("encoder2", otherMotorController.getEncoder()::getPosition) //ADDED BY LOGAN
            .withPosition(4, 1)
            .withSize(1, 1);

    tab.addDouble("motor power", otherMotorController::get)
            .withPosition(2, 1)
            .withSize(1, 1);

    tab.add("reset encoders", new Button("reset encoders", () -> {
      motorController.getEncoder().setPosition(0);
      otherMotorController.getEncoder().setPosition(0);
    }))
            .withPosition(7, 0);

    Command unstuckCommand = unstuckCommand();
            unstuckCommand.setName("unstuck");

    tab.add(unstuckCommand)
            .withPosition(7, 1);

    manualMovementEntry = tab.add("manual movement", false)
            .withPosition(7, 2)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
            
    ShuffleboardLayout setpointLayout = tab.getLayout("change setpoint", BuiltInLayouts.kList)
            .withSize(2, 4);
    for (Setpoint setpoint : setpoints) {
      setpointLayout.add(setpoint.name(), new Button(setpoint.name(), () -> this.setpoint = setpoint));
    }

    ShuffleboardLayout setpointPidList = tab.getLayout("setpoint pid controllers", BuiltInLayouts.kList)
            .withPosition(5, 0)
            .withSize(2, 5);
    for (Setpoint setpoint : setpoints) {
      setpointPidList.add(setpoint.name(), setpoint.pidController);
    }

    tab.addDoubleArray("arm pid graph", () -> new double[] { setpoint.getPidController().getSetpoint(), motorController.getEncoder().getPosition() })
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("visible time", 2))
            .withPosition(2, 2)
            .withSize(3, 3);
  }

  public enum Setpoint {
    SPEAKER(new PIDController(0.06, 0.034, 0), false),
    DOWN_SPEAKER(new PIDController(0.015, 0, 0), false),
    AMP(new PIDController(0.03, 0, 0), true),
    INTAKE(new PIDController(0.007, 0, 0), true),
    NONE(new PIDController(0, 0, 0), true),
    ;

    static {
      SPEAKER.pidController.setSetpoint(-7.5);
      SPEAKER.pidController.setIntegratorRange(-0.2, 0.2);

      DOWN_SPEAKER.pidController.setSetpoint(-7.5);
      DOWN_SPEAKER.pidController.setIntegratorRange(-0.2, 0.2);
      DOWN_SPEAKER.pidController.setTolerance(1);

      AMP.pidController.setSetpoint(-22.78);
      AMP.pidController.setIntegratorRange(-0.2, 0.2);
      AMP.pidController.setTolerance(0.2);

      INTAKE.pidController.setSetpoint(0);
      INTAKE.pidController.setIntegratorRange(-0.2, 0.2);
    }

    private final PIDController pidController;
    private final boolean rest;

    Setpoint(PIDController pidController, boolean rest) {
      this.pidController = pidController;
      this.rest = rest;
    }

    public PIDController getPidController() {
      return pidController;
    }

    public static void reset() {
      for (Setpoint setpoint : values()) {
        setpoint.pidController.reset();
      }
    }
  }
}
