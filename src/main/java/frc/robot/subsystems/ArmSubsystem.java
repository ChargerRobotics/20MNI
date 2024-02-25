// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Button;

import java.util.Map;

/**
 * Represents the arm on the robot
 */
public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax motorController;
  private final CANSparkMax otherMotorController;

  private Setpoint setpoint = Setpoint.INTAKE;

  /**
   * Creates a new {@code ArmSubsystem} with a motor controller and pid controller
   * @param motorController The motor controller that moves the arm
   */
  public ArmSubsystem(CANSparkMax motorController, CANSparkMax otherMotorController) {
    this.motorController = motorController;
    this.otherMotorController = otherMotorController;

    setDefaultCommand(run(() -> {
      if (setpoint == Setpoint.INTAKE && setpoint.pidController.atSetpoint()) {
        motorController.stopMotor();
        otherMotorController.stopMotor();
        return;
      }

      double speed = MathUtil.clamp(setpoint.pidController.calculate(motorController.getEncoder().getPosition()), -1, 1);
      motorController.set(speed);
      otherMotorController.set(-speed);
    }));
  }

  public void reset() {
    motorController.getEncoder().setPosition(0);

    motorController.stopMotor();
    otherMotorController.stopMotor();

    Setpoint.reset();
  }

  public Command setSetpointCommand(Setpoint setpoint) {
    return runOnce(() -> {
      Setpoint.reset();
      this.setpoint = setpoint;
    });
  }

  public void addShuffleboardData(ShuffleboardTab tab) {
    Setpoint[] setpoints = Setpoint.values();
    double encoderPosition = motorController.getEncoder().getPosition();

    tab.addString("setpoint", () -> setpoint.name())
            .withPosition(2, 0)
            .withSize(3, 1);

    tab.addDouble("encoder", () -> encoderPosition)
            .withPosition(2, 1)
            .withSize(3, 1);

    tab.addDouble("motor power", motorController::get)
            .withPosition(2, 1)
            .withSize(2, 1);

    ShuffleboardLayout setpointLayout = tab.getLayout("change setpoint", BuiltInLayouts.kList)
            .withSize(2, 2);
    for (Setpoint setpoint : setpoints) {
      setpointLayout.add(setpoint.name(), new Button(setpoint.name(), () -> this.setpoint = setpoint));
    }

    ShuffleboardLayout setpointPidList = tab.getLayout("setpoint pid controllers", BuiltInLayouts.kList)
            .withPosition(5, 0)
            .withSize(2, 5);
    for (Setpoint setpoint : setpoints) {
      setpointPidList.add(setpoint.name(), setpoint.pidController);
    }

    tab.addDoubleArray("arm pid graph", () -> new double[] { setpoint.getPidController().getSetpoint(), encoderPosition})
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("visible time", 2))
            .withPosition(2, 2)
            .withSize(3, 3);
  }

  public enum Setpoint {
    SPEAKER(new PIDController(0, 0, 0)),
    AMP(new PIDController(0, 0, 0)),
    INTAKE(new PIDController(0, 0, 0)),
    ;

    static {
      SPEAKER.pidController.setSetpoint(-6.5);
      SPEAKER.pidController.setIntegratorRange(-0.2, 0.2);

      AMP.pidController.setSetpoint(-12);
      AMP.pidController.setIntegratorRange(-0.2, 0.2);

      INTAKE.pidController.setSetpoint(0);
      INTAKE.pidController.setIntegratorRange(-0.2, 0.2);
    }

    private final PIDController pidController;

    Setpoint(PIDController pidController) {
      this.pidController = pidController;
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
