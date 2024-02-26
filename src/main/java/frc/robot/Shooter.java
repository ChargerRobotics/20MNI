// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class Shooter {
  private final IntakeSubsystem intakeSubsystem;
  private final OuttakeSubsystem outtakeSubsystem;

  public Shooter(IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.outtakeSubsystem = outtakeSubsystem;
  }

  public Command outtakeNoteCommand() {
    return Commands.deadline(
      Commands.waitSeconds(1).andThen(intakeCommand(-1).withTimeout(0.2)),
      outtakeCommand(-0.7)
    );
  }

  public Command intakeCommand(double speed) {
    return intakeSubsystem.intakeCommand(speed);
  }

  public Command outtakeCommand(double speed) {
    return outtakeSubsystem.outtakeCommand(speed);
  }

  public IntakeSubsystem getIntake() {
    return intakeSubsystem;
  }

  public OuttakeSubsystem getOuttake() {
    return outtakeSubsystem;
  }

  public void addShuffleboardData(ShuffleboardTab tab) {
    ShuffleboardLayout intakeList = tab.getLayout("Intake", BuiltInLayouts.kList)
          .withSize(2, 3)
          .withPosition(2, 0);

    intakeList.addDouble("motor speed", () -> intakeSubsystem.getMotorController().get());
    Command intake = intakeCommand(-1);
    intake.setName("intake");
    intakeList.add(intake);
    Command eject = intakeCommand(1);
    eject.setName("eject");
    intakeList.add(eject);

    ShuffleboardLayout outtakeList = tab.getLayout("Outtake", BuiltInLayouts.kList)
          .withSize(2, 3)
          .withPosition(4, 0);

    outtakeList.addDouble("motor speed", () -> outtakeSubsystem.getMotorController().get());
    Command outtake = outtakeCommand(-0.7);
    outtake.setName("outtake");
    outtakeList.add(outtake);
  }
}
