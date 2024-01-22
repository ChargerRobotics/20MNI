package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivetrain.OmniDriveTrain;
import frc.robot.drivetrain.OmniSpeeds;

import java.util.function.Supplier;

public class DriveTrainSubsystem extends SubsystemBase {
  private final OmniDriveTrain driveTrain;

  public DriveTrainSubsystem(OmniDriveTrain driveTrain, Supplier<OmniSpeeds> speedsSupplier) {
    this.driveTrain = driveTrain;

    setDefaultCommand(run(() -> drive(speedsSupplier.get())));
  }

  public void drive(OmniSpeeds speeds) {
    driveTrain.drive(speeds);
  }

  public OmniDriveTrain getDriveTrain() {
    return driveTrain;
  }
}
