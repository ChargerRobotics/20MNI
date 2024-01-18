package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivetrain.OmniDriveTrain;
import frc.robot.drivetrain.OmniSpeeds;

public class DriveTrainSubsystem extends SubsystemBase {
  private final OmniDriveTrain driveTrain;
  private final Supplier<OmniSpeeds> speedsSupplier;

  public DriveTrainSubsystem(OmniDriveTrain driveTrain, Supplier<OmniSpeeds> speedsSupplier) {
    this.driveTrain = driveTrain;
    this.speedsSupplier = speedsSupplier;

    setDefaultCommand(run(() -> drive(speedsSupplier.get())));
  }

  public void drive(OmniSpeeds speeds) {
    driveTrain.drive(speeds);
  }

  public OmniSpeeds getSpeeds() {
    return speedsSupplier.get();
  }
}
