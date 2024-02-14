package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivetrain.OmniDriveTrain;
import frc.robot.drivetrain.OmniSpeeds;

import java.util.function.Supplier;

/**
 * Represents the drive train
 */
public class DriveTrainSubsystem extends SubsystemBase {
  private final OmniDriveTrain driveTrain;

  /**
   * Creates a new {@code DriveTrainSubsystem} with a drive train and a speeds supplier
   * @param driveTrain The drive train to use
   * @param speedsSupplier A supplier that supplies the speeds of the drive train.
   * The output of this function will be fed directly into {@link OmniDriveTrain#drive(OmniSpeeds)}
   */
  public DriveTrainSubsystem(OmniDriveTrain driveTrain, Supplier<OmniSpeeds> speedsSupplier) {
    this.driveTrain = driveTrain;

    setDefaultCommand(run(() -> drive(speedsSupplier.get())));
  }

  /**
   * Drives this drive train.
   * <p>
   * <strong>NOTE</strong>
   * <p>
   * This will almost always be overriden by the default command of this subsystem.
   * Make sure that some other command is using this subsystem to make sure the default command doesn't get run.
   * @param speeds The speeds to drive at
   */
  public void drive(OmniSpeeds speeds) {
    driveTrain.drive(speeds);
  }

  public OmniDriveTrain getDriveTrain() {
    return driveTrain;
  }
}
