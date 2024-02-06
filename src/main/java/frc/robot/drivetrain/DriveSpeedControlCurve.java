package frc.robot.drivetrain;

import com.datasiqn.robotutils.controlcurve.ControlCurve;
import com.datasiqn.robotutils.controlcurve.ControlCurves;

public class DriveSpeedControlCurve {
  public static final SpeedType DEFAULT_SPEED_TYPE = SpeedType.CUBIC;
  public static final double DEFAULT_MAX_SPEED = 0.5;

  private final double deadZone;
  private final double minPower;

  private ControlCurve controlCurve;
  private SpeedType speedType = DEFAULT_SPEED_TYPE;
  private double maxSpeed = DEFAULT_MAX_SPEED;

  public DriveSpeedControlCurve(double deadZone, double minPower) {
    this.deadZone = deadZone;
    this.minPower = minPower;

    recreateControlCurve();
  }

  public double getMaxSpeed() {
    return maxSpeed;
  }

  public void setMaxSpeed(double maxSpeed) {
    if (this.maxSpeed != maxSpeed) {
      this.maxSpeed = maxSpeed;

      recreateControlCurve();
    }
  }

  public SpeedType getSpeedType() {
    return speedType;
  }

  public void setSpeedType(SpeedType speedType) {
    if (this.speedType != speedType) {
      this.speedType = speedType;

      recreateControlCurve();
    }
  }

  private void recreateControlCurve() {
    controlCurve = switch (speedType) {
      case CUBIC -> ControlCurves.power(3)
              .withDeadZone(deadZone)
              .withMinimumPower(minPower)
              .withPowerMultiplier(maxSpeed)
              .build();
      case LINEAR -> ControlCurves.linear()
              .withDeadZone(deadZone)
              .withMinimumPower(minPower)
              .withPowerMultiplier(maxSpeed)
              .build();
    };
  }

  public ControlCurve getControlCurve() {
    return controlCurve;
  }

  public enum SpeedType {
    CUBIC("Cubic"),
    LINEAR("Linear"),
    ;

    private final String name;

    SpeedType(String name) {
      this.name = name;
    }

    @Override
    public String toString() {
      return name;
    }
  }
}
