// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

public class OmniSpeeds {
  private final double leftPower;
  private final double topPower;
  private final double rightPower;
  private final double bottomPower;

  public OmniSpeeds(double forwardPower, double sidewaysPower) {
    this.leftPower = forwardPower;
    this.rightPower = forwardPower;
    this.topPower = sidewaysPower;
    this.bottomPower = sidewaysPower;
  }
  public OmniSpeeds(double leftPower, double topPower, double rightPower, double bottomPower) {
    this.leftPower = leftPower;
    this.topPower = topPower;
    this.rightPower = rightPower;
    this.bottomPower = bottomPower;
  }

  /**
   * @param magnitude Joystick magnitude
   * @param angleRadians The angle of the joystick
   * @param rotatePower The rotate power. This plus the max joystick magnitude should not exceed 1.
   * @param heading The robot heading in radians
   */
  public static OmniSpeeds fromRelative(double magnitude, double angleRadians, double rotatePower, double heading) {
    double actualRotation = angleRadians - heading;
    double forwardPower = Math.cos(actualRotation) * magnitude;
    double sidewaysPower = Math.sin(actualRotation) * magnitude;

    return new OmniSpeeds(forwardPower + rotatePower, sidewaysPower + rotatePower, forwardPower - rotatePower, sidewaysPower - rotatePower););
  }

  public double getLeftPower() {
    return leftPower;
  }

  public double getTopPower() {
    return topPower;
  }

  public double getRightPower() {
    return rightPower;
  }

  public double getBottomPower() {
    return bottomPower;
  }
}
