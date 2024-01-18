// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

public class OmniSpeeds {
  private final double forwardPower;
  private final double sidewaysPower;

  /**
   * @param forwardPower The power power
   * @param sidewaysPower The sidepways power
   * @param rotatePower The rotate power. This plus either the max forward or sideways power should not exceed 1.
   * @param heading The robot heading in radians
   */
  public OmniSpeeds(double forwardPower, double sidewaysPower) {
    this.forwardPower = forwardPower;
    this.sidewaysPower = sidewaysPower;
  }

  /**
   * @param magnitude Joystick magnitude
   * @param angleRadians The angle of the joystick
   * @param rotatePower The rotate power. This plus the max joystick magnitude should not exceed 1.
   * @param heading The robot heading in radians
   */
  public static OmniSpeeds fromRelative(double magnitude, double angleRadians, double rotatePower, double heading) {
    double actualRotation = angleRadians - heading;
    double forwardPower = Math.cos(actualRotation) * magnitude + rotatePower;
    double sidewaysPower = Math.sin(actualRotation) * magnitude + rotatePower;

    return new OmniSpeeds(forwardPower, sidewaysPower);
  }

  public double getForwardPower() {
    return forwardPower;
  }

  public double getSidewaysPower() {
    return sidewaysPower;
  }
}
