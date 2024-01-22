// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

public record OmniSpeeds(double heading, double leftPower, double topPower, double rightPower, double bottomPower) {
  /**
   * @param magnitude Joystick magnitude
   * @param angleRadians The angle of the joystick
   * @param rotatePower The rotation power. This plus the max joystick magnitude should not exceed 1.
   * @param heading The robot heading in radians
   */
  public static OmniSpeeds fromRelative(double magnitude, double angleRadians, double rotatePower, double heading) {
    double actualRotation = angleRadians - heading;
    double forwardPower = Math.cos(actualRotation) * magnitude;
    double sidewaysPower = Math.sin(actualRotation) * magnitude;

    return new OmniSpeeds(heading, forwardPower + rotatePower, sidewaysPower + rotatePower, forwardPower - rotatePower, sidewaysPower - rotatePower);
  }
}
