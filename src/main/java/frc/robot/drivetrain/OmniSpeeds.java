// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

/**
 * Represents motor speed values for an {@link OmniDriveTrain}. This includes the robot heading and all the powers for each chassis.
 * <p>
 * In most cases, you shouldn't need to create an instance of this record.
 * Rather, use the static factory methods like {@link #from(double, double, double, double) from} and {@link #fromRelative(double, double, double, double) fromRelative}.
 * <p>
 * <strong>P.S.</strong>
 * <p>
 * If you're confused as to what the heck a record is, see <a href="https://www.geeksforgeeks.org/what-are-java-records-and-how-to-use-them-alongside-constructors-and-methods/">here</a>.
 */
public record OmniSpeeds(double heading, double leftPower, double topPower, double rightPower, double bottomPower) {
  /**
   * Creates a new {@code OmniSpeeds} instance that isn't using field-centric driving.
   * To create one with field-centric drive, use {@link #fromRelative(double, double, double, double) fromRelative} instead.
   * @param magnitude The joystick magnitude
   * @param angleRadians The angle of the joystick
   * @param rotatePower The rotation power. This plus the max joystick magnitude should not exceed 1.
   * @param heading The robot heading in radians
   * @return The newly created {@code OmniSpeeds} instance
   */
  public static OmniSpeeds from(double magnitude, double angleRadians, double rotatePower, double heading) {
    double forwardPower = Math.cos(angleRadians) * magnitude;
    double sidewaysPower = Math.sin(angleRadians) * magnitude;

    return new OmniSpeeds(heading, forwardPower + rotatePower, sidewaysPower + rotatePower, forwardPower - rotatePower, sidewaysPower - rotatePower);
  }

  /**
   * Creates a new {@code OmniSpeeds} instance that is using field-centric driving.
   * To create one without field-centric drive, use {@link #from(double, double, double, double) from} instead.
   * @param magnitude Joystick magnitude
   * @param angleRadians The angle of the joystick
   * @param rotatePower The rotation power. This plus the max joystick magnitude should not exceed 1.
   * @param heading The robot heading in radians
   * @return The newly created {@code OmniSpeeds} instance
   */
  public static OmniSpeeds fromRelative(double magnitude, double angleRadians, double rotatePower, double heading) {
    double actualRotation = angleRadians - heading;
    double forwardPower = Math.cos(actualRotation) * magnitude;
    double sidewaysPower = Math.sin(actualRotation) * magnitude;

    return new OmniSpeeds(heading, forwardPower + rotatePower, sidewaysPower + rotatePower, forwardPower - rotatePower, sidewaysPower - rotatePower);
  }

  public static OmniSpeeds zero(double heading) {
    return new OmniSpeeds(heading, 0, 0, 0, 0);
  }
}
