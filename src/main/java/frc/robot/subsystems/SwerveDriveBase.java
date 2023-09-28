// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class SwerveDriveBase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SwerveDriveWheel frontRight;
  private final SwerveDriveWheel frontLeft;
  private final SwerveDriveWheel backRight;
  private final SwerveDriveWheel backLeft;

  public SwerveDriveBase(int frontRightID, int frontRightAngleID, int frontLeftID, int frontLeftAngleID, int backRightID, int backRightAngleID, int backLeftID, int backLeftAngleID) {
    this.frontLeft = new SwerveDriveWheel(frontLeftID, frontLeftAngleID);
    this.frontRight = new SwerveDriveWheel(frontRightID, frontRightAngleID);
    this.backLeft = new SwerveDriveWheel(backLeftID, backLeftAngleID);
    this.backRight = new SwerveDriveWheel(backRightID, backRightAngleID);
  }

  public void drive (double strafeX, double strafeY, double rotationX) {
    strafeY *= -1;

    double a = strafeX - rotationX * (RobotConstants.LENGTH / RobotConstants.RADIUS);
    double b = strafeX + rotationX * (RobotConstants.LENGTH / RobotConstants.RADIUS);
    double c = strafeX - rotationX * (RobotConstants.WIDTH / RobotConstants.RADIUS);
    double d = strafeX + rotationX * (RobotConstants.WIDTH/ RobotConstants.RADIUS);

    double backRightSpeed = Math.sqrt ((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

    double backRightAngle = Math.atan2 (a, d) / Math.PI;
    double backLeftAngle = Math.atan2 (a, c) / Math.PI;
    double frontRightAngle = Math.atan2 (b, d) / Math.PI;
    double frontLeftAngle = Math.atan2 (b, c) / Math.PI;

    frontLeft.drive(frontLeftSpeed, frontLeftAngle);
    frontRight.drive(frontRightSpeed, frontRightAngle);
    backLeft.drive(backLeftSpeed, backLeftAngle);
    backRight.drive(backRightSpeed, backRightAngle);
  }
}
