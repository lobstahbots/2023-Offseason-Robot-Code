// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDriveWheel extends SubsystemBase {
  private WPI_TalonFX angleMotor;
  private WPI_TalonFX speedMotor;
  private PIDController pidController;
  /** Creates a new SwerveDriveWheel. */
  public SwerveDriveWheel (int angleMotorID, int speedMotorID) {
    this.angleMotor = new WPI_TalonFX (angleMotorID);
    this.speedMotor = new WPI_TalonFX (speedMotorID);
    pidController = new PIDController (1, 0, 0);
  
    pidController.setTolerance(5);
  }

  public void drive (double speed, double angle) {
    speedMotor.set(speed);
    double setpoint = angle * (DriveConstants.MAX_VOLTS * 0.5) + (DriveConstants.MAX_VOLTS * 0.5); 
    if (setpoint < 0) {
        setpoint = DriveConstants.MAX_VOLTS + setpoint;
    }
    if (setpoint > DriveConstants.MAX_VOLTS) {
        setpoint = setpoint - DriveConstants.MAX_VOLTS;
    }

    double currentAngle = ticksToDegrees(angleMotor.getSelectedSensorPosition());

    pidController.reset();
    angleMotor.set(pidController.calculate(currentAngle, currentAngle + closestAngle(currentAngle, setpoint)));
}

private static double closestAngle(double a, double b) {
  double dir = b % 360 - a % 360;
  if (Math.abs(dir) > 180.0) {
   dir = -(Math.signum(dir) * 360.0) + dir;
  }
  return dir;
}

public static double ticksToDegrees(double ticks) {
  return ((ticks%2048) / 2048)*360;
}

public SwerveModuleState getState() {
  return new SwerveModuleState(
      speedMotor.getSelectedSensorVelocity(), Rotation2d.fromDegrees(ticksToDegrees(angleMotor.getSelectedSensorPosition())));
}


}

