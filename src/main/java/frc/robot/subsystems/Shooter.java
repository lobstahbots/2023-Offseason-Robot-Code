// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax auxiliaryMotor;
  private final CANSparkMax mainMotor;

  public Shooter(int mainMotorID, int auxiliaryMotorID) {
    this.auxiliaryMotor = new CANSparkMax(auxiliaryMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    this.mainMotor = new CANSparkMax(mainMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    this.auxiliaryMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.mainMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void auxiliaryDrive(double auxiliarySpeed) {
    auxiliaryMotor.set(auxiliarySpeed);
  }

  public void mainDrive(double mainSpeed) {
    mainMotor.set(mainSpeed);
  }

  public void drive(double speed) {
    auxiliaryDrive(speed);
    mainDrive(speed);
  }

  public void stopDrive() {
    drive(0);
  }
}
