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

  /**
   * Constructs a Shooter with main and auxiliary motors controlled by
   * {@link CANSparkMax}es.
   * 
   * @param mainMotorID      The ID of the main motor controller.
   * @param auxiliaryMotorID The ID of the auxiliary motor controller.
   */
  public Shooter(int mainMotorID, int auxiliaryMotorID) {
    this.auxiliaryMotor = new CANSparkMax(auxiliaryMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    this.mainMotor = new CANSparkMax(mainMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    this.auxiliaryMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.mainMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /**
   * Drives the auxiliary motor.
   * 
   * @param auxiliarySpeed The speed at which to drive the motor; in [-1, 1].
   */
  public void auxiliaryDrive(double auxiliarySpeed) {
    auxiliaryMotor.set(auxiliarySpeed);
  }

  /**
   * Drives the main motor.
   * 
   * @param mainSpeed The speed at which to drive the motor; in [-1, 1].
   */
  public void mainDrive(double mainSpeed) {
    mainMotor.set(mainSpeed);
  }

  /**
   * Stops the main motor.
   */
  public void stopMain() {
    mainMotor.stopMotor();
  }

  /**
   * Stops the auxiliary motor.
   */
  public void stopAuxiliary() {
    auxiliaryMotor.stopMotor();
  }
}
