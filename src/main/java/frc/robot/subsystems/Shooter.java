// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final TalonSRX auxiliaryMotor;
  private final TalonSRX mainMotor;

  /**
   * Constructs a Shooter with main and auxiliary motors controlled by
   * {@link WPI_TalonSRX}es.
   * 
   * @param mainMotorID      The ID of the main motor controller.
   * @param auxiliaryMotorID The ID of the auxiliary motor controller.
   */
  public Shooter(int mainMotorID, int auxiliaryMotorID) {
    this.auxiliaryMotor = new TalonSRX(auxiliaryMotorID);
    this.mainMotor = new TalonSRX(mainMotorID);
    this.auxiliaryMotor.setNeutralMode(NeutralMode.Brake);
    this.mainMotor.setNeutralMode(NeutralMode.Brake);
    this.auxiliaryMotor.configContinuousCurrentLimit(ShooterConstants.CURRENT_LIMIT);
    this.mainMotor.configContinuousCurrentLimit(ShooterConstants.CURRENT_LIMIT);
  }

  /**
   * Drives the auxiliary motor.
   * 
   * @param auxiliarySpeed The speed at which to drive the motor; in [-1, 1].
   */
  public void auxiliaryDrive(double auxiliarySpeed) {
    auxiliaryMotor.set(TalonSRXControlMode.Velocity, auxiliarySpeed);
  }

  /**
   * Drives the main motor.
   * 
   * @param mainSpeed The speed at which to drive the motor; in [-1, 1].
   */
  public void mainDrive(double mainSpeed) {
    mainMotor.set(TalonSRXControlMode.Velocity, mainSpeed);
  }

  /**
   * Stops the main motor.
   */
  public void stopMain() {
    mainMotor.set(TalonSRXControlMode.Disabled, 0);
  }

  /**
   * Stops the auxiliary motor.
   */
  public void stopAuxiliary() {
    auxiliaryMotor.set(TalonSRXControlMode.Disabled, 0);
  }
}
