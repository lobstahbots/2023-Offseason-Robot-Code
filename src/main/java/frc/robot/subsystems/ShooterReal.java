// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.ShooterConstants;

public class ShooterReal implements ShooterIO {
  /** Creates a new Shooter. */
  private final TalonFX auxiliaryMotor;
  private final TalonFX mainMotor;

  /**
   * Constructs a Shooter for real cases.
   * 
   * @param mainMotorID      The ID of the main motor controller.
   * @param auxiliaryMotorID The ID of the auxiliary motor controller.
   */
  public ShooterReal(int mainMotorID, int auxiliaryMotorID) {
    this.auxiliaryMotor = new TalonFX(auxiliaryMotorID);
    this.mainMotor = new TalonFX(mainMotorID);
    this.auxiliaryMotor.setNeutralMode(NeutralModeValue.Brake);
    this.mainMotor.setNeutralMode(NeutralModeValue.Brake);
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
    mainMotor.set(0);
  }

  /**
   * Stops the auxiliary motor.
   */
  public void stopAuxiliary() {
    auxiliaryMotor.set(0);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.mainAppliedVolts = mainMotor.getSupplyVoltage().getValueAsDouble();
    inputs.mainCurrentAmps = mainMotor.getSupplyVoltage().getValueAsDouble();

    inputs.auxiliaryAppliedVolts = auxiliaryMotor.getMotorVoltage().getValueAsDouble();
    inputs.auxiliaryCurrentAmps = auxiliaryMotor.getMotorVoltage().getValueAsDouble();
  }
}
