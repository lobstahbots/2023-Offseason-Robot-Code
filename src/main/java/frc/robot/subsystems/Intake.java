// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax spinner;
  private final CANSparkMax mover;


  /** Creates a new Intake. */
  public Intake(int spinnerID, int moverID) {
    this.spinner = new CANSparkMax(spinnerID, MotorType.kBrushless);
    this.mover = new CANSparkMax(moverID, MotorType.kBrushless);
    this.spinner.setIdleMode(IdleMode.kBrake);
    this.mover.setIdleMode(IdleMode.kBrake);
    this.spinner.setInverted(true);
    this.mover.setInverted(false);
  }
  /**
   * 
   */
  public void setSpinnerSpeed(double spinnerSpeed) {
    spinner.set(spinnerSpeed);
  }

  public void setMoverSpeed(double moverSpeed) {
    spinner.set(moverSpeed);
  }

  public void stopSpinner() {
    spinner.stopMotor();
  }

  public void stopMover() {
    mover.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
