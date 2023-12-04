// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax roller;
  private final CANSparkMax mover;


  /** Creates a new Intake. */
  public Intake(int rollerID, int moverID) {
    this.roller = new CANSparkMax(rollerID, MotorType.kBrushless);
    this.mover = new CANSparkMax(moverID, MotorType.kBrushless);
    this.roller.setIdleMode(IdleMode.kBrake);
    this.mover.setIdleMode(IdleMode.kBrake);
    this.roller.setInverted(true);
    this.mover.setInverted(false);
  }
  /**
   * Sets the spin speed of the intake rollers.
   * @param rollerSpeed The speed to spin them at.
   */
  public void setRollerSpeed(double rollerSpeed) {
    roller.set(rollerSpeed);
  }

  /**
   * Sets the speed of the intake mover motor.
   * @param moverSpeed The speed for the mover to move with.
   */
  public void setMoverSpeed(double moverSpeed) {
    roller.set(moverSpeed);
  }

  /**
   * Stops the roller motor.
   */
  public void stopRoller() {
    roller.stopMotor();
  }

  /**Stops the mover motor. */
  public void stopMover() {
    mover.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
