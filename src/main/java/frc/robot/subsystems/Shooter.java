// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private ShooterIO io;

  /**
   * Constructs a Shooter with main and auxiliary motors controlled by
   * {@link TalonSRX}es.
   * 
   * @param io The {@link ShooterIO} object.
   */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  /**
   * Drives the auxiliary motor.
   * 
   * @param auxiliarySpeed The speed at which to drive the motor; in [-1, 1].
   */
  public void auxiliaryDrive(double auxiliarySpeed) {
    io.auxiliaryDrive(auxiliarySpeed);
  }

  /**
   * Drives the main motor.
   * 
   * @param mainSpeed The speed at which to drive the motor; in [-1, 1].
   */
  public void mainDrive(double mainSpeed) {
    io.mainDrive(mainSpeed);
  }

  /**
   * Stops the main motor.
   */
  public void stopMain() {
    io.stopMain();
  }

  /**
   * Stops the auxiliary motor.
   */
  public void stopAuxiliary() {
    io.stopAuxiliary();
  }

   /**
   * Get the voltage applied to the main motor.
   * @return The applied voltage (in volts).
   */
  public double getMainAppliedVolts() {
    return io.getMainAppliedVolts();
  }

  /**
   * Get the current output from the main motor controller.
   * @return The output/stator current (in amps).
   */
  public double getMainCurrentAmps() {
    return io.getMainCurrentAmps();
  }

  /**
   * Get the voltage applied to the auxiliary motor.
   * @return The applied voltage (in volts).
   */
  public double getAuxiliaryAppliedVolts() {
    return io.getAuxiliaryAppliedVolts();
  }

  /**
   * Get the current output from the auxiliary motor controller.
   * @return The output/stator current (in amps).
   */
  public double getAuxiliaryCurrentAmps() {
    return io.getAuxiliaryCurrentAmps();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
