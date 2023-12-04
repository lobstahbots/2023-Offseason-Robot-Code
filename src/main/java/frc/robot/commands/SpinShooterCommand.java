// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpinShooterCommand extends Command {
  private final Shooter shooter;
  private final double mainSpeed;
  private final double auxiliarySpeed;
  /**
   * Command to spin the shooter's ({@link Shooter}) main and auxiliary motor.
   * @param shooter The shooter subsystem.
   * @param mainSpeed The speed for the main motor.
   * @param auxiliarySpeed The speed for the auxiliary motor.
   */
  public SpinShooterCommand(Shooter shooter, double mainSpeed, double auxiliarySpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.mainSpeed = mainSpeed;
    this.auxiliarySpeed = auxiliarySpeed;
    addRequirements(shooter);
  }
  /**
   * Execute the command; called every time the scheduler is run. Sets the speed of both motors to their respective speeds.
   */
  @Override
  public void execute() {
    shooter.mainDrive(mainSpeed);
    shooter.auxiliaryDrive(auxiliarySpeed);
  }
  /**
   * End the command; called when the command terminates. Stops both motors.
   */
  @Override
  public void end(boolean interrupted) {
    shooter.stopAuxiliary();
    shooter.stopMain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
