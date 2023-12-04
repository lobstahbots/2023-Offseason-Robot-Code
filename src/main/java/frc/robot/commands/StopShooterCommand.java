// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopShooterCommand extends Command {
  private final Shooter shooter;
  /**
   * Command to stop the shooter.
   * @param shooter The shooter subsystem.
   */
  public StopShooterCommand(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);
  }

  /**
   * Execute the command; called every time the scheduler is run. Stops both motors.
   */
  @Override
  public void execute() {
    shooter.stopAuxiliary();
    shooter.stopMain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
