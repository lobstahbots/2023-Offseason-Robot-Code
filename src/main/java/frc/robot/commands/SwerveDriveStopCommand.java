// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveBase;

public class SwerveDriveStopCommand extends CommandBase {
  /** Creates a new SwerveDriveStopCommand. */
  private final SwerveDriveBase swerveDriveBase;
  public SwerveDriveStopCommand(SwerveDriveBase swerveDriveBase) {
    this.swerveDriveBase = swerveDriveBase;
    addRequirements(swerveDriveBase);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDriveBase.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
