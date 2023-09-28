// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveBase;

/** An example command that uses an example subsystem. */
public class SwerveDriveCommand extends CommandBase {
  private final SwerveDriveBase swerveDriveBase;
  private final DoubleSupplier strafeXSupplier;
  private final DoubleSupplier strafeYSupplier;
  private final DoubleSupplier rotationSupplier;

  public SwerveDriveCommand(SwerveDriveBase swerveDriveBase, double strafeX, double strafeY, double rotation) {
    this(swerveDriveBase, () -> strafeX, () -> strafeY, () -> rotation);
  }

  public SwerveDriveCommand(SwerveDriveBase swerveDriveBase, DoubleSupplier strafeXSupplier, DoubleSupplier strafeYSupplier, DoubleSupplier rotationSupplier) {
    this.swerveDriveBase = swerveDriveBase;
    this.strafeXSupplier = strafeXSupplier;
    this.strafeYSupplier = strafeYSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(swerveDriveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDriveBase.drive(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble(), rotationSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
