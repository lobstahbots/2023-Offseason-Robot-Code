// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

/** An example command that uses an example subsystem. */
public class SwerveDriveCommand extends CommandBase {
  private final DriveBase driveBase;
  private final DoubleSupplier strafeXSupplier;
  private final DoubleSupplier strafeYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final boolean fieldCentric;

  public SwerveDriveCommand(DriveBase driveBase, DoubleSupplier strafeXSupplier, DoubleSupplier strafeYSupplier, DoubleSupplier rotationSupplier, boolean fieldCentric) {
    this.driveBase = driveBase;
    this.strafeXSupplier = strafeXSupplier;
    this.strafeYSupplier = strafeYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldCentric = fieldCentric;
    addRequirements(driveBase);
  }

  public SwerveDriveCommand(DriveBase driveBase, double strafeX, double strafeY, double rotation, boolean fieldCentric) {
    this(driveBase, () -> strafeX, () -> strafeY, () -> rotation, fieldCentric);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.drive(new Translation2d(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble()), rotationSupplier.getAsDouble(), fieldCentric);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
