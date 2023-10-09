// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveDriveStopCommand;
import frc.robot.subsystems.SwerveDriveBase;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDriveBase swerveDriveBase = new SwerveDriveBase();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverJoystick =
      new Joystick(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setDefaultCommands();
  }

  private void setDefaultCommands() {
    swerveDriveBase.setDefaultCommand(
      new SwerveDriveCommand(swerveDriveBase,
          () -> -driverJoystick.getRawAxis(OperatorConstants.STRAFE_X_AXIS),
          () -> -driverJoystick.getRawAxis(OperatorConstants.STRAFE_Y_AXIS),
          () -> -driverJoystick.getRawAxis(OperatorConstants.ROTATION_AXIS),
          DriveConstants.FIELD_CENTRIC, DriveConstants.CLOSED_LOOP));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

  public void setAutonDefaultCommands() {
    swerveDriveBase.setBrakingMode(IdleMode.kBrake);
    swerveDriveBase.setDefaultCommand(new SwerveDriveStopCommand(swerveDriveBase));
  }

}
