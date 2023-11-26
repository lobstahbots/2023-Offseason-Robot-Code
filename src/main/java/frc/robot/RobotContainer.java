// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModuleReal;
import frc.robot.subsystems.SwerveModuleSim;
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
  private final DriveBase driveBase;

  private final Shooter shooter = new Shooter(ShooterConstants.MAIN_MOTOR_ID, ShooterConstants.AUXILIARY_MOTOR_ID);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverJoystick =
      new Joystick(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(Robot.isReal()) {
      SwerveModuleReal frontLeft = new SwerveModuleReal(FrontLeftModuleConstants.moduleID, FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID, FrontLeftModuleConstants.angleOffset, FrontLeftModuleConstants.inverted);
      SwerveModuleReal frontRight = new SwerveModuleReal(FrontRightModuleConstants.moduleID, FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID, FrontRightModuleConstants.angleOffset, FrontRightModuleConstants.inverted);
      SwerveModuleReal backLeft = new SwerveModuleReal(BackLeftModuleConstants.moduleID, BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID, BackLeftModuleConstants.angleOffset, BackLeftModuleConstants.inverted);
      SwerveModuleReal backRight = new SwerveModuleReal(BackRightModuleConstants.moduleID, BackRightModuleConstants.angleID, BackRightModuleConstants.driveID, BackRightModuleConstants.angleOffset, BackRightModuleConstants.inverted);
  
      driveBase = new DriveBase(new NavXGyro(), frontLeft, frontRight, backRight, backLeft, false);
    } else {
      SwerveModuleSim frontLeft = new SwerveModuleSim();
      SwerveModuleSim frontRight = new SwerveModuleSim();
      SwerveModuleSim backLeft = new SwerveModuleSim();
      SwerveModuleSim backRight = new SwerveModuleSim();

      driveBase = new DriveBase(new GyroIO(){}, frontLeft, frontRight, backLeft, backRight, false);
    }
   
    setTeleopDefaultCommands();
  }

  private void setTeleopDefaultCommands() {
    driveBase.setDefaultCommand(
      new SwerveDriveCommand(driveBase,
          () -> -driverJoystick.getRawAxis(OperatorConstants.STRAFE_X_AXIS),
          () -> -driverJoystick.getRawAxis(OperatorConstants.STRAFE_Y_AXIS),
          () -> -driverJoystick.getRawAxis(OperatorConstants.ROTATION_AXIS),
          DriveConstants.FIELD_CENTRIC));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return trajectoryFactory.getPathFindToPoseCommand(PathConstants.TARGET_POSE);
  }

  public void setAutonDefaultCommands() {
    driveBase.setBrakingMode(IdleMode.kBrake);
  }

}
