// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.TrajectoryFactory.PathType;
import frc.robot.commands.SpinShooterCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterReal;
import frc.robot.subsystems.SwerveModuleReal;
import frc.robot.subsystems.SwerveModuleSim;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveBase driveBase;

  // private final Shooter shooter;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverJoystick =
      new Joystick(IOConstants.DRIVER_CONTROLLER_PORT);
  
  private final CommandJoystick operatorJoystick = new CommandJoystick(IOConstants.OPERATOR_CONTROLLER_PORT);

  private final TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(Robot.isReal()) {
      SwerveModuleReal frontLeft = new SwerveModuleReal(FrontLeftModuleConstants.moduleID, FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID, FrontLeftModuleConstants.angleOffset, FrontLeftModuleConstants.inverted);
      SwerveModuleReal frontRight = new SwerveModuleReal(FrontRightModuleConstants.moduleID, FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID, FrontRightModuleConstants.angleOffset, FrontRightModuleConstants.inverted);
      SwerveModuleReal backLeft = new SwerveModuleReal(BackLeftModuleConstants.moduleID, BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID, BackLeftModuleConstants.angleOffset, BackLeftModuleConstants.inverted);
      SwerveModuleReal backRight = new SwerveModuleReal(BackRightModuleConstants.moduleID, BackRightModuleConstants.angleID, BackRightModuleConstants.driveID, BackRightModuleConstants.angleOffset, BackRightModuleConstants.inverted);
  
      driveBase = new DriveBase(new NavXGyro(), frontLeft, frontRight, backRight, backLeft, false);
    } 
    else {
      SwerveModuleSim frontLeft = new SwerveModuleSim(FrontLeftModuleConstants.angleOffset);
      SwerveModuleSim frontRight = new SwerveModuleSim(FrontRightModuleConstants.angleOffset);
      SwerveModuleSim backLeft = new SwerveModuleSim(BackLeftModuleConstants.angleOffset);
      SwerveModuleSim backRight = new SwerveModuleSim(BackRightModuleConstants.angleOffset);

      driveBase = new DriveBase(new GyroIO(){}, frontLeft, frontRight, backLeft, backRight, false);
    }

    // ShooterReal shooterReal = new ShooterReal(ShooterConstants.MAIN_MOTOR_ID, ShooterConstants.AUXILIARY_MOTOR_ID);
    // shooter = new Shooter(shooterReal);
   
    setTeleopDefaultCommands();
  }

  private void setTeleopDefaultCommands() {
    driveBase.setDefaultCommand(
      new SwerveDriveCommand(driveBase,
          () -> driverJoystick.getRawAxis(IOConstants.STRAFE_Y_AXIS),
          () -> -driverJoystick.getRawAxis(IOConstants.STRAFE_X_AXIS),
          () -> driverJoystick.getRawAxis(IOConstants.ROTATION_AXIS),
          DriveConstants.FIELD_CENTRIC));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(trajectoryFactory.getPathFindToPathCommand("Path", PathType.PATHPLANNER));
  }

  public void setAutonDefaultCommands() {
    driveBase.setBrakingMode(IdleMode.kBrake);
  }

  public void configureButtonBindings() {
    // Trigger mainWheelTrigger = operatorJoystick.button(OperatorConstants.SHOOTER_MAIN_BUTTON_ID);
    // Trigger auxiliaryWheelTrigger = operatorJoystick.button(OperatorConstants.SHOOTER_AUXILIARY_BUTTON_ID);
    // mainWheelTrigger.and(auxiliaryWheelTrigger).whileTrue(
    //   new SpinShooterCommand(shooter, ShooterConstants.MAIN_MOTOR_SPEED, ShooterConstants.AUXILIARY_MOTOR_SPEED));
    // mainWheelTrigger.and(auxiliaryWheelTrigger.negate()).whileTrue(
    //   new SpinShooterCommand(shooter, ShooterConstants.MAIN_MOTOR_SPEED, 0));
    // mainWheelTrigger.negate().and(auxiliaryWheelTrigger).whileTrue(
    //   new SpinShooterCommand(shooter, 0, ShooterConstants.AUXILIARY_MOTOR_SPEED));
  }

}
