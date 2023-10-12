// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;

public class SwerveDriveBase extends SubsystemBase {
  /** Creates a new SwerveDriveBase. */
  private final SwerveDriveModule frontRight;
  private final SwerveDriveModule frontLeft;
  private final SwerveDriveModule backRight;
  private final SwerveDriveModule backLeft;
  private final SwerveDriveModule[] modules;

  private SwerveDriveOdometry swerveOdometry;
  private final AHRS gyro = new AHRS();
  private Field2d field;

  public SwerveDriveBase() {
    this.frontLeft = new SwerveDriveModule(FrontLeftModuleConstants.moduleID, FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID, FrontLeftModuleConstants.angleOffset);
    this.frontRight = new SwerveDriveModule(FrontRightModuleConstants.moduleID, FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID, FrontRightModuleConstants.angleOffset);
    this.backLeft = new SwerveDriveModule(BackLeftModuleConstants.moduleID, BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID, BackLeftModuleConstants.angleOffset);
    this.backRight = new SwerveDriveModule(BackRightModuleConstants.moduleID, BackRightModuleConstants.angleID, BackRightModuleConstants.driveID, BackRightModuleConstants.angleOffset);

    modules = new SwerveDriveModule[]{frontLeft, frontRight, backLeft, backRight};

    zeroGyro();
    swerveOdometry = new SwerveDriveOdometry(DriveConstants.KINEMATICS, getYaw(), getPositions(), new Pose2d());
  }
  
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveDriveModule module : modules) {
      states[module.getModuleID()] = module.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveDriveModule module : modules) {
      positions[module.getModuleID()] = module.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public Rotation2d getYaw() {
    return (SwerveConstants.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_DRIVE_SPEED);

    for (SwerveDriveModule module : modules) {
      module.setDesiredState(swerveModuleStates[module.getModuleID()], isOpenLoop);
    }
}

public void setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.MAX_DRIVE_SPEED);
  frontLeft.setDesiredState(desiredStates[0], DriveConstants.IS_OPEN_LOOP);
  frontRight.setDesiredState(desiredStates[1], DriveConstants.IS_OPEN_LOOP);
  backLeft.setDesiredState(desiredStates[2], DriveConstants.IS_OPEN_LOOP);
  backRight.setDesiredState(desiredStates[3], DriveConstants.IS_OPEN_LOOP);
}

public void setBrakingMode(IdleMode mode) {
  for(SwerveDriveModule module: modules) {
    module.setBrakingMode(mode);
  }
}

public void stopMotors() {
  for(SwerveDriveModule module: modules) {
    module.stopMotors();
  }
}

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
  }
}
