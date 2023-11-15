// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SimConstants;

public class DriveBase extends SubsystemBase {
  /** Creates a new SwerveDriveBase. */
  private final SwerveModule[] modules; 

  private SwerveDrivePoseEstimator swerveOdometry;
  private final NavXGyro gyro = new NavXGyro();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private boolean isOpenLoop;

  private ChassisSpeeds setpoint = new ChassisSpeeds();

  private SwerveModuleState[] states =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private Field2d field;

  public DriveBase(GyroIO gyroIO, SwerveModuleIO frontLeft, SwerveModuleIO frontRight, SwerveModuleIO backRight, SwerveModuleIO backLeft, boolean isOpenLoop) {

    this.modules = new SwerveModule[]{new SwerveModule(frontLeft, 0), new SwerveModule(frontRight, 1), new SwerveModule(backRight, 2), new SwerveModule(backLeft, 3)};
    
    this.gyroIO = gyroIO;

    gyro.zeroGyro();
    swerveOdometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS, gyro.getYaw(), getPositions(), new Pose2d());

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    this.isOpenLoop = isOpenLoop;
  }
  
  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(gyro.getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule module : modules) {
      states[module.getModuleID()] = module.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule module : modules) {
      positions[module.getModuleID()] = module.getPosition();
    }
    return positions;
  }

  public void swerveDrive(Translation2d translation, double rotation, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, gyro.getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_DRIVE_SPEED);

    for (SwerveModule module : modules) {
      module.setDesiredState(swerveModuleStates[module.getModuleID()], isOpenLoop);
    }
}

public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
  SwerveModuleState[] desiredStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
  this.setModuleStates(desiredStates);
}

public SwerveModuleState[] setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
  SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.MAX_DRIVE_SPEED);
  for(SwerveModule module: modules) {
    optimizedStates[module.getModuleID()] = module.setDesiredState(desiredStates[module.getModuleID()], isOpenLoop);
  }
  return optimizedStates;
}

public void setBrakingMode(IdleMode mode) {
  for(SwerveModule module: modules) {
    module.setBrakingMode(mode);
  }
}

public void stopMotors() {
  for(SwerveModule module: modules) {
    module.stop();
  }
}

public boolean isOpenLoop() {
  return isOpenLoop;
}

public void setIsOpenLoop(boolean newValue) {
  isOpenLoop = newValue;
}

public ChassisSpeeds getRobotRelativeSpeeds() {
  return ChassisSpeeds.fromFieldRelativeSpeeds(DriveConstants.KINEMATICS.toChassisSpeeds(getStates()), gyro.getYaw());
} 

public Command followPathCommand(String pathName){
  PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

  // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
  return new FollowPathWithEvents(
      new FollowPathHolonomic(
          path,
          this::getPose, // Robot pose supplier
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
          new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
              4.5, // Max module speed, in m/s
              0.36, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options here
          ),
          this // Reference to this subsystem to set requirements
      ),
      path, // FollowPathWithEvents also requires the path
      this::getPose // FollowPathWithEvents also requires the robot pose supplier
  );
}

  @Override
  public void periodic() {
    swerveOdometry.update(gyro.getYaw(), getPositions());
    field.setRobotPose(getPose());
    SmartDashboard.putString("Pose", getPose().toString());
    Logger.getInstance().recordOutput("Odometry", getPose());
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    for (SwerveModule module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      // Stop moving while disabled
      for (var module : modules) {
        module.stop();
      }

      // Clear setpoint logs
      Logger.getInstance().recordOutput("SwerveStates/Desired", new double[] {});

    } else {
      Twist2d setpointTwist =
          new Pose2d()
              .log(
                  new Pose2d(
                      setpoint.vxMetersPerSecond * SimConstants.LOOP_TIME,
                      setpoint.vyMetersPerSecond * SimConstants.LOOP_TIME,
                      new Rotation2d(setpoint.omegaRadiansPerSecond * SimConstants.LOOP_TIME)));

      ChassisSpeeds adjustedSpeeds =
          new ChassisSpeeds(
              setpointTwist.dx / SimConstants.LOOP_TIME,
              setpointTwist.dy / SimConstants.LOOP_TIME,
              setpointTwist.dtheta / SimConstants.LOOP_TIME);
      SwerveModuleState[] newStates = DriveConstants.KINEMATICS.toSwerveModuleStates(adjustedSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DriveConstants.MAX_DRIVE_SPEED);

      // Set to last angles if zero
      if (adjustedSpeeds.vxMetersPerSecond == 0.0
          && adjustedSpeeds.vyMetersPerSecond == 0.0
          && adjustedSpeeds.omegaRadiansPerSecond == 0) {
        for (int i = 0; i < 4; i++) {
          newStates[i] = new SwerveModuleState(0.0, states[i].angle);
        }
      }

      states = newStates;

    Logger.getInstance().recordOutput("SwerveStates/Desired", setModuleStates(newStates));
    Logger.getInstance().recordOutput("SwerveStates/Measured", getStates());
    Logger.getInstance().recordOutput("Odometry/Robot", getPose());
    
    // Log 3D odometry pose
    Pose3d robotPose3d = new Pose3d(getPose());
    robotPose3d =
        robotPose3d
            .exp(
                new Twist3d(
                    0.0,
                    0.0,
                    Math.abs(gyroInputs.pitchPositionRad) * RobotConstants.TRACK_WIDTH / 2.0,
                    0.0,
                    gyroInputs.pitchPositionRad,
                    0.0))
            .exp(
                new Twist3d(
                    0.0,
                    0.0,
                    Math.abs(gyroInputs.rollPositionRad) * RobotConstants.TRACK_WIDTH / 2.0,
                    gyroInputs.rollPositionRad,
                    0.0,
                    0.0));

    Logger.getInstance().recordOutput("Odometry/Robot3d", robotPose3d);
    }
  }
}