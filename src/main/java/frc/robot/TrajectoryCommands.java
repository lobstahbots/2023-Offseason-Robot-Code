// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveBase;

public class TrajectoryCommands {
  /** Creates a new TrajectoryCommands. */
  public TrajectoryCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static Command followTrajectoryCommand(DriveBase driveBase, PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               driveBase.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             driveBase::getPose, // Pose supplier
             DriveConstants.KINEMATICS, // SwerveDriveKinematics
             new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             driveBase::setModuleStates, // Module states consumer
             true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             driveBase // Requires this drive subsystem
         )
     );
 }
}
