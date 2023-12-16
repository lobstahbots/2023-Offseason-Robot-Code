// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class PathConstants {
    public static final Pose2d TARGET_POSE = new Pose2d(16, 7, Rotation2d.fromDegrees(180));
    public static final Pose2d INITIAL_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }
  public static class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 0; // Note: add real value
    public static final int STRAFE_X_AXIS = 0;
    public static final int STRAFE_Y_AXIS = 1;
    public static final int ROTATION_AXIS = 2;
    public static final double JOYSTICK_DEADBAND = 0.1;
  }
  public static class RobotConstants {
    public static final double WHEELBASE = Units.inchesToMeters(20);
    public static final double TRACK_WIDTH = Units.inchesToMeters(20);
    public static final double RADIUS = Units.inchesToMeters(new Translation2d(20, 20).getNorm());
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double DRIVE_GEAR_RATIO = 4.71;
    public static final double ANGLE_GEAR_RATIO = 6.1;
  }
  public static class DriveConstants {
    public static final double MAX_VOLTS = 4.95;
    public static final double MAX_ACCELERATION = 4;
    public static final double MAX_DRIVE_SPEED = 5;
    public static final double MAX_ANGULAR_SPEED = MAX_DRIVE_SPEED / RobotConstants.RADIUS;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
    public static final int ANGLE_MOTOR_CURRENT_LIMIT = 40;
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(-RobotConstants.WHEELBASE / 2.0, RobotConstants.TRACK_WIDTH / 2.0),
      new Translation2d(RobotConstants.WHEELBASE / 2.0, RobotConstants.TRACK_WIDTH / 2.0),
      new Translation2d(-RobotConstants.WHEELBASE / 2.0, -RobotConstants.TRACK_WIDTH / 2.0),
      new Translation2d(RobotConstants.WHEELBASE / 2.0, -RobotConstants.TRACK_WIDTH / 2.0));

    public static final boolean FIELD_CENTRIC = true;
    public static final boolean IS_OPEN_LOOP = false;

    public static final double PATH_MAX_ACCEL = 3;
    public static final double PATH_MAX_VELOCITY = 3;

    public static class FrontLeftModuleConstants {
      public static final int moduleID = 0;
      public static final int driveID = 12;
      public static final int angleID = 13;
      public static final double angleOffset = -90;
      public static final boolean inverted = false;
    }
    public static class BackLeftModuleConstants {
      public static final int moduleID = 1;
      public static final int driveID = 16;
      public static final int angleID = 17;
      public static final double angleOffset = 180;
      public static final boolean inverted = false;
    }
    public static class FrontRightModuleConstants {
      public static final int moduleID = 2;
      public static final int driveID = 14;
      public static final int angleID = 15;
      public static final double angleOffset = 0;
      public static final boolean inverted = true;
    }
    public static class BackRightModuleConstants {
      public static final int moduleID = 3;
      public static final int driveID = 10;
      public static final int angleID = 11;
      public static final double angleOffset = 90;
      public static final boolean inverted = true;
    }
  }

  public static class OperatorConstants {
    public static final int SHOOTER_MAIN_BUTTON_ID = 0; // Note: replace with real value
    public static final int SHOOTER_AUXILIARY_BUTTON_ID = 0; // Same as above
  }

  public static class SwerveConstants {
    public static final boolean invertGyro = true;

    public static final double KS = 0.1;
    public static final double KA = 0.1;
    public static final double KV = 0.1;

    public static final double DRIVING_ENCODER_POSITION_CONVERSION_FACTOR = (RobotConstants.WHEEL_DIAMETER * Math.PI) / (RobotConstants.DRIVE_GEAR_RATIO);
    public static final double DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR = DRIVING_ENCODER_POSITION_CONVERSION_FACTOR / 60.0;
    public static final double TURNING_ENCODER_POSITION_CONVERSION_FACTOR = (2 * Math.PI) / (RobotConstants.ANGLE_GEAR_RATIO * 4096);
    public static final double TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = TURNING_ENCODER_POSITION_CONVERSION_FACTOR / 60.0;

    public static final double TURN_PID_MIN_INPUT = 0;
    public static final double TURN_PID_MAX_INPUT = TURNING_ENCODER_POSITION_CONVERSION_FACTOR;

    public static final double DRIVE_PID_MIN_OUTPUT = -1;
    public static final double DRIVE_PID_MAX_OUTPUT = 1;
    public static final double DRIVE_PID_P = 0.04;
    public static final double DRIVE_PID_I = 0;
    public static final double DRIVE_PID_D = 0;
    public static final double DRIVE_PID_FF = 0;

    public static final double TURN_PID_MIN_OUTPUT = -1;
    public static final double TURN_PID_MAX_OUTPUT = 1;
    public static final double TURN_PID_P = 1;
    public static final double TURN_PID_I = 0;
    public static final double TURN_PID_D = 0;
    public static final double TURN_PID_FF = 0;
  }

  public static class SimConstants {
    public static final double LOOP_TIME = 0.01;
  }

  public static class ShooterConstants {
    public static final int AUXILIARY_MOTOR_ID = 1; // NOTE: Replace with real value
    public static final int MAIN_MOTOR_ID = 50; // See above
    public static final int CURRENT_LIMIT = 40; // Max current
    public static final double MAIN_MOTOR_SPEED = 0; // Replace with real value
    public static final double AUXILIARY_MOTOR_SPEED = 0; // See above
  }
}
