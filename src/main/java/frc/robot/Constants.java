// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class RobotConstants {
    public static final double LENGTH = 24;
    public static final double WIDTH = 24;
    public static final double RADIUS = Math.sqrt((WIDTH*WIDTH) + (LENGTH*LENGTH));
  }
  public static class DriveConstants {
    public static final double MAX_VOLTS = 4.95;
    public static final int frontLeftID = 0;
    public static final int frontLeftAngleID = 0;
    public static final int frontRightID = 0;
    public static final int frontRightAngleID = 0;
    public static final int backLeftID = 0;
    public static final int backLeftAngleID = 0;
    public static final int backRightID = 0;
    public static final int backRightAngleID = 0;
  }
}
