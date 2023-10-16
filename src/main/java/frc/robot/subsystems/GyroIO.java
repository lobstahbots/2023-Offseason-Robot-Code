// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
      public boolean connected = false;
      public double rollPositionRad = 0.0;
      public double pitchPositionRad = 0.0;
      public double yawPositionRad = 0.0;
      public double rollVelocity = 0.0;
      public double pitchVelocity = 0.0;
      public double yawVelocity= 0.0;
    }
  
    public default void updateInputs(GyroIOInputs inputs) {}
  }