// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  /** Creates a new ShooterIO. */
  @AutoLog
  public static class ShooterIOInputs {
    public double mainCurrentAmps = 0.0;
    public double mainAppliedVolts = 0.0;

    public double auxiliaryCurrentAmps = 0.0;
    public double auxiliaryAppliedVolts = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the main motor speed. */
  public default void mainDrive(double mainSpeed) {}

  /** Set the auxiliary motor speed. */
  public default void auxiliaryDrive(double auxiliarySpeed) {}

  /** Stops the main motor. */
  public default void stopMain() {}

  /** Stops the auxiliary motor. */
  public default void stopAuxiliary() {}
}
