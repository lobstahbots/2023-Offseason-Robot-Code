// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
  private final int moduleID;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private SwerveModuleIO io;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      SwerveConstants.KS, SwerveConstants.KV, SwerveConstants.KA);
  private final PIDController driveController;
  private final PIDController angleController;

  public SwerveModule(SwerveModuleIO io, int moduleID) {
    this.io = io;
    this.moduleID = moduleID;
    this.driveController = new PIDController(SwerveConstants.DRIVE_PID_P, SwerveConstants.DRIVE_PID_I,
        SwerveConstants.DRIVE_PID_D);
    this.angleController = new PIDController(SwerveConstants.TURN_PID_P, SwerveConstants.TURN_PID_I,
        SwerveConstants.TURN_PID_D);
  }

  public void stop() {
    io.setDriveVoltage(0);
    io.setTurnVoltage(0);
  }

  /**
   * Returns the module ID.
   *
   * @return The ID number of the module (0-3).
   */
  public int getModuleID() {
    return moduleID;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive/Module" + Integer.toString(moduleID), inputs);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState A {@link SwerveModuleState} with desired speed and angle.
   */
  public SwerveModuleState setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(inputs.angularOffset);

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(inputs.turnPositionRad));

    if (Robot.isReal()) {
      io.setDriveSpeed(optimizedDesiredState, isOpenLoop);
      io.setAngle(optimizedDesiredState);
    } else {
      io.setTurnVoltage(
          angleController.calculate(getAngle().getRadians(), optimizedDesiredState.angle.getRadians()));

      // Update velocity based on turn error
      optimizedDesiredState.speedMetersPerSecond *= Math.cos(angleController.getPositionError());

      // Run drive controller
      double velocityRadPerSec = optimizedDesiredState.speedMetersPerSecond / (RobotConstants.WHEEL_DIAMETER / 2);
      io.setDriveVoltage(
          feedforward.calculate(velocityRadPerSec)
              + driveController.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
    }

    return optimizedDesiredState;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakingMode(IdleMode mode) {
    io.setDriveBrakeMode(mode);
    io.setTurnBrakeMode(mode);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(inputs.turnPositionRad));
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * RobotConstants.WHEEL_DIAMETER / 2;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * RobotConstants.WHEEL_DIAMETER / 2;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

}