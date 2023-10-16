// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SimConstants;
import stl.math.LobstahMath;

public class SwerveModuleSim implements SwerveModuleIO {
  /** Creates a new SwerveModuleSim. */
  private FlywheelSim simDriveMotor = new FlywheelSim(DCMotor.getNEO(1), RobotConstants.DRIVE_GEAR_RATIO, 0.025);
  private FlywheelSim simAngleMotor = new FlywheelSim(DCMotor.getNEO(1), RobotConstants.ANGLE_GEAR_RATIO, 0.025);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public SwerveModuleSim() {
  }

  public void updateInputs(ModuleIOInputs inputs) {
    simDriveMotor.update(SimConstants.LOOP_TIME);
    simAngleMotor.update(SimConstants.LOOP_TIME);

    double angleDelta = simAngleMotor.getAngularVelocityRadPerSec() * 0.020;

    SmartDashboard.putNumber("Angle delta", angleDelta);
    SmartDashboard.putNumber("drive position rad", inputs.drivePositionRad);

    inputs.turnPositionRad += angleDelta;

    inputs.turnPositionRad = LobstahMath.wrapValue(inputs.turnPositionRad, 0, 2 * Math.PI);

    inputs.drivePositionRad = inputs.drivePositionRad + (simDriveMotor.getAngularVelocityRadPerSec() * SimConstants.LOOP_TIME);
    inputs.driveVelocityRadPerSec = simDriveMotor.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] { Math.abs(simDriveMotor.getCurrentDrawAmps()) };

    SmartDashboard.putNumber("angle", simAngleMotor.getAngularVelocityRadPerSec());

    inputs.turnVelocityRadPerSec = simAngleMotor.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] { Math.abs(simAngleMotor.getCurrentDrawAmps()) };
  }

  public void setDriveVoltage(double volts) {
    simDriveMotor.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  public void setTurnVoltage(double volts) {
    simAngleMotor.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

}
