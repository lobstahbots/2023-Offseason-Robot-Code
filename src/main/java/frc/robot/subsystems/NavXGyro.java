// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class NavXGyro implements GyroIO {
    private final AHRS gyro = new AHRS();

    public NavXGyro(){}

    public Rotation2d getYaw() {
        return (SwerveConstants.invertGyro)
            ? Rotation2d.fromDegrees(360 - (gyro.getRotation2d().getDegrees() % 360))
            : Rotation2d.fromDegrees(gyro.getRotation2d().getDegrees() % 360);
      }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public double getYawVelocity() {
        return gyro.getVelocityZ();
    }

    public double getRollVelocity() {
        return gyro.getVelocityY();
    }

    public double getPitchVelocity() {
        return gyro.getVelocityX();
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.rollPositionRad = getRoll().getRadians();
        inputs.pitchPositionRad = getPitch().getRadians();
        inputs.yawPositionRad = getYaw().getRadians();
        inputs.rollVelocity = getRollVelocity();
        inputs.pitchVelocity = getPitchVelocity();
        inputs.yawVelocity = getYawVelocity();
    }
}
