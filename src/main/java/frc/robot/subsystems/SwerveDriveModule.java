// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDriveModule extends SubsystemBase {
  private final CANSparkMax angleMotor;
  private final CANSparkMax driveMotor;
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder angleEncoder;
  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SwerveConstants.KS, SwerveConstants.KV, SwerveConstants.KA);
  private Rotation2d angularOffset;
  private Rotation2d lastAngle;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private final int moduleID;

  /** Creates a new SwerveDriveWheel. 
   * 
   * @param moduleID The module number (0-3).
   * @param angleMotorID The CAN ID of the motor controlling the angle.
   * @param driveMotorID The CAN ID of the motor controlling drive speed.
   * @param angularOffsetDegrees The offset angle in degrees.
  */
  public SwerveDriveModule (int moduleID, int angleMotorID, int driveMotorID, double angularOffsetDegrees) {
    this.moduleID = moduleID;

    this.angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    angleMotor.restoreFactoryDefaults();
    driveMotor.restoreFactoryDefaults();
    driveMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    angleMotor.setSmartCurrentLimit(DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT);

    driveController = this.driveMotor.getPIDController();
    angleController = this.angleMotor.getPIDController();
    drivingEncoder = angleMotor.getEncoder();
    angleEncoder = driveMotor.getAbsoluteEncoder(Type.kDutyCycle);
    drivingEncoder.setPositionConversionFactor(SwerveConstants.DRIVING_ENCODER_POSITION_CONVERSION_FACTOR);
    drivingEncoder.setVelocityConversionFactor(SwerveConstants.DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR);
    angleEncoder.setPositionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR);
    angleEncoder.setVelocityConversionFactor(SwerveConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR);
    driveController.setFeedbackDevice(drivingEncoder);
    angleController.setFeedbackDevice(angleEncoder);
    angleEncoder.setInverted(true);

    angleController.setPositionPIDWrappingEnabled(true);
    angleController.setPositionPIDWrappingMinInput(SwerveConstants.TURN_PID_MIN_INPUT);
    angleController.setPositionPIDWrappingMaxInput(SwerveConstants.TURN_PID_MAX_INPUT);

    driveController.setP(SwerveConstants.DRIVE_PID_P);
    driveController.setI(SwerveConstants.DRIVE_PID_I);
    driveController.setD(SwerveConstants.DRIVE_PID_D);
    driveController.setFF(SwerveConstants.DRIVE_PID_FF);
    driveController.setOutputRange(SwerveConstants.DRIVE_PID_MIN_OUTPUT, SwerveConstants.DRIVE_PID_MAX_OUTPUT);

    angleController.setP(SwerveConstants.TURN_PID_P);
    angleController.setI(SwerveConstants.TURN_PID_I);
    angleController.setD(SwerveConstants.TURN_PID_D);
    angleController.setFF(SwerveConstants.TURN_PID_FF);
    angleController.setOutputRange(SwerveConstants.TURN_PID_MIN_OUTPUT,
            SwerveConstants.TURN_PID_MAX_OUTPUT);

    driveMotor.burnFlash();
    angleMotor.burnFlash();

    this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees);
    this.desiredState.angle = new Rotation2d(angleEncoder.getPosition());
    drivingEncoder.setPosition(0);

    lastAngle = getState().angle;
  }

  /**
   * Returns the module ID.
   *
   * @return The ID number of the module (0-3).
   */
  public int getModuleID () {
    return moduleID;
  }


  /**
   * Returns the current state of the module.
   *
   * @return The current encoder velocities of the module as a {@link SwerveModuleState}.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(drivingEncoder.getVelocity(), new Rotation2d(angleEncoder.getPosition() - angularOffset.getRadians()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current encoder positions position of the module as a {@link SwerveModulePosition}.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(drivingEncoder.getPosition(), new Rotation2d(angleEncoder.getPosition() - angularOffset.getRadians()));
  }

  /**
   * Sets the braking mode of the motors.
   * 
   * @param the {@link IdleMode} to set motors to.
   */
  public void setBrakingMode(IdleMode mode) {
    angleMotor.setIdleMode(mode);
    driveMotor.setIdleMode(mode);
  }

  public void stopMotors() {
    angleMotor.stopMotor();
    driveMotor.stopMotor();
  }

  
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState A {@link SwerveModuleState} with desired speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(angularOffset);

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(angleEncoder.getPosition()));

    setSpeed(optimizedDesiredState, isOpenLoop);
    setAngle(optimizedDesiredState);

    this.desiredState = desiredState;
  }

  /**
   *  Sets just the drive speed of the Swerve Module.
   *
   * @param desiredState A {@link SwerveModuleState with desired speed}.
   * @param isOpenLoop Whether or not to use PID.
   */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.MAX_DRIVE_SPEED;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity, 
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  /**
   *  Sets just the angle of the Swerve Module.
   *
   * @param desiredState A {@link SwerveModuleState with desired angle}.
   */
  private void setAngle(SwerveModuleState desiredState) {
    /*Keep from jittering.*/
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.TURN_PID_MAX_OUTPUT * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

   /** Zeroes the drive encoder. */
   public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

}

