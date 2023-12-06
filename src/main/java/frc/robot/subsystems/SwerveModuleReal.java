// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleReal implements SwerveModuleIO {
  private final CANSparkMax angleMotor;
  private final CANSparkMax driveMotor;
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder angleAbsoluteEncoder;
  private final RelativeEncoder angleEncoder;
  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SwerveConstants.KS, SwerveConstants.KV, SwerveConstants.KA);
  private Rotation2d angularOffset;
  private Rotation2d lastAngle;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private final int moduleID;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  /** Creates a new SwerveModule for real cases. 
   * 
   * @param moduleID The module number (0-3).
   * @param angleMotorID The CAN ID of the motor controlling the angle.
   * @param driveMotorID The CAN ID of the motor controlling drive speed.
   * @param angularOffsetDegrees The offset angle in degrees.
  */
  public SwerveModuleReal (int moduleID, int angleMotorID, int driveMotorID, double angularOffsetDegrees, boolean inverted) {
    this.moduleID = moduleID;

    this.angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    angleMotor.restoreFactoryDefaults();
    driveMotor.restoreFactoryDefaults();
    driveMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    angleMotor.setSmartCurrentLimit(DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT);

    driveMotor.setInverted(inverted);

    driveController = this.driveMotor.getPIDController();
    angleController = this.angleMotor.getPIDController();
    drivingEncoder = driveMotor.getEncoder();
    angleAbsoluteEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    angleEncoder = angleMotor.getEncoder();
    drivingEncoder.setPositionConversionFactor(SwerveConstants.DRIVING_ENCODER_POSITION_CONVERSION_FACTOR);
    drivingEncoder.setVelocityConversionFactor(SwerveConstants.DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR);
    angleEncoder.setPositionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR);
    angleEncoder.setVelocityConversionFactor(SwerveConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR);
    driveController.setFeedbackDevice(drivingEncoder);
    angleController.setFeedbackDevice(angleEncoder);
    angleAbsoluteEncoder.setInverted(true);
    // angleEncoder.setInverted(true);

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
    
    angleEncoder.setPosition(angleAbsoluteEncoder.getPosition());
    this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees);
    this.desiredState.angle = new Rotation2d(angleEncoder.getPosition());
    drivingEncoder.setPosition(0);

    lastAngle = getState().angle;
  }

  /**Stops the module motors. */
  public void stopMotors() {
    angleMotor.stopMotor();
    driveMotor.stopMotor();
  }

  /**Initializes angle based on initial absolute encoder reading. */
  public void initializeAngle() {
    angleEncoder.setPosition(angleAbsoluteEncoder.getPosition());
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
   * Sets the braking mode of the driving motor.
   * 
   * @param the {@link IdleMode} to set motors to.
   */
  public void setDriveBrakingMode(IdleMode mode) {
    driveMotor.setIdleMode(mode);
  }


   /**
   * Sets the braking mode of the turning motor.
   * 
   * @param the {@link IdleMode} to set motors to.
   */
  public void setTurnBrakingMode(IdleMode mode) {
    angleMotor.setIdleMode(mode);
  }

  /**
   *  Sets just the drive speed of the Swerve Module.
   *
   * @param desiredState A {@link SwerveModuleState with desired speed}.
   * @param isOpenLoop Whether or not to use PID.
   */
  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
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
  public void setAngle(SwerveModuleState desiredState) {
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

  /**Sets voltage of driving motor.
   * 
   * @param volts The voltage the motor should be set to.
   */
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  /**Sets voltage of turn motor.
   * 
   * @param volts The voltage the motor should be set to.
   */
  public void setTurnVoltage(double volts) {
    angleMotor.setVoltage(volts);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = drivingEncoder.getPosition();
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(angleEncoder.getVelocity() / SwerveConstants.DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR);
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};

    inputs.turnPositionRad = Units.rotationsToRadians(angleEncoder.getPosition());
    inputs.turnAbsolutePositionRad = Units.rotationsToRadians(angleAbsoluteEncoder.getPosition());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(angleEncoder.getVelocity() / SwerveConstants.DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR);
    inputs.turnAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {angleMotor.getOutputCurrent()};
    inputs.angularOffset = angularOffset;
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(moduleID), inputs);
    SmartDashboard.putNumber("Angle" + moduleID, angleEncoder.getPosition());
  }

}

