// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.ModuleConstants;


public class MAXSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMaxConfig k_drivingSparkMaxConfig;

  private final SparkMax m_turningSparkMax;
  private final SparkMaxConfig k_turningSparkMaxConfig;

  public final RelativeEncoder m_drivingEncoder;
  public final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    
    // Driving Spark Max configuration
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    k_drivingSparkMaxConfig = new SparkMaxConfig();
    k_drivingSparkMaxConfig
      .idleMode(ModuleConstants.kDrivingMotorIdleMode)
      .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    k_drivingSparkMaxConfig.encoder
      .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
      .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    k_drivingSparkMaxConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
      .velocityFF(1/ModuleConstants.kDriveWheelFreeSpeedRps)
      .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);
    m_drivingSparkMax.configure(k_drivingSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    //
    
    // Turning Spark Max configuration
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    k_turningSparkMaxConfig = new SparkMaxConfig();
    k_turningSparkMaxConfig
      .idleMode(ModuleConstants.kTurningMotorIdleMode)
      .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    k_turningSparkMaxConfig.absoluteEncoder
      .inverted(ModuleConstants.kTurningEncoderInverted)
      .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
      .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    k_turningSparkMaxConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
      .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput, 
      ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    m_turningSparkMax.configure(k_turningSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    //

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
    
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
