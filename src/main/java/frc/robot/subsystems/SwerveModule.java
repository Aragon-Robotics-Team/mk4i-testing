// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turnMotor;

  private final RelativeEncoder m_driveEncoder;
  private final DutyCycleEncoder m_absoluteEncoder;

  private final double m_absoluteEncoderOffset;
  private final PIDController m_turningPIDController;

  private final int m_moduleId;

  private Translation2d m_translation = new Translation2d();

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveId, int turnId, int absoluteEncoderPort, double absoluteEncoderOffset,
      boolean driveReversed, boolean turningReversed, int moduleId) {
    // Initialize motors and encoders.
    m_driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
    m_turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_absoluteEncoder = new DutyCycleEncoder(new DigitalInput(absoluteEncoderPort));

    // Set conversion coefficients.
    m_driveEncoder.setVelocityConversionFactor(DriveConstants.kDriveEncoderVelocityToMetersPerSec);
    m_driveEncoder.setPositionConversionFactor(DriveConstants.kDriveEncoderPositionToMeters);
    m_absoluteEncoder.setDistancePerRotation(DriveConstants.kTurnEncoderPositionToRadians);

    // Initialize Everything else.
    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_turningPIDController = new PIDController(DriveConstants.kPTurning, DriveConstants.kITurning,
        DriveConstants.kDTurning);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_moduleId = moduleId;

    m_turnMotor.setIdleMode(IdleMode.kBrake);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_driveMotor.setInverted(driveReversed);
    m_turnMotor.setInverted(turningReversed);
  }

  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return m_absoluteEncoder.getDistance() - m_absoluteEncoderOffset;
  }

  public Rotation2d getRotation() {
    return new Rotation2d(getTurningPosition());
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public void resetEncoders() {
    m_driveEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getRotation());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), getRotation());
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < DriveConstants.kTranslationalDeadbandMetersPerSecond) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    SmartDashboard.putNumber("Swerve/Commanded/Speed_" + m_moduleId, state.speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Commanded/Angle_" + m_moduleId, state.angle.getRadians());

    m_driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxTranslationalMetersPerSecond);
    m_turnMotor.set(m_turningPIDController.calculate(getRotation().getRadians(), state.angle.getRadians()));

    SmartDashboard.putString("Swerve_" + m_moduleId + "_state", state.toString());
  }

  public Translation2d getTranslation(){
    return new Translation2d(m_translation.getX(), m_translation.getY());
  }

  public void stop() {
    m_driveMotor.set(0.0);
    m_turnMotor.set(0.0);
  }

  public double getDriveCurrent(){
    return m_driveMotor.getOutputCurrent();
  }

  public double getTurnCurrent(){
    return m_turnMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Swerve/Angle/Module_" + m_moduleId, getRotation().getRadians());
    SmartDashboard.putNumber("Swerve/Speed/Module_" + m_moduleId, getDriveVelocity());
  }
}
