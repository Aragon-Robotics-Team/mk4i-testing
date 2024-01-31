// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class JoystickDrive extends Command {
  private final SwerveDrive m_swerve;
  private final Supplier<Double> m_xSpeed, m_ySpeed, m_turningSpeed;
  private final SlewRateLimiter m_xLimiter, m_yLimiter, m_turningLimiter;

  /** Creates a new JoystickDrive. */
  public JoystickDrive(SwerveDrive swerve, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> turningSpeed) {
    m_swerve = swerve;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_turningSpeed = turningSpeed;

    m_xLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAccelMetersPerSecondSquared);
    m_yLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAccelMetersPerSecondSquared);
    m_turningLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAngularAccelRadiansPerSecondSquared);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get joystick inputs.
    double xSpeed = m_xSpeed.get();
    double ySpeed = m_ySpeed.get();
    double turningSpeed = m_turningSpeed.get();

    // Apply deadband.
    xSpeed = Math.abs(xSpeed) > DriveConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > DriveConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > DriveConstants.kDeadband ? turningSpeed : 0.0;

    // Limit velocity and acceleration.
    xSpeed = m_xLimiter.calculate(xSpeed) * DriveConstants.kTeleopMaxSpeedMetersPerSecond;
    ySpeed = m_yLimiter.calculate(ySpeed) * DriveConstants.kTeleopMaxSpeedMetersPerSecond;
    turningSpeed = m_turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleopMaxTurningRadiansPerSecond;
    SmartDashboard.putNumber("Swerve/turningSpeedCommanded", turningSpeed);

    // Construct chassis speed objects.
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, m_swerve.getAngle());

    // Calculate module states.
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Set module states.
    m_swerve.setModuleStates(moduleStates);

    SmartDashboard.putNumber("Swerve/heading", m_swerve.getAngle().getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
