// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveId, 
    DriveConstants.kFrontLeftTurnId,
    DriveConstants.kFrontLeftAbsoluteEncoderPort, 
    DriveConstants.kFrontLeftAbsoluteEncoderOffset, 
    DriveConstants.kFrontLeftDriveReversed,
    DriveConstants.kFrontLeftTurningReversed, 
    0
  );

  private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveId, 
    DriveConstants.kFrontRightTurnId,
    DriveConstants.kFrontRightAbsoluteEncoderPort, 
    DriveConstants.kFrontRightAbsoluteEncoderOffset, 
    DriveConstants.kFrontRightDriveReversed, 
    DriveConstants.kFrontRightTurningReversed, 
    1
  );

  private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveId, 
    DriveConstants.kBackLeftTurnId,
    DriveConstants.kBackLeftAbsoluteEncoderPort, 
    DriveConstants.kBackLeftAbsoluteEncoderOffset, 
    DriveConstants.kBackLeftDriveReversed, 
    DriveConstants.kBackLeftTurningReversed, 
    2
  );

  private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.kBackRightDriveId, 
    DriveConstants.kBackRightTurnId,
    DriveConstants.kBackRightAbsoluteEncoderPort, 
    DriveConstants.kBackRightAbsoluteEncoderOffset, 
    DriveConstants.kBackRightDriveReversed, 
    DriveConstants.kBackRightTurningReversed, 
    3
  );

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeft.getTranslation(), m_frontRight.getTranslation(), m_backLeft.getTranslation(), m_backRight.getTranslation());

  private final AHRS m_imu = new AHRS();

  private double m_totalCurrent;

  private final PIDController m_xPID = new PIDController(DriveConstants.kP_X, DriveConstants.kI_X, DriveConstants.kD_X);
  private final PIDController m_yPID = new PIDController(DriveConstants.kP_Y, DriveConstants.kI_Y, DriveConstants.kD_Y);
  private final PIDController m_thetaPID = new PIDController(DriveConstants.kP_Theta, DriveConstants.kI_Theta, DriveConstants.kD_Theta);

  private final SwerveDriveOdometry m_odo = new SwerveDriveOdometry(m_kinematics, getAngle(), new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_backLeft.getPosition(),
    m_backRight.getPosition()
  });

  public void resetHeading() {
    m_imu.reset();
  }

  public void resetOdo(Pose2d pose) {
    m_odo.resetPosition(getAngle(), new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    }, pose);
  }

  public Pose2d getPoseMeters(){
    return m_odo.getPoseMeters();
  }

  public SwerveDriveKinematics getSwerveKinematics(){
    return m_kinematics;
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxTranslationalMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setModuleStates(states);  
  }

  public void printVelocitiesandPositions(){
    System.out.println("Front left Velocity: " + m_frontLeft.getDriveVelocity());
    System.out.println("Front left Position: " + m_frontLeft.getDrivePosition());
    System.out.println("Front right velocity: " + m_frontRight.getDriveVelocity());
    System.out.println("Front right Position: " + m_frontRight.getDrivePosition());
    System.out.println("Back left velocity: " + m_backLeft.getDriveVelocity());
    System.out.println("Back left Position: " + m_backLeft.getDrivePosition());
    System.out.println("Back right velocity: " + m_backRight.getDriveVelocity());
    System.out.println("Back right Position: " + m_backRight.getDrivePosition());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(m_imu.getAngle(), 360));
  }

  public PIDController getXController(){
    return m_xPID;
  }

  public PIDController getYController(){
    return m_yPID;
  }

  public PIDController getThetaController(){
    return m_thetaPID;
  }
  
  public ChassisSpeeds getChassisSpeeds() { 
    return new ChassisSpeeds(); 
  }

  public void driveRobotRelative(ChassisSpeeds speeds) { }

  public SwerveDrive(){
     AutoBuilder.configureHolonomic(
                this::getPoseMeters, // Robot pose supplier
                this::resetOdo, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0.5, 0.0, 0.0), // Rotation PID constants
                        0.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new com.pathplanner.lib.util.ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
     );
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  @Override
  public void periodic() {
    m_odo.update(getAngle(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(),
        m_backLeft.getPosition(), m_backRight.getPosition()
      });

    SmartDashboard.putNumber("Angle", getAngle().getDegrees());
    
    m_totalCurrent = m_frontLeft.getDriveCurrent() + m_frontLeft.getTurnCurrent() + m_frontRight.getDriveCurrent() + m_frontRight.getTurnCurrent() + m_backLeft.getDriveCurrent() + m_backLeft.getTurnCurrent() + m_backRight.getDriveCurrent() + m_backRight.getTurnCurrent();
    SmartDashboard.putNumber("Total Current", m_totalCurrent);
  }
}