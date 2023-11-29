// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

public class AutoDriveForward extends CommandBase {

  private SwerveDrive m_swerve;
  private PathConstraints m_constraints = new PathConstraints(DriveConstants.kTeleopMaxSpeedMetersPerSecond, DriveConstants.kTeleopMaxAccelMetersPerSecondSquared);
  
  List<PathPlannerTrajectory> m_trajectories = TrajectoryUtils.readTrajectory("C:/Users/admin/OneDrive/Documents/GitHub/Robotics/mk4i-testing/mk4i-testing/src/main/deploy/pathplanner/Drive Forward.path", m_constraints);
  List<PPSwerveControllerCommand> m_swerveCommands = TrajectoryUtils.generatePPSwerveControllerCommand(m_swerve, m_trajectories);
  
  /** Creates a new AutoDriveForward. */
  public AutoDriveForward(SwerveDrive swerve) {
    m_swerve = swerve;
        // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveCommands.get(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
