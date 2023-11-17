// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class FollowTrajectoryCommand extends CommandBase {
  
  private SwerveDrive m_swerve;
  private Supplier<PathPlannerTrajectory> m_trajectory;

  /** Creates a new FollowTrajectoryCommand. */
  public FollowTrajectoryCommand(SwerveDrive swerve, Supplier<PathPlannerTrajectory> trajectory) {
    m_swerve = swerve;
    m_trajectory = trajectory;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
