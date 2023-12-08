// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoDriveForward;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.PrintData;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveDrive m_swerve = new SwerveDrive();

  private final Joystick m_driverJoystick = new Joystick(DriveConstants.kDriveJoystickId);

  private final PrintData m_print = new PrintData(m_swerve);

  private final JoystickDrive m_drive = new JoystickDrive(m_swerve, 
    () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickXAxis),
    () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickYxis),
    () -> -m_driverJoystick.getRawAxis(DriveConstants.kJoystickRotAxis)
  );

  // private AutoDriveForward m_autoDriveForward = new AutoDriveForward(m_swerve);

  private PathConstraints m_constraints = new PathConstraints(DriveConstants.kTeleopMaxSpeedMetersPerSecond, DriveConstants.kTeleopMaxAccelMetersPerSecondSquared);
  private List<PathPlannerTrajectory> m_trajectories = TrajectoryUtils.readTrajectory("DriveForward", m_constraints);
  private List<PPSwerveControllerCommand> m_swerveCommands = TrajectoryUtils.generatePPSwerveControllerCommand_(m_swerve, m_trajectories);

  public RobotContainer() {
    m_swerve.setDefaultCommand(m_drive);

    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("Swerve/Reset_Heading", new InstantCommand(() -> m_swerve.resetHeading()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // System.out.println("Trajectories :" + m_trajectories);
    // An example command will be run in autonomous
    // return new ParallelCommandGroup(m_swerveCommands.get(0), m_print);
    // return m_autoDriveForward;
    return m_swerveCommands.get(0);
  }
}
