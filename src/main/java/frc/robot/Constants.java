// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants {
    public static final double kDriveEncoderPositionToMeters = (1.0 / 6.75) * Units.inchesToMeters(4.0 * Math.PI); 
    public static final double kDriveEncoderVelocityToMetersPerSec = kDriveEncoderPositionToMeters / 60.0;
    public static final double kTurnEncoderPositionToRadians = 2.0 * Math.PI;
    public static final double kTranslationalDeadbandMetersPerSecond = 0.001;
    // public static final double kMaxTranslationalMetersPerSecond = Units.feetToMeters(14.5);
    public static final double kMaxTranslationalMetersPerSecond = Units.feetToMeters(4.5);

    
    public static final double kPTurning = 0.5;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0;
    public static final double kWheelBase = Units.inchesToMeters(28);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2.0, kWheelBase / 2.0),
      new Translation2d(kWheelBase / 2.0, -kWheelBase / 2.0),
      new Translation2d(-kWheelBase / 2.0, kWheelBase / 2.0),
      new Translation2d(-kWheelBase / 2.0, -kWheelBase / 2.0)
    );
    // TODO: gather all of these constants.
    public static int kFrontLeftDriveId = 2;
    public static int kFrontLeftTurnId = 1;
    public static int kFrontLeftAbsoluteEncoderPort = 0;
    public static double kFrontLeftAbsoluteEncoderOffset = -2.000703539422531 ;
    public static boolean kFrontLeftDriveReversed = false;

    public static int kFrontRightDriveId = 4;
    public static int kFrontRightTurnId = 3;
    public static int kFrontRightAbsoluteEncoderPort = 1;
    public static double kFrontRightAbsoluteEncoderOffset = 3.012447823937954;
    public static boolean kFrontRightDriveReversed = true;

    public static int kBackLeftDriveId = 8;
    public static int kBackLeftTurnId = 7;
    public static int kBackLeftAbsoluteEncoderPort = 3;
    public static double kBackLeftAbsoluteEncoderOffset = 2.821914151725208;
    public static boolean kBackLeftDriveReversed = false;

    public static int kBackRightDriveId = 6;
    public static int kBackRightTurnId = 5;
    public static int kBackRightAbsoluteEncoderPort = 2;
    public static double kBackRightAbsoluteEncoderOffset = 4.275249192969147;
    public static boolean kBackRightDriveReversed = true;

    public static double kDeadband = 0.05;
    // public static double kTeleopMaxAccelMetersPerSecondSquared = 3;
    public static double kTeleopMaxAccelMetersPerSecondSquared = 0.5;
    // public static double kTeleopMaxAngularAccelRadiansPerSecondSquared = 4;
    public static double kTeleopMaxAngularAccelRadiansPerSecondSquared = 0.5;
    public static double kTeleopMaxSpeedMetersPerSecond = kMaxTranslationalMetersPerSecond / 4.0;
    // public static double kTeleopMaxTurningRadiansPerSecond = 4.0 * Math.PI;
    public static double kTeleopMaxTurningRadiansPerSecond = 1.5   * Math.PI;
    public static int kDriveJoystickId = 0;
    public static int kJoystickXAxis = 1;
    public static int kJoystickYxis = 0;
    public static int kJoystickRotAxis = 4;
    public static boolean kFrontLeftTurningReversed = true;
    public static boolean kFrontRightTurningReversed = true;
    public static boolean kBackLeftTurningReversed = true;
    public static boolean kBackRightTurningReversed = true;

    public static double kP_X = 0.5;
    public static double kI_X = 0;
    public static double kD_X = 0;

    public static double kP_Y = 0.5;
    public static double kI_Y = 0;
    public static double kD_Y = 0;

    public static double kP_Theta = 0.5;
    public static double kI_Theta = 0;
    public static double kD_Theta = 0;
  }
}
