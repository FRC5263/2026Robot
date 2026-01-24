// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    // TODO: Ask build team about this
    public static final int kDrivePinionTeeth = 14;

    public static final double kDriveMotorSpeedRPS = MotorConstants.kFreeSpeedRPM / 60;
    public static final float kWheelDiameterMeters = 0.1016f; // TODO: Get these measurments
    public static final double kWheelCircumfrenceMeters = kWheelDiameterMeters * Math.PI;

    // TODO: get: Wheel bevel gear tooth count, first-stage spur gear tooth count, bevel pinion tooth count
    public static final double kDriveMotorReduction = (15 * 14) / (kDrivePinionTeeth * 45); // Cheeseburgers Y Fries -ConNor
    public static final double kDriveWheelFreeSpeedRPS = (kDriveMotorSpeedRPS * kWheelCircumfrenceMeters) / kDriveMotorReduction;
  }

  public static final class DriveConstants {
    public static final float kMaxSpeedMPS = 4.8f;  // Limits :(
    public static final double kMaxAngularSpeed = Math.PI * 2;  // In radians per second

    // TODO: Change when bot dimentions come back pls
    public static final double kTrackWidth = Units.inchesToMeters(27);
    public static final double kWheelBase = Units.inchesToMeters(27);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // ------------ CAN ID ------------ //
    public static int kFrontLeftDriveID = 1, 
                      kBackLeftDriveID = 2, 
                      kBackRightDriveID = 3, 
                      kFrontRightDriveID = 4,
                      kFrontLeftAngleID = 5,
                      kBackLeftAngleID = 6,
                      kBackRightAngleID = 7,
                      kFrontRightAngleID = 8;
    // ------------ ------ ------------ //

    public static final double kFrontLeftAngleOffset = -Math.PI / 2;
    public static final double kBackLeftAngleOffset = Math.PI;
    public static final double kBackRightAngleOffset = Math.PI / 2;
    public static final double kFrontRightAngleOffset = 0;

    public static final boolean isGryroReversed = false;
    }
    public static final class ShooterConstants{
      public static int kShooterID = 9;
    }

    public static final class OIConstants{
      public static final int kDriverJoystickPort = 0;
      public static final int kAngleJoystickPort  = 1;
      public static final int kEverythingElsePort = 2;
      public static final double kDriveDeadband = 0.005;
    }

    public static final class AutoConstants{
      public static final double kMaxSpeedMetersPerSecond = 0; // Limits in case auton shits
      public static final double kMaxAccelelerationMetersPerSecondSquared = 0;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
      public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;

      // Constraints oh yeah
    }

    public static final class MotorConstants{
      public static int kFreeSpeedRPM = 5676; // Taken direct from the website
    }
}