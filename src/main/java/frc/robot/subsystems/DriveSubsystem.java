// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
      Constants.DriveConstants.kFrontLeftDriveID, 
      Constants.DriveConstants.kFrontLeftAngleID, 
      Constants.DriveConstants.kFrontLeftAngleOffset);

  private final SwerveModule m_backLeft = new SwerveModule(
      Constants.DriveConstants.kBackLeftDriveID, 
      Constants.DriveConstants.kBackLeftAngleID, 
      Constants.DriveConstants.kBackLeftAngleOffset);

  private final SwerveModule m_backRight = new SwerveModule(
    Constants.DriveConstants.kBackRightDriveID, 
    Constants.DriveConstants.kBackRightAngleID, 
    Constants.DriveConstants.kBackRightAngleOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
    Constants.DriveConstants.kFrontRightDriveID, 
    Constants.DriveConstants.kFrontRightAngleID, 0);

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry
  SwerveDriveOdometry m_driveOdometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics, 
    Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)), 
    new SwerveModulePosition[]{
      m_frontLeft.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition(),
      m_frontRight.getPosition()
    });

  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    m_driveOdometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition(),
          m_frontRight.getPosition()
        });
  }
  
  public Pose2d getPose() {
    return m_driveOdometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose){
    m_driveOdometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition(),
          m_frontRight.getPosition()
        },
        pose);
  }

  // Now for the big one
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative){
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMPS;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMPS;
    double rotDelivered = rotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMPS);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_backLeft.setDesiredState(swerveModuleStates[1]);
    m_backRight.setDesiredState(swerveModuleStates[2]);
    m_frontRight.setDesiredState(swerveModuleStates[3]);
  }

  // Wheel stops
  public void setX(){
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMPS);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_backLeft.setDesiredState(desiredStates[1]);
    m_backRight.setDesiredState(desiredStates[2]);
    m_frontRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders(){
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
    m_frontRight.resetEncoders();
  }

  public void zeroHeading(){
    m_gyro.reset();
  }

  public double getHeading(){
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  public double getTurnRate(){
    return m_gyro.getRate(IMUAxis.kZ) * (Constants.DriveConstants.isGryroReversed ? -1.0 : 1.0);
  }
}
