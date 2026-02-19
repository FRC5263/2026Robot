// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.node.ObjectNode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class swerveSubsystem extends SubsystemBase {
  
  private final SwerveDrive m_swerveDrive;

  public swerveSubsystem(File directory) {
    boolean blueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;

    Pose2d m_startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)),
    Rotation2d.fromDegrees(0)) : new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)),
    Rotation2d.fromDegrees(180));

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try{
      m_swerveDrive = new SwerveParser(directory).createSwerveDrive(4.5, m_startingPose);
    } catch (Exception e) { throw new RuntimeException(e); }
    m_swerveDrive.setHeadingCorrection(false);
    m_swerveDrive.setCosineCompensator(false);
    m_swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // TODO: check coeff
    m_swerveDrive.setModuleEncoderAutoSynchronize(true, 1); // TODO: check
  }

  public swerveSubsystem(SwerveDriveConfiguration driveConfig, SwerveControllerConfiguration controllerConfig){
    m_swerveDrive = new SwerveDrive(driveConfig,
                                  controllerConfig,
                                  4.5,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
  }

  @Override
  public void periodic() {}

  public Command  c_sysIDMotor(){
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setDriveSysIdRoutine(
        new Config(), 
        this, m_swerveDrive, 
        12, 
        true), 
        3.0, 5.0, 3.0);
  }

  public Command c_sysIDAngleMotor(){
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setAngleSysIdRoutine(
        new Config(), this, m_swerveDrive), 
        3.0, 5.0, 3.0);
  }

  public Command c_centerModules(){
    return run(() -> Arrays.asList(m_swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  public Command c_forwardDrive(){
    return run(() -> {
      m_swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> m_swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public void v_editFeedForward(double kS, double kV, double kA){
    m_swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    m_swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      m_swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void v_drive(Translation2d translation, double rotation, boolean fieldRelative){
    m_swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  public void v_driveFieldOriented(ChassisSpeeds velocity){
    m_swerveDrive.driveFieldOriented(velocity);
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    m_swerveDrive.resetOdometry(initialHolonomicPose);
  }
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        4.5);
  }
  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return m_swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    m_swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    m_swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    m_swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public SwerveDrive getSwerve(){
    return m_swerveDrive;
  }
}
