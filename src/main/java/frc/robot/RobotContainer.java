// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.shootContinuous;
import frc.robot.subsystems.swerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final shootContinuous m_calculateAndShoot = new shootContinuous();

  private final swerveSubsystem driveBase = new swerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));


  Joystick m_driveStick = new Joystick(Constants.OIConstants.kDriverJoystickPort);
  Joystick m_angleStick = new Joystick(Constants.OIConstants.kAngleJoystickPort);

  SwerveInputStream driveDirectAngle = SwerveInputStream.of(driveBase.getSwerve(), 
  () -> m_driveStick.getY() * -1, 
  () -> m_driveStick.getX() * -1)
  .withControllerRotationAxis(() -> m_angleStick.getX())
  .deadband(0.05)
  .scaleRotation(0.8)
  .allianceRelativeControl(true)
  .withControllerHeadingAxis(() -> m_angleStick.getX(), () -> m_angleStick.getY())
  .headingWhile(true);

  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {
    Command driveFieldOrientedCorrectAngle = driveBase.driveFieldOriented(driveDirectAngle);
    driveBase.setDefaultCommand(driveFieldOrientedCorrectAngle);
  }

  public Command getAutonomousCommand() {
    return new Command() {
    }; // TODO: Verify this works pls
  }
}
