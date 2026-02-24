// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.shootContinuous;
import frc.robot.subsystems.swerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  //private final shootContinuous m_calculateAndShoot = new shootContinuous();
  private Command c_driveAnglularVelocity;

  Joystick m_driveStick = new Joystick(Constants.OIConstants.kDriverJoystickPort);
  Joystick m_angleStick = new Joystick(Constants.OIConstants.kAngleJoystickPort);

  private final swerveSubsystem driveBase  = new swerveSubsystem(new File(Filesystem.getDeployDirectory().getPath()));

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                                () -> m_driveStick.getY() * -1,
                                                                () -> m_driveStick.getX() * -1)
                                                            .withControllerRotationAxis(() -> m_driveStick.getX())
                                                            .deadband(0.05)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> m_angleStick.getY(),
                                                                                             () -> m_driveStick.getY())
                                                           .headingWhile(true);

  public RobotContainer() {
    configureBindings();
    Command c_driveAnglularVelocity = driveBase.driveFieldOriented(driveDirectAngle);
    driveBase.setDefaultCommand(c_driveAnglularVelocity);
  }

  private void configureBindings() {
    JoystickButton m_button = new JoystickButton(m_angleStick, 0); 
    //m_button.whileTrue(m_calculateAndShoot);
  }

  public Command getAutonomousCommand() {
    return new Command(){};
  }
}
