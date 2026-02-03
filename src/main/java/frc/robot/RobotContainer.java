// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.shoot;

public class RobotContainer {

  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();


  Joystick m_driveStick = new Joystick(Constants.OIConstants.kDriverJoystickPort);
  Joystick m_angleStick = new Joystick(Constants.OIConstants.kAngleJoystickPort);
  Joystick m_utilStick = new Joystick(Constants.OIConstants.kEverythingElsePort);
  

  public RobotContainer() {
    configureBindings();

    m_drive.setDefaultCommand(
      new RunCommand(
        () -> m_drive.drive(
          -MathUtil.applyDeadband(m_driveStick.getY(), OIConstants.kDriveDeadband), 
          -MathUtil.applyDeadband(m_driveStick.getX(), OIConstants.kDriveDeadband), 
          -MathUtil.applyDeadband(m_angleStick.getX(), OIConstants.kDriveDeadband), 
          false),
         m_drive));


       
  }

  private void configureBindings() {
    new JoystickButton(m_driveStick, 3)
      .onTrue(new RunCommand(
        () -> m_drive.setX(), 
        m_drive));

    new JoystickButton(m_driveStick, 2)
      .onTrue(new InstantCommand(
        () -> m_drive.zeroHeading(),
        m_drive));

       new JoystickButton(m_utilStick, 1) 
       .whileTrue(new shoot(m_shooter));



  }

  public Command getAutonomousCommand() {
    return new Command() {}; // TODO: Build pls
  }
}
