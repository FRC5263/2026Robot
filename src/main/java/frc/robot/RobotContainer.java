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
import frc.robot.commands.shootContinuous;
import frc.robot.commands.auton.auton;
import frc.robot.commands.auton.autonPlayer;
import frc.robot.commands.auton.autonWriter;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final shootContinuous m_calculateAndShoot = new shootContinuous();
  private final autonWriter m_recorder = new autonWriter();
  private final autonPlayer m_player = new autonPlayer();
  private final auton m_auton = new auton(m_player, m_drive);

  Joystick m_driveStick = new Joystick(Constants.OIConstants.kDriverJoystickPort);
  Joystick m_angleStick = new Joystick(Constants.OIConstants.kAngleJoystickPort);

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
    
    new JoystickButton(m_angleStick, 1)
    .onTrue(m_calculateAndShoot);

    new JoystickButton(m_angleStick, 2)
      .toggleOnTrue(new InstantCommand(
        () -> m_recorder.record(m_drive),
        m_drive));
  }

  public Command getAutonomousCommand() {
    return m_auton; // TODO: Build pls
  }
}
