// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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

  Joystick m_driveStick = new Joystick(Constants.OIConstants.kDriverJoystickPort);
  Joystick m_angleStick = new Joystick(Constants.OIConstants.kAngleJoystickPort);

  private final swerveSubsystem driveBase  = new swerveSubsystem(new File(Filesystem.getDeployDirectory().getPath()));
  
   Command driveTest = driveBase.driveCommand(() -> MathUtil.applyDeadband(m_driveStick.getY(), 0.1) * -1, 
                                              () -> MathUtil.applyDeadband(m_driveStick.getX(), 0.1) * -1, 
                                              () -> MathUtil.applyDeadband(m_angleStick.getX(), 0.1));
  
  public RobotContainer() {
    configureBindings();
    driveBase.setDefaultCommand(driveTest);
  }

  private void configureBindings() {
    JoystickButton m_button = new JoystickButton(m_angleStick, 0); 
    //m_button.whileTrue(m_calculateAndShoot);
  }

  public Command getAutonomousCommand() {
    return new Command(){};
  }
}
