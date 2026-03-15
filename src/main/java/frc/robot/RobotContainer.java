// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootContinuous;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  Joystick m_driveStick = new Joystick(Constants.OIConstants.kDriverJoystickPort);
  Joystick m_angleStick = new Joystick(Constants.OIConstants.kAngleJoystickPort);

  private final SwerveSubsystem driveBase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory().getPath()));
  
  private final RunIntake m_intake = new RunIntake();

   Command driveTest = driveBase.driveCommand(() -> MathUtil.applyDeadband(m_driveStick.getY(), 0.1), 
                                              () -> MathUtil.applyDeadband(m_driveStick.getX(), 0.1), 
                                              () -> m_angleStick.getX());
  Command shoot = new ShootContinuous();

  public RobotContainer() {
    configureBindings();
    NamedCommands.registerCommand("test", Commands.print("Hello, World!"));
    //NamedCommands.registerCommand("driveadShootLeftBlue", m_shooter.withTimeout(1.2));
    NamedCommands.registerCommand("intake", m_intake);
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("do nothing", Commands.none());
    autoChooser.addOption("drive", driveBase.driveForward().withTimeout(1));
    SmartDashboard.putData("auto chooser", autoChooser);
    DriverStation.silenceJoystickConnectionWarning(true);
    driveBase.setDefaultCommand(driveTest);
  }

  private void configureBindings() {
    JoystickButton commandButton = new JoystickButton(m_driveStick, 1);
    JoystickButton intakeToggle = new JoystickButton(m_driveStick, 2);
    JoystickButton commandStop = new JoystickButton(m_driveStick, 3);
    commandButton.onTrue(DriverStation.getAlliance().isPresent() ?  
    driveBase.driveToPose(
      DriverStation.getAlliance().get() == Alliance.Red ? 
        new Pose2d(14.115, 2.886, Rotation2d.fromDegrees(152.447))
      : new Pose2d(3.060, 5.244, Rotation2d.fromDegrees(-37.694)))
      : Commands.none());
    
    intakeToggle.toggleOnTrue(m_intake);
    commandStop.toggleOnTrue(shoot);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
