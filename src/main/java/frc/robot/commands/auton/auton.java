// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class auton extends Command {
  private autonPlayer m_player;
  private DriveSubsystem m_subsystem;

  public auton(autonPlayer player, DriveSubsystem subsystem) {
    this.m_player = player;
    this.m_subsystem = subsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_player.play(m_subsystem);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
