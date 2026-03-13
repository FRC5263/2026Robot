// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootContinuous extends Command {

  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private float power = 0.6f;

  public ShootContinuous() {
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
   do {
       m_shooter.setPower(power);
   } while (m_shooter.isIdealDistance);
   if(m_shooter.isIdealDistance == false){
   }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
