// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class shootContinuous extends Command {
  // TODO: math
  private double power = 1;

  private ShooterSubsystem m_shooterSub = new ShooterSubsystem();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(gameData.s_gameData.length() > 0) {
      switch(gameData.s_gameData.charAt(0)) {
        case 'R':
          if(gameData.getAlliance() == 'R'){
            m_shooterSub.setPower(power);
          }else {end(isScheduled());}
          break;
        case 'B':
          if(gameData.s_gameData == 'B'){
            m_shooterSub.setPower(power);
          }else {end(isScheduled());}
          break;
        default :
          break;
      }
    }else{}
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSub.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
*/