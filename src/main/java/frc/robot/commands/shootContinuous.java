// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class shootContinuous extends Command {
  // TODO: math
  private double power = 1;

  Optional<Alliance> team = DriverStation.getAlliance();

  String gameData = DriverStation.getGameSpecificMessage();
  char hubToShoot = team.get() == (Alliance.Red) ? 'R' : 'B';

  private ShooterSubsystem m_shooterSub = new ShooterSubsystem();
  public shootContinuous() {
    addRequirements(m_shooterSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(gameData.length() > 0) {
      switch(gameData.charAt(0)) {
        case 'R':
          if(hubToShoot == 'R'){
            m_shooterSub.setPower(power);
          }else {end(isScheduled());}
          break;
        case 'B':
          if(hubToShoot == 'B'){
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
