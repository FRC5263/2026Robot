// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AgitatorSubsystem extends SubsystemBase {
  private final SparkMax m_agitatorMotor;
  public AgitatorSubsystem() {
    m_agitatorMotor = new SparkMax(Constants.ShooterConstants.kAgitatorID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setPower(double power) {
    m_agitatorMotor.set(power);
  }
}
