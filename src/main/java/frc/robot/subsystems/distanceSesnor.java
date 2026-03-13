// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class distanceSesnor extends SubsystemBase {

  private final Ultrasonic sensor;

  public distanceSesnor() {
    //Ultrasonic.setAutomaticMode(true);  // Just in case
    sensor = new Ultrasonic(0, 1);
  }

  @Override
  public void periodic() {
    update();
  }

  public void update() {
    sensor.getRangeMM();
    SmartDashboard.putNumber("Distance: ", (int)getRange());
  }

  public double getRange() {
    return (sensor.getRangeMM() / 1000);
  }

}