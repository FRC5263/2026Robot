// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.io.FileWriter;
import java.io.IOException;

import frc.robot.subsystems.DriveSubsystem;
public class autonWriter {

    private double startTime;

    private FileWriter m_writer;

    public autonWriter() {
        try {
            startTime = System.currentTimeMillis();

            m_writer = new FileWriter("/home/lvuser/deploy/auton.csv");
        } catch (Exception e) {
            System.out.print("file gone!" + e);
            return;
        }
    }

    public void record(DriveSubsystem driveSubsystem) {
        if(m_writer != null) {
            try {
                m_writer.append("" + (System.currentTimeMillis() - startTime));

                m_writer.append("," + (driveSubsystem.getModule("front left").getDriveMotor().get()));
                m_writer.append("," + (driveSubsystem.getModule("front left").getAngleMotor().get()));

                m_writer.append("," + (driveSubsystem.getModule("front right").getDriveMotor().get()));
                m_writer.append("," + (driveSubsystem.getModule("front right").getAngleMotor().get()));

                m_writer.append("," + (driveSubsystem.getModule("rear left").getDriveMotor().get()));
                m_writer.append("," + (driveSubsystem.getModule("rear left").getAngleMotor().get()));

                m_writer.append("," + (driveSubsystem.getModule("rear right").getDriveMotor().get()));
                m_writer.append("," + (driveSubsystem.getModule("rear right").getAngleMotor().get()) + "\n");
            } catch(Exception e) {
                System.out.print(e);
                return;
            }
        }
    }

    public void end() throws IOException {
        if(m_writer != null) {
            m_writer.flush();
            m_writer.close();
        }
    }
}
