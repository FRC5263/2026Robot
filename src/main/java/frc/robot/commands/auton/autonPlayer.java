// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.io.File;
import java.util.Scanner;

import frc.robot.subsystems.DriveSubsystem;

public class autonPlayer {
    Scanner scanner;
    double startTime;
    boolean onTime = true;
    double nextDouble;

    public autonPlayer() {
        try{
            scanner = new Scanner(new File("/home/lvuser/auton.csv"));

            scanner.useDelimiter(",|\\n");
            startTime = System.currentTimeMillis();
        } catch(Exception e){
            System.out.print("shit's fucked" + e);
            return;
        }
    }


    public void play(DriveSubsystem driveSubsystem) {
        if((scanner != null) && (scanner.hasNextDouble())){
            double deltaTime;

            if(onTime){
                nextDouble = scanner.nextDouble();
            }

            deltaTime = nextDouble - (System.currentTimeMillis()-startTime);

            if(deltaTime <= 0){
                driveSubsystem.getModule("front left").getDriveMotor().set(scanner.nextDouble());
                driveSubsystem.getModule("front left").getAngleMotor().set(scanner.nextDouble());

                driveSubsystem.getModule("front right").getDriveMotor().set(scanner.nextDouble());
                driveSubsystem.getModule("front right").getAngleMotor().set(scanner.nextDouble());

                driveSubsystem.getModule("rear left").getDriveMotor().set(scanner.nextDouble());
                driveSubsystem.getModule("rear left").getAngleMotor().set(scanner.nextDouble());

                driveSubsystem.getModule("rear right").getDriveMotor().set(scanner.nextDouble());
                driveSubsystem.getModule("rear right").getAngleMotor().set(scanner.nextDouble());

                onTime = true;
            } else {
                onTime = false;
            }
        } else {
            this.end(driveSubsystem);
            if(scanner != null) {
                scanner.close();
                scanner = null;
            }
        }
    }
    public void end(DriveSubsystem driveSubsystem) {
        driveSubsystem.getModule("front left").getDriveMotor().set(0);
        driveSubsystem.getModule("front left").getAngleMotor().set(0);

        driveSubsystem.getModule("front right").getDriveMotor().set(0);
        driveSubsystem.getModule("front right").getAngleMotor().set(0);

        driveSubsystem.getModule("rear left").getDriveMotor().set(0);
        driveSubsystem.getModule("rear left").getAngleMotor().set(0);

        driveSubsystem.getModule("rear right").getDriveMotor().set(0);
        driveSubsystem.getModule("rear right").getAngleMotor().set(0);

        if(scanner != null){
            scanner.close();
        }
    }
}
