// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class SimModule {

    private final DCMotor si_driveMotor = DCMotor.getNEO(1);
    private final DCMotor si_angleMotor = DCMotor.getNEO(1);

    private final SparkMaxSim si_driveMAX;
    private final SparkMaxSim si_angleMAX;

    private final SparkAbsoluteEncoderSim si_turningEncoder;
    private final SparkRelativeEncoderSim si_drivingEncoder;

    private double si_chassisAngularOffset = 0;

    private SparkClosedLoopController si_driveController;
    private SparkClosedLoopController si_angleController;

    private SwerveModuleState si_desiredState;

    public SimModule(
    SparkMax driveMAX, 
    SparkMax angleMAX,
    SwerveModuleState state,
    SparkClosedLoopController driveLoopController,
    SparkClosedLoopController angleLoopController,
    double chassisAngularOffset){
        this.si_driveMAX = new SparkMaxSim(driveMAX, si_driveMotor);
        this.si_angleMAX = new SparkMaxSim(angleMAX, si_angleMotor);

        this.si_drivingEncoder = new SparkRelativeEncoderSim(driveMAX);
        this.si_turningEncoder = new SparkAbsoluteEncoderSim(angleMAX);

        this.si_driveController = driveLoopController;
        this.si_angleController = angleLoopController;

        this. si_chassisAngularOffset = chassisAngularOffset;
        
        si_desiredState = state;
        si_desiredState.angle = state.angle;
    }
}
