package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

public class SwerveModule {
    private final  SparkMax m_driveMotor;
    private final SparkMax m_angleMotor;

    private final RelativeEncoder m_driveEncoder;
    private final AbsoluteEncoder m_angleEncoder;

    private final SparkClosedLoopController m_driveClosedLoopController;
    private final SparkClosedLoopController m_angleClosedLoopController;

    private double m_angleOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveCANID, int ANGLECANID, double angleOffset){
        m_driveMotor = new SparkMax(driveCANID, MotorType.kBrushless);
        m_angleMotor = new SparkMax(ANGLECANID, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_angleEncoder = m_angleMotor.getAbsoluteEncoder();

        m_driveClosedLoopController = m_driveMotor.getClosedLoopController();
        m_angleClosedLoopController = m_angleMotor.getClosedLoopController();

        // TODO: Reset parameters on spark max, apply config and then set to perisist
        m_driveMotor.configure(Configs.SwerveModule.driveConfig, 
                               ResetMode.kResetSafeParameters, 
                               PersistMode.kPersistParameters);
        
        m_angleMotor.configure(Configs.SwerveModule.driveConfig, 
                               ResetMode.kResetSafeParameters, 
                               PersistMode.kPersistParameters);
        m_angleOffset = angleOffset;
        m_desiredState.angle = new Rotation2d(m_angleEncoder.getPosition());
        m_driveEncoder.setPosition(0);
    }

    public SwerveModuleState getState(){
        // Actual code vomit
        return new SwerveModuleState(m_driveEncoder.getVelocity(),
                                     new Rotation2d(
                                        m_angleEncoder.getPosition() - m_angleOffset)
        );
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            new Rotation2d(m_angleEncoder.getPosition() - m_angleOffset)
        );
    }
    public void setDesiredState(SwerveModuleState desiredState){
        // Apply current angle to desired state
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_angleOffset));

        // Optimize
        correctedDesiredState.optimize(new Rotation2d(m_angleEncoder.getPosition()));

        m_driveClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, 
                                                ControlType.kVelocity);

        m_angleClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), 
                                                ControlType.kPosition);
        // And set it
        m_desiredState = desiredState;
    }
    
    public void resetEncoders(){
        m_driveEncoder.setPosition(0);
    }
}
