package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

public class SwerveModule {
  private final SparkMax m_driveMotor;
    private final SparkMax m_angleMotor;
  
    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;
  
    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;
  
    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  
    public SwerveModule(int driveCANID, int turningCANID, double chassisAngularOffset) {
      m_driveMotor = new SparkMax(driveCANID, MotorType.kBrushless);
      m_angleMotor = new SparkMax(turningCANID, MotorType.kBrushless);
  
      m_drivingEncoder = m_driveMotor.getEncoder();
      m_turningEncoder = m_angleMotor.getAbsoluteEncoder();
  
      m_drivingClosedLoopController = m_driveMotor.getClosedLoopController();
      m_turningClosedLoopController = m_angleMotor.getClosedLoopController();
  
      m_driveMotor.configure(Configs.SwerveModule.driveConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
      m_angleMotor.configure(Configs.SwerveModule.angleConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
  
      m_chassisAngularOffset = chassisAngularOffset;
      m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
      m_drivingEncoder.setPosition(0);
    }
  
    public SwerveModuleState getState() {
      return new SwerveModuleState(m_drivingEncoder.getVelocity(),
          new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    public RelativeEncoder getRelativeEncoder(){
      return m_drivingEncoder;
    }

    public AbsoluteEncoder getAbsoluteEncoder(){
      return m_turningEncoder;
    }

    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
          m_drivingEncoder.getPosition(),
          new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }
  
    public void setDesiredState(SwerveModuleState desiredState) {
      SwerveModuleState correctedDesiredState = new SwerveModuleState();
      correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
      correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
  
      correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
  
      m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
      m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
  
      m_desiredState = desiredState;
    }
  
    public void resetEncoders() {
      m_drivingEncoder.setPosition(0);
    }
    
    public SparkMax getDriveMotor(){
      return m_driveMotor;
  }

  public SparkMax getAngleMotor(){
    return m_angleMotor;
  }
}
