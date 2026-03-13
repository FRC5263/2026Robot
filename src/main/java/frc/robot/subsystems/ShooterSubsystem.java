package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder; 

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{

    private final SparkMax m_shooterMotor;
    private final RelativeEncoder m_shooterEncoder;
    private final distanceSesnor m_sensor;
    public boolean isIdealDistance = false;

    public ShooterSubsystem(){
        m_shooterMotor = new SparkMax(Constants.ShooterConstants.kShooterID, MotorType.kBrushless);
        m_shooterEncoder  = m_shooterMotor.getEncoder();
        m_sensor = new distanceSesnor();
    }

    public void setPower(double power) {
        m_shooterMotor.set(power);
    }

    @Override
    public void periodic(){
        if(m_sensor.getRange() > 2.2 && m_sensor.getRange() < 3.0) isIdealDistance = true;
    }

    public double getPower() {
        return m_shooterMotor.get();
    }
}
