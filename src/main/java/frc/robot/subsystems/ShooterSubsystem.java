package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder; 

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{

    private final SparkMax m_shooterMotor = new SparkMax(Constants.ShooterConstants.kShooterID, MotorType.kBrushless);
   // private final RelativeEncoder m_shootEncoder = m_shooterMotor.getEncoder();

 

        public void doshoot(){
            m_shooterMotor.set(1);
        }

}
