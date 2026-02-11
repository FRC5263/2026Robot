package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.RelativeEncoder;

public class HatchSubsystem extends SubsystemBase {
    //TODO: i think garret said make servo so
    private final Servo HatchServo = new Servo(Constants.ShooterConstants.kHatchID);
   
    


}
