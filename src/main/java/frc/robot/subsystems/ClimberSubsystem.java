package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants;
import com.revrobotics.spark.config.SparkMaxConfig;
public class ClimberSubsystem extends SubsystemBase{

private final SparkMax climb1Motor = new SparkMax(Constants.ClimbingConstants.kClimb1ID, MotorType.kBrushless);
//climb 2 follows 1 i think??????
private final SparkMax climb2Motor = new SparkMax(Constants.ClimbingConstants.kClimb2ID, MotorType.kBrushless);
private final RelativeEncoder m_climbEncoder = climb1Motor.getEncoder();
//2 on both limits
DigitalInput topLimit = new DigitalInput(Constants.ClimbingConstants.m_TOPCHANNEL);
DigitalInput bottomLimit = new DigitalInput(Constants.ClimbingConstants.m_BOTTOMCHANNEL);
  double position =  m_climbEncoder.getPosition();

                        //tune these please
   /*in most scenarios useless ->*/ private static final double MaxHeight = Constants.ClimbingConstants.m_MAXHEIGHT;
  
    /*whatever position the encoder starts at -> */    private static final double MinHeight = Constants.ClimbingConstants.m_MINHEIGHT;
    // I think the climber will be at it's bottom when we are on the field so it doesn't matter that much

   

        public static final SparkMaxConfig climb2Config = new SparkMaxConfig(); 
        public static final SparkMaxConfig climb1Config = new SparkMaxConfig();


public ClimberSubsystem(){
    //.follow has to be set through sparkmaxconfig now i guess??????????
    climb2Config
    .follow(climb1Motor,false)
    .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(40);
            


       climb1Config
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(40);
            //limit because climb

       
        climb1Motor.configure(
            climb1Config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        climb2Motor.configure(
            climb2Config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        
    m_climbEncoder.setPosition(0);
     
}


        public void ClimbUp(double speed){
         if (position >= MaxHeight && speed > 0){
            climb1Motor.set(0);
        return;
    }
         if (speed > 0 && topLimit.get()){
        climb1Motor.set(0);
        return;
    }
    else {
        climb1Motor.set(speed);
    }
        }


        public void ClimbDown(double speed){
         if (speed >  0 && bottomLimit.get()){
        climb1Motor.set(0);
        return;
    }
  
    if (position <= MinHeight && speed < 0){
        climb1Motor.set(0);
    }
    else {
        climb1Motor.set(speed);
    }
        }
        public void StopClimb(){
            climb1Motor.set(0);
        }
        
}