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

/*isn't it funny i didn't know what a digital limit switch looked like whilst making this :/ */
DigitalInput topLimit = new DigitalInput(Constants.ClimbingConstants.m_TOPCHANNEL);
DigitalInput bottomLimit = new DigitalInput(Constants.ClimbingConstants.m_BOTTOMCHANNEL);


   

        public static final SparkMaxConfig climb2Config = new SparkMaxConfig(); 
        public static final SparkMaxConfig climb1Config = new SparkMaxConfig();


public ClimberSubsystem(){
    //.follow has to be set through sparkmaxconfig now i guess??????????
    climb2Config
    .follow(climb1Motor,false)
    .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(40);
            


       climb1Config
       //I have not a clue in the world if the brake thing works at all
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
        /*I redid climb again */
        public void Climb(double speed){
            boolean attop = !topLimit.get();
            boolean atbottom = !bottomLimit.get();
            if(speed >= 0 && attop){
                StopClimb();
                return;
            }      
        if(speed<=0 && atbottom){
                StopClimb();
                return;
        }
        else{
                climb1Motor.set(speed);
                return;
        }
        }
        


        
        public void StopClimb(){
            climb1Motor.set(0);
        }
      
        
}