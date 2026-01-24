package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Configs;

public class ClimberSubsystem extends SubsystemBase{

private final SparkMax climb1Motor = new SparkMax(Constants.ClimbingConstants.kClimb1ID, MotorType.kBrushless);
//climb 2 follows 1 i think??????
private final SparkMax climb2Motor = new SparkMax(Constants.ClimbingConstants.kClimb2ID, MotorType.kBrushless);
private final RelativeEncoder m_climbEncoder = climb1Motor.getEncoder();
//2 on both limits
DigitalInput topLimit = new DigitalInput(Constants.ClimbingConstants.m_TOPCHANNEL);
DigitalInput bottomLimit = new DigitalInput(Constants.ClimbingConstants.m_BOTTOMCHANNEL);

        public static final SparkMaxConfig climb2Config = new SparkMaxConfig();



public void setClimb(double speed){

    //idk how the follow function works so im just gonna set climb2 to the same instead
if (speed>0){
    climb2Motor.isFollower();
    if (topLimit.get()){
        //stop when top limit reached
        climb1Motor.set(0);
        //climb2Motor.set(0)
    }
    else {
        climb1Motor.set(speed);
        //climb2Motor.set(speed);
    }
}
    else{
        if (bottomLimit.get()){
            climb1Motor.set(0);
            //climb2Motor.set(0)
        }
    else{

       climb1Motor.set(speed);
            //climb2Motor.set(speed)
        }
    }


}

        public void ClimbUp(){
            climb1Motor.set(1); //or whatever
            climb1Motor.setInverted(false);
        }
        
        public void ClimbDown(){
            climb1Motor.setInverted(true);
            climb1Motor.set(1);
        }

  




}