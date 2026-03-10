package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import pabeles.concurrency.IntOperatorTask.Max;

import com.google.flatbuffers.DoubleVector;
import com.revrobotics.RelativeEncoder;

public class HatchSubsystem extends SubsystemBase {
   public final Servo hatchServo;
   public double openAngle;
   public double closedAngle;
   public boolean isHmax;
   public boolean isHmin;
   public boolean closedPos;
   public double currentAngle;
   

   public HatchSubsystem(){
    hatchServo = new Servo(Constants.ShooterConstants.kHatchChannel);
      
      
      
      
    hatchServo.set(0);
    
   }

   
   public void Hatch(double angle){
      hatchServo.setAngle(angle);
  
    }
    
      
      
     
   
   
 
 
   
  
   public void StopHatch(){
    hatchServo.setSpeed(0);
   }

}