package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;

public class HatchSubsystem extends SubsystemBase {
   Servo hatchServo;

   public HatchSubsystem(){
    hatchServo = new Servo(Constants.ShooterConstants.kHatchID);
   }

   public void Hatch(double angle){
    hatchServo.setAngle(angle);
   }

   public void StopHatch(){
    hatchServo.setSpeed(0);
   }
}
