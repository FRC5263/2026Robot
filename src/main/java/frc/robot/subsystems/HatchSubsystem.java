package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class HatchSubsystem extends SubsystemBase {
   Servo hatchServo;

   public HatchSubsystem(){
    hatchServo = new Servo(Constants.ShooterConstants.kHatchID);
   }

   public void Hatch(double power){
    hatchServo.set(power);
   }

   public void StopHatch(){
    hatchServo.setSpeed(0);
   }

   public Command RunHatch(DoubleSupplier power) {
      return run(() -> {Hatch(power.getAsDouble());});
   }
}
