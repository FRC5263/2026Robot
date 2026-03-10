package frc.robot.commands;
import frc.robot.subsystems.HatchSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class hatchCommad extends Command {
    HatchSubsystem hatchSubsystem = new HatchSubsystem();
    double angle;

    public hatchCommad(HatchSubsystem hatchSubsystem, double angle){
        this.hatchSubsystem = hatchSubsystem;
        this.angle = angle;

        addRequirements(hatchSubsystem);
    }
    
    @Override
    public void initialize(){}

    @Override
    public void execute(){
      hatchSubsystem.Hatch(angle);
    }
    
   @Override
   public void end(boolean interrupted){
    hatchSubsystem.StopHatch();
   }

   @Override
   public boolean isFinished(){
    return false;
   }
}
