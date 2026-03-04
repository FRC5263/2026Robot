package frc.robot.commands;
import frc.robot.subsystems.HatchSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class hatchCommad extends Command {
    HatchSubsystem hatchSubsystem = new HatchSubsystem();
    DoubleSupplier angleSupplier;

    public hatchCommad(HatchSubsystem hatchSubsystem, DoubleSupplier angleSupplier){
        this.hatchSubsystem = hatchSubsystem;
        this.angleSupplier = angleSupplier;

        addRequirements(hatchSubsystem);
    }
    
    @Override
    public void initialize(){}

    @Override
    public void execute(){
        hatchSubsystem.Hatch(angleSupplier.getAsDouble());
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
