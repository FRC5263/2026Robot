package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;

public class climbCommand extends Command{
    ClimberSubsystem climbersubsystem = new ClimberSubsystem();
    private final DoubleSupplier speedSupplier;
   

    public climbCommand(ClimberSubsystem climbersubsystem, DoubleSupplier speedSupplier){
        this.climbersubsystem = climbersubsystem;
        this.speedSupplier = speedSupplier;
        

        addRequirements(climbersubsystem);
    }
   
    @Override
    public void initialize() {}
    @Override
    public void execute(){
        climbersubsystem.Climb(speedSupplier.getAsDouble());
    }
    @Override
    public void end(boolean interrupted){
        climbersubsystem.StopClimb();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}