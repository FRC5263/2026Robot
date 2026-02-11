package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class climb extends Command{
    ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
   

    public climb(ClimberSubsystem climbersubsystem){
        m_ClimberSubsystem = climbersubsystem;
        

        addRequirements(climbersubsystem);
    }
   

    @Override
    public void execute(){
        m_ClimberSubsystem.Climb(1);
    }
    @Override
    public void end(boolean interrupted){
        m_ClimberSubsystem.StopClimb();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
