package frc.robot.commands;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command  {

    private final ShooterSubsystem m_shooterSubsystem;

    public Shoot(ShooterSubsystem shooterSubsystem){
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }


    @Override
    public void execute(){
        m_shooterSubsystem.shoot(); 
    }


}
