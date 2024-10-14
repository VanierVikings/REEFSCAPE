package frc.robot.commands;

import frc.robot.subsystems.Climbers;
import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
    private static Climbers m_climbers;
    private static double direction;

    public Climb(Climbers m_climbers, double direction){
        Climb.m_climbers = m_climbers;
        Climb.direction = direction;
        setName("Climb");
        addRequirements(m_climbers);
    }

    @Override
    public void execute(){
        m_climbers.winch(direction);
    }

    @Override
    public void end(boolean interrupted){
        m_climbers.stopClimb();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}

