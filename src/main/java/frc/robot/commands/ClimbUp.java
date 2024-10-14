package frc.robot.commands;

import frc.robot.subsystems.Climbers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbUp extends Command {
    private static Climbers m_climbers;

    public ClimbUp(Climbers m_climbers){
        ClimbUp.m_climbers = m_climbers;
        setName("Climbers");
        addRequirements(m_climbers);
    }

    @Override
    public void initialize(){
        m_climbers.robotUp();
    }

    @Override
    public void end(boolean interrupted){
        m_climbers.stopClimb();
    }

    @Override
    public boolean isFinished(){
        return m_climbers.climbReached();
    }
}

