package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LED.States;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;


public class Prime extends Command{
    private static Shooter m_shooter;
    private static LED m_led;
    private static Intake m_intake;

    public Prime(Shooter m_shooter, LED m_led, Intake m_intake){
      Prime.m_shooter = m_shooter;
      setName("Prime");
      addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
      m_shooter.set(ShooterConstants.maxRPM);
      m_led.requestState(States.ShooterEnabled);
    }

    @Override 
    public void execute(){
      if (m_shooter.atSetpoint()){
        m_led.requestState(States.ShooterSpeedReached);
      }
    }

    @Override
    public void end(boolean interrupted) {
      m_shooter.stop();
      if (m_intake.hasNote()){
        m_led.requestState(States.IntakedNote);
      } else{
        m_led.requestState(States.Default);
      }
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
