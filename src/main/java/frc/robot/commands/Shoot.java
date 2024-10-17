package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;


public class Shoot extends Command{
    private static Shooter m_shooter;
    private static Intake m_intake;
    private static LED m_led;

    public Shoot(Shooter m_shooter, Intake m_intake, LED m_led){
      Shoot.m_shooter = m_shooter;
      Shoot.m_intake = m_intake;
      setName("Shoot");
      addRequirements(m_shooter, m_intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_shooter.set(ShooterConstants.maxRPM);
        m_intake.startIntake(1);
        if (m_shooter.atSetpoint()){
          m_led.setColor(Color.kLimeGreen, true);
        } else{
          m_led.setColor(Color.kLimeGreen, false);
        }
    }
  
    @Override
    public void end(boolean interrupted) {
      m_shooter.stop();
      m_intake.stopIntake();
      m_led.setColor(Color.kAliceBlue, false);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
