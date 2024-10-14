package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class HoldNoteIntake extends Command {
  private static Intake m_intake;
  public int timer;

  public HoldNoteIntake(Intake m_intake) {
    HoldNoteIntake.m_intake = m_intake;
    setName("FloorIntake");
    addRequirements(m_intake);


  }

  @Override
  public void initialize() {
    m_intake.holdNote();
    timer = 0;
  }

  @Override
  public void execute(){
    timer++;
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    if (timer > 12){
        return true;
    } else{
        return false;
    }
  }
}
