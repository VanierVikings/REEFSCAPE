package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class FloorIntake extends Command {
  private static Intake m_intake;
  private static LED m_led;
  private double direction;

  public FloorIntake(Intake m_intake, double direction, LED m_Led) {
    FloorIntake.m_intake = m_intake;
    this.direction = direction;
    setName("FloorIntake");
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_led.setColor(Color.kLightYellow, true);
    m_intake.startIntake(direction);
  }

  @Override
  public void execute(){
    SmartDashboard.putBoolean("Has Note", m_intake.hasNote());
  }

  @Override
  public void end(boolean interrupted) {
    if (m_intake.hasNote()){
      m_led.setColor(Color.kDarkRed, false);
    } else {
      m_led.setColor(Color.kAliceBlue, false);
    }
    m_intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return direction == 1 ? m_intake.hasNote() : false;
  }
}
