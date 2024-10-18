package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FloorIntake extends Command {
  private static Intake m_intake;
  private double direction;

  public FloorIntake(Intake m_intake, double direction) {
    FloorIntake.m_intake = m_intake;
    this.direction = direction;
    setName("FloorIntake");
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    switch ((int)(direction)) {
      case (1):
        m_intake.intakeSequence(direction);
      case(-1):
        m_intake.intakeCommand(direction);
    }
  }

  @Override
  public void execute(){
    SmartDashboard.putBoolean("Has Note", m_intake.hasNote());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return m_intake.hasNote();
  }
}
