package frc.robot.commands;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class FloorIntake extends Command {
  private static Intake m_intake;

  public FloorIntake(Intake m_intake) {
    setName("FloorIntake");
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.startIntake();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return m_intake.hasNote();
  }
}
