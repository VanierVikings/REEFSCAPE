package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.FloorIntake;

public class Intake extends SubsystemBase {

    DigitalInput beamBreak = new DigitalInput(9);
    private final VictorSPX intakeBottom = new VictorSPX(DeviceIDs.intakeBottom);
    private final VictorSPX intakeTop = new VictorSPX(DeviceIDs.intakeTop);
    public boolean running = false;
    public Intake() {
        intakeBottom.setInverted(true);
        intakeTop.setNeutralMode(NeutralMode.Brake);
        intakeBottom.setNeutralMode(NeutralMode.Brake);

    }

    public void startIntake(double direction){
        intakeBottom.set(VictorSPXControlMode.PercentOutput, IntakeConstants.bottomSpeed * direction);
        intakeTop.set(VictorSPXControlMode.PercentOutput, IntakeConstants.topSpeed * direction);
        running = true;
    }


    public void stopIntake() {
        intakeBottom.set(VictorSPXControlMode.PercentOutput, 0);
        intakeTop.set(VictorSPXControlMode.PercentOutput, 0);
        running = false;
    }
     public Boolean hasNote(){
        return !beamBreak.get();
    }

    public Command intakeCommand(double direction) {
        return this.startEnd(() -> this.startIntake(direction), () -> this.stopIntake()).withName("Intake");
      }
      
    public Command reverseCommand(double direction) {
        return this.startEnd(() -> this.startIntake(-direction), () -> this.stopIntake()).withName("Reverse");
    }

    public Command intakeSequence(double direction) {
        return intakeCommand(direction).until(() -> hasNote())
        .andThen(intakeCommand(direction).withTimeout(0.5))
        .andThen(reverseCommand(direction).withTimeout(0.3));
    }

    public boolean isIntakeActive(){
        return running;
    }


    @Override
    public void periodic() {
    }
}
