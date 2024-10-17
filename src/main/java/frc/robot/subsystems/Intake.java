package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    @Override
    public void periodic() {
        if(!beamBreak.get() && !running){
            LED.color = Color.kAliceBlue;
        }
        else if(beamBreak.get() && !running){
            LED.color = Color.kDarkRed;
        }
    }
}
