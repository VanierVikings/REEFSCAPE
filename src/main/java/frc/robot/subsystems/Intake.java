package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    DigitalInput beamBreak = new DigitalInput(0);
    private final VictorSPX intakeBottom = new VictorSPX(DeviceIDs.intakeBottom);
    private final VictorSPX intakeTop = new VictorSPX(DeviceIDs.intakeTop);
    public Intake() {
        intakeBottom.setInverted(true);
        intakeTop.setNeutralMode(NeutralMode.Brake);
        intakeBottom.setNeutralMode(NeutralMode.Brake);

    }

    public void startIntake(){
        intakeBottom.set(VictorSPXControlMode.PercentOutput, IntakeConstants.bottomSpeed);
        intakeTop.set(VictorSPXControlMode.PercentOutput, IntakeConstants.topSpeed);
    }


    public void stopIntake() {
        intakeBottom.set(VictorSPXControlMode.PercentOutput, 0);
        intakeTop.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public void holdNote(){
        intakeBottom.set(VictorSPXControlMode.PercentOutput, -IntakeConstants.bottomSpeed*0.2);
        intakeTop.set(VictorSPXControlMode.PercentOutput, -IntakeConstants.topSpeed*0.2);
    }

    public Boolean hasNote(){
        return !beamBreak.get();
    }
}
