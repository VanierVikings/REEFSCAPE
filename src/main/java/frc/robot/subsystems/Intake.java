package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    DigitalInput beamBreak = new DigitalInput(9);
    VictorSPX victorTop = new VictorSPX(DeviceIDs.intakeTop);
    VictorSPX victorBot = new VictorSPX(DeviceIDs.intakeBottom);
    Boolean running = false;

    public Intake() {

    }

    public Boolean hasNote() {
        return !beamBreak.get();    
    }

    public void runMotors(int direction) {
        // Set mode (percent output) and speed
        // direction is 1 or -1, for forwards and backwards
        victorTop.set(VictorSPXControlMode.PercentOutput, -1 * direction);
        victorBot.set(VictorSPXControlMode.PercentOutput, -1 * direction);
        running = true;
    }

    public void stopMotors() {
        // Set mode (percent output) and speed (0 for stationary)
        victorTop.set(VictorSPXControlMode.PercentOutput, 0);
        victorBot.set(VictorSPXControlMode.PercentOutput, 0);
        running = false;
    }

    public Command shlurp(int direction) {
        return this.startEnd(() -> this.runMotors(direction), () -> this.stopMotors());
    }

    @Override
    public void periodic() {
        if(hasNote()) {
            LED.setColor(Color.kDarkGreen);
        }
        else if(!hasNote() && running) {
            LED.setColor(Color.kGold);
        }
        else if (!running){
            LED.setColor(Color.kAliceBlue);
        }
    }

}
