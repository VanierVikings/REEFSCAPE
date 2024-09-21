package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIDs;

public class Intake extends SubsystemBase {
    private final VictorSPX intakeLeft = new VictorSPX(DeviceIDs.intakeBottom);
    private final VictorSPX intakeRight = new VictorSPX(DeviceIDs.intakeTop);  
    public Intake() {
        intakeLeft.follow(intakeRight);
        intakeLeft.setInverted(true);

    }

    public void setIntake(double speed){
        intakeRight.set(VictorSPXControlMode.PercentOutput, -speed);
        intakeLeft.set(VictorSPXControlMode.PercentOutput, speed);
    }



    public void stopIntake() {
        intakeLeft.set(VictorSPXControlMode.PercentOutput, 0);
        intakeRight.set(VictorSPXControlMode.PercentOutput, 0);
    }
    
}
