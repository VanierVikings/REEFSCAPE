package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIDs;

public class Climbers extends SubsystemBase{
    private final VictorSPX leftClimbMotor = new VictorSPX(DeviceIDs.leftClimb);
    private final VictorSPX rightClimbMotor = new VictorSPX(DeviceIDs.rightClimb);
    private double leftSpeed = 0.7;
    private double rightSpeed = 0.75;
    public Climbers(){
        leftClimbMotor.setNeutralMode(NeutralMode.Brake);
        rightClimbMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void winch(double direction){
        leftClimbMotor.set(VictorSPXControlMode.PercentOutput, leftSpeed*direction);
        rightClimbMotor.set(VictorSPXControlMode.PercentOutput, rightSpeed*direction);
    }

    public void stopClimb(){
        leftClimbMotor.set(VictorSPXControlMode.PercentOutput, 0);
        rightClimbMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
    }
} 