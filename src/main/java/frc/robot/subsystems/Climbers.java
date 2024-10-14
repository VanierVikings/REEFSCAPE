package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.IntakeConstants;

public class Climbers extends SubsystemBase{
    private final VictorSPX leftClimbMotor = new VictorSPX(DeviceIDs.leftClimb);
    private final VictorSPX rightClimbMotor = new VictorSPX(DeviceIDs.rightClimb);
    private final DigitalInput leftSwtich = new DigitalInput(DeviceIDs.leftClimbSwitch);
    private final DigitalInput rightSwitch = new DigitalInput(DeviceIDs.rightClimbSwitch);
    private final double ClimbSpeed = 0.2;
    public Climbers(){
        leftClimbMotor.setNeutralMode(NeutralMode.Brake);
        rightClimbMotor.setNeutralMode(NeutralMode.Brake);
        leftClimbMotor.setInverted(true);
        rightClimbMotor.setInverted(true);
        SmartDashboard.putBoolean("Switch Active", climbReached());

    }

    public void robotUp(){
        leftClimbMotor.set(VictorSPXControlMode.PercentOutput, ClimbSpeed);
        rightClimbMotor.set(VictorSPXControlMode.PercentOutput, ClimbSpeed);
    }

    public void robotDown(){
        leftClimbMotor.set(VictorSPXControlMode.PercentOutput, -ClimbSpeed);
        rightClimbMotor.set(VictorSPXControlMode.PercentOutput, -ClimbSpeed);
    }

    public void stopClimb(){
        leftClimbMotor.set(VictorSPXControlMode.PercentOutput, 0);
        rightClimbMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public Boolean climbReached(){
        if (leftSwtich.get() || rightSwitch.get()){
            return true;
        } else{
            return false;
        }
    }
}