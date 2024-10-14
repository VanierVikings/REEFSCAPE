package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.ClimbConstants;

public class Climbers extends SubsystemBase{
    private final VictorSPX leftClimbMotor = new VictorSPX(DeviceIDs.leftClimb);
    private final VictorSPX rightClimbMotor = new VictorSPX(DeviceIDs.rightClimb);
    private final DigitalInput leftSwtich = new DigitalInput(DeviceIDs.leftClimbSwitch);
    private final DigitalInput rightSwitch = new DigitalInput(DeviceIDs.rightClimbSwitch);
    private double leftSpeed = 0.2;
    private double rightSpeed = 0.2;
    public Climbers(){
        leftClimbMotor.setNeutralMode(NeutralMode.Brake);
        rightClimbMotor.setNeutralMode(NeutralMode.Brake);
        SmartDashboard.putBoolean("Switch Active", climbReached());
    }

    public void winch(double direction){
        leftClimbMotor.set(VictorSPXControlMode.PercentOutput, leftSpeed*direction);
        rightClimbMotor.set(VictorSPXControlMode.PercentOutput, rightSpeed*direction);
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Climb Speed", leftSpeed);
        SmartDashboard.putNumber("Right Climb Speed", rightSpeed);

        leftSpeed = SmartDashboard.getNumber("Left Climb Speed", leftSpeed);
        rightSpeed = SmartDashboard.getNumber("Right Climb Speed", rightSpeed);
    }
}