package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Hang extends SubsystemBase{

private SparkMax grabber;  //DO NOT CHANGE
private SparkClosedLoopController grabberClosedLoopController;
private RelativeEncoder grabberEncoder;
public double target = 0;
 

public Hang(){
    grabber = new SparkMax(HangConstants.grabberID, MotorType.kBrushless);
    SparkMaxConfig grabberConfig = new SparkMaxConfig();
    grabberConfig
    .smartCurrentLimit(45)
    .inverted(true)
    .idleMode(IdleMode.kCoast);

    grabberEncoder = grabber.getEncoder();

    grabberClosedLoopController = grabber.getClosedLoopController();

    grabberConfig.encoder
            .positionConversionFactor(HangConstants.conversionFactor);

    grabberEncoder.setPosition(0);

    grabberConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(HangConstants.kP);

    grabber.configure(grabberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}

public Command setpoint() {
    return this.run(() -> target = 1);
}

@Override
public void periodic() {
    SmartDashboard.putNumber("Hang Encoder", grabberEncoder.getPosition());
    SmartDashboard.putNumber("Hang Setpoint", target);
    grabber.set(target);
}

}   