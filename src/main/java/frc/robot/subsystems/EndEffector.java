package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    private final SparkMax wristMotor;
    private final SparkMax shooterMotor;
    private final RelativeEncoder wristEncoder;
    private final SparkClosedLoopController wristClosedLoopController;
    private final SparkClosedLoopController shooterClosedLoopController;

    
    public EndEffector() {
        shooterMotor = new SparkMax(EndEffectorConstants.shooterMotorID, MotorType.kBrushless);
        wristMotor = new SparkMax(EndEffectorConstants.wristMotorID, MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();

        
        SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

        wristClosedLoopController = wristMotor.getClosedLoopController();
        shooterClosedLoopController = shooterMotor.getClosedLoopController();

        wristMotorConfig.encoder
            .positionConversionFactor(EndEffectorConstants.WRIST_ENCODER_TO_DEGREES);

        wristMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(EndEffectorConstants.WRIST_KP)
            .i(EndEffectorConstants.WRIST_KI)
            .d(EndEffectorConstants.WRIST_KD);

    }

    public double getAngle(){
        return wristMotor.getEncoder().getPosition(); 
    }

    public boolean atAngle(){
        return getAngle() == EndEffectorConstants.wristL1_Angle;
    }

    public void setAngle(double targetAngle){
        wristClosedLoopController.setReference(targetAngle, ControlType.kMAXMotionPositionControl);
    } //Set the angle of the wrist


    public void runShooter(double speed){
        shooterClosedLoopController.setReference(speed, ControlType.kVelocity);//idk the control type
    }
    
    public Command wristL1PosCommand(){
        return run( ()-> setAngle(EndEffectorConstants.wristL1_Angle))
        .until(()-> atAngle())
        .andThen(runShooter(speed)).withTimeout(0.110); //speed unknown for now
    }
    
}