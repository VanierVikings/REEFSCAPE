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

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    private final SparkMax wristMotor;
    private final SparkMax shooterMotor;
    private final RelativeEncoder wristEncoder;

    
    public EndEffector() {
        shooterMotor = new SparkMax(EndEffectorConstants.shooterMotorID, MotorType.kBrushless);
        wristMotor = new SparkMax(EndEffectorConstants.wristMotorID, MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();

        
        SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

        wristMotorConfig.encoder
            .positionConversionFactor(EndEffectorConstants.WRIST_ENCODER_TO_DEGREES);

        wristMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(EndEffectorConstants.WRIST_KP)
            .i(EndEffectorConstants.WRIST_KI)
            .d(EndEffectorConstants.WRIST_KD);


    }
    
    
    
}