package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    private final SparkMax wristMotor;
    private final SparkMax shooterMotor;
   // private final RelativeEncoder wristEncoder;
    private final SparkClosedLoopController wristClosedLoopController;

    
    public EndEffector() {
        shooterMotor = new SparkMax(EndEffectorConstants.shooterMotorID, MotorType.kBrushless);
        wristMotor = new SparkMax(EndEffectorConstants.wristMotorID, MotorType.kBrushless);
       // wristEncoder = wristMotor.getEncoder();

        SparkMaxConfig shooterMotorConfig= new SparkMaxConfig();
        shooterMotorConfig.smartCurrentLimit(EndEffectorConstants.SHOOTER_CURRENT_LIMIT);
        SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
        wristMotorConfig.smartCurrentLimit(EndEffectorConstants.WRIST_CURRENT_LIMIT);

        wristClosedLoopController = wristMotor.getClosedLoopController();

        wristMotorConfig.encoder
            .positionConversionFactor(EndEffectorConstants.WRIST_ENCODER_TO_DEGREES);

        wristMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(EndEffectorConstants.WRIST_KP)
            .i(EndEffectorConstants.WRIST_KI)
            .d(EndEffectorConstants.WRIST_KD);

    }

    public double getAngle() {
        return wristMotor.getEncoder().getPosition(); 
    }

    public boolean checkAngle(double targetAngle, double tolerance) {
        double currentAngle = getAngle();
        return Math.abs(currentAngle - targetAngle) < tolerance;
    }

    public void setAngle(double targetAngle){
        wristClosedLoopController.setReference(targetAngle, ControlType.kPosition);
    } //Set the angle of the wrist


    public void runShooter(int direction){
        shooterMotor.set(direction * EndEffectorConstants.shooterMaxVelocity);
    }

    public void stopShooter(){
        shooterMotor.set(0);
    }
    
    public void stopWrist(){
        wristMotor.set(0);
    }

    public Command intake() {
        return run(
            () -> setAngle(0))
            .until(() -> checkAngle(0, 0.01));
    }

    public Command moveToL1Command() {
        return run(
            () -> setAngle(EndEffectorConstants.wristL1_ANGLE))
             .until(() -> checkAngle(EndEffectorConstants.wristL1_ANGLE, 0.01));
    }

    public Command moveToL2Command() {
        return run(
            () -> setAngle(EndEffectorConstants.wristL2_ANGLE))
             .until(() -> checkAngle(EndEffectorConstants.wristL2_ANGLE, 0.01));
    }

    public Command moveToL3Command() {
        return run(
            () -> setAngle(EndEffectorConstants.wristL3_ANGLE))
             .until(() -> checkAngle(EndEffectorConstants.wristL3_ANGLE, 0.01));
    }

    // public Command moveToL4Command() {
    //     return run(
    //         () -> setAngle(EndEffectorConstants.wristL4_ANGLE))
    //          .until(() -> checkAngle(EndEffectorConstants.wristL4_ANGLE, 0.01));
    // }
    // If L4 eve
}