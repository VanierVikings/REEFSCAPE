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
import frc.robot.Constants.PivotConstants;

import static java.lang.Math.PI;

public class Elevator extends SubsystemBase {
    private final SparkMax motorOne;
    private final SparkMax motorTwo;
    private final SparkMax PivotMotorOne;
    private final SparkMax PivotMotorTwo;
    private final SparkClosedLoopController pivotClosedLoopController;
    private final DutyCycleEncoder pivotEncoder;
    private final SparkClosedLoopController elevatorClosedLoopController;
    private final RelativeEncoder elevatorEncoder;
    private final SparkMaxConfig elevatorMotorConfig;
    private final LimitSwitchConfig elevatorLimitSwitch;
    
        public Elevator() {
        motorOne = new SparkMax(ElevatorConstants.motorOneID, MotorType.kBrushless);
        motorTwo = new SparkMax(ElevatorConstants.motorTwoID, MotorType.kBrushless);
        
        PivotMotorOne = new SparkMax(PivotConstants.motorOneID, MotorType.kBrushless);
        PivotMotorTwo = new SparkMax(PivotConstants.motorTwoID, MotorType.kBrushless);
        

        //Pivot motor configuration
        SparkMaxConfig pivotFollow = new SparkMaxConfig();
        pivotFollow.follow(11, true);
        PivotMotorTwo.configure(pivotFollow, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        pivotClosedLoopController = PivotMotorOne.getClosedLoopController();

        pivotEncoder = new DutyCycleEncoder(0);
        pivotEncoder.setDutyCycleRange(0, 0);
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        
        pivotMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(PivotConstants.PIVOT_KP)
            .i(PivotConstants.PIVOT_KI)
            .d(PivotConstants.PIVOT_KD)
            .outputRange(0,0);

        pivotMotorConfig.closedLoop.maxMotion
            .maxVelocity(PivotConstants.MAX_VELOCITY/ PivotConstants.ENCODER_TO_DEGREES)
            .maxAcceleration(PivotConstants.MAX_ACCELERATION/ PivotConstants.ENCODER_TO_DEGREES)
            .allowedClosedLoopError(0.0/ PivotConstants.ENCODER_TO_DEGREES);
        PivotMotorOne.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        
        
        //how to make motorTwo follow motorOne?? does this work?
        //Elevator motor configuration
        SparkMaxConfig follow = new SparkMaxConfig();
        follow.follow(9, true); 
        motorTwo.configure(follow, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        elevatorClosedLoopController = motorOne.getClosedLoopController();

        elevatorEncoder = motorOne.getEncoder();

        elevatorMotorConfig = new SparkMaxConfig();
        
        elevatorLimitSwitch = elevatorMotorConfig.limitSwitch;
        elevatorLimitSwitch.forwardLimitSwitchEnabled(true);
        
        elevatorMotorConfig.encoder
            .positionConversionFactor(20*Math.PI*((0.85*0.85)) ); //inches for now


        elevatorMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
            .p(ElevatorConstants.ELEVATOR_KP) 
            .i(ElevatorConstants.ELEVATOR_KI) 
            .d(ElevatorConstants.ELEVATOR_KD) 
            .outputRange(-1, 1); 

        elevatorMotorConfig.closedLoop.maxMotion
            .maxVelocity(ElevatorConstants.MAX_VELOCITY / ElevatorConstants.ENCODER_TO_METERS) 
            .maxAcceleration(ElevatorConstants.MAX_ACCELERATION / ElevatorConstants.ENCODER_TO_METERS) 
            .allowedClosedLoopError(0.01 / ElevatorConstants.ENCODER_TO_METERS); 

        motorOne.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
}

    public void setElevatorHeight(double targetHeight) {

        elevatorClosedLoopController.setReference(targetHeight/ElevatorConstants.ENCODER_TO_METERS, ControlType.kMAXMotionPositionControl);
    }

    public double getElevatorHeight() {
        return motorOne.getEncoder().getPosition() * ElevatorConstants.ENCODER_TO_METERS;
    }

    public boolean isAtHeight(double targetHeight, double tolerance) {
        double currentHeight = getElevatorHeight();
        return Math.abs(currentHeight - targetHeight) < tolerance;
    }

    
    //public void resetEncoders() {
    //    if() {
    //        elevatorEncoder.setPosition(0);
    //    }
    //}

    public Command moveToL2Command() {
        return run(() -> setElevatorHeight(ElevatorConstants.L2)).until(() -> isAtHeight(ElevatorConstants.L2, 0.01)); // tolerance will be tuned or whatever later
           
    }

    public void setPivotAngle(double targetAngle){

    }   

}