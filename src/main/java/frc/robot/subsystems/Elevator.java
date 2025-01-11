package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;

public class Elevator extends SubsystemBase {
    private final SparkMax motorOne;
    private final SparkMax motorTwo;
    private final SparkMax PivotMotorOne;
    private final SparkMax PivotMotorTwo;
    private final SparkClosedLoopController pivotClosedLoopController;
    private final DutyCycleEncoder pivotEncoder;
    private final SparkClosedLoopController elevatorClosedLoopController;
    private final Encoder elevatorEncoder;

        private double L2 = 1.5; // This is only put here bc Andy has all the calculated constants (will have to be in meters)
    
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

        elevatorEncoder = new Encoder(0, 1);
        elevatorEncoder.setDistancePerPulse(ElevatorConstants.ENCODER_TO_METERS);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        
        

        motorConfig.closedLoop
        
            .feedbackSensor(FeedbackSensor.kAnalogSensor) 
            .p(ElevatorConstants.ELEVATOR_KP) 
            .i(ElevatorConstants.ELEVATOR_KI) 
            .d(ElevatorConstants.ELEVATOR_KD) 
            .outputRange(-1, 1); 

        motorConfig.closedLoop.maxMotion
            .maxVelocity(ElevatorConstants.MAX_VELOCITY / ElevatorConstants.ENCODER_TO_METERS) 
            .maxAcceleration(ElevatorConstants.MAX_ACCELERATION / ElevatorConstants.ENCODER_TO_METERS) 
            .allowedClosedLoopError(0.01 / ElevatorConstants.ENCODER_TO_METERS); 

        motorOne.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
}

    public void setElevatorHeight(double targetHeight) {
        double targetPosition = metersToEncoderUnits(targetHeight);

        elevatorClosedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    private double metersToEncoderUnits(double meters) {
        return meters / ElevatorConstants.ENCODER_TO_METERS;
    }

    private double encoderUnitsToMeters(double encoderUnits) {
        return encoderUnits * ElevatorConstants.ENCODER_TO_METERS;
    }

    public double getElevatorHeight() {
        return encoderUnitsToMeters(elevatorEncoder.getDistance());
    }

    public boolean isAtHeight(double targetHeight, double tolerance) {
        double currentHeight = getElevatorHeight();
        return Math.abs(currentHeight - targetHeight) < tolerance;
    }

    public void resetEncoders() {
        elevatorEncoder.reset();
    }

    public Command moveToL2Command() {
        return run(() -> setElevatorHeight(L2)).until(() -> isAtHeight(L2, 0.01)); // tolerance will be tuned or whatever later
           
    }

    public void setPivotAngle(double targetAngle){

    }   

}