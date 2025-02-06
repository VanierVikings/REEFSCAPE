package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
public class Elevator extends SubsystemBase {
    private final SparkMax ElevatorMotorOne;
    private final SparkMax ElevatorMotorTwo;
    private final SparkMax PivotMotorOne;
    private final SparkMax PivotMotorTwo;
    private final SparkClosedLoopController pivotClosedLoopController;
    private final DutyCycleEncoder pivotEncoder;
    private final SparkClosedLoopController elevatorClosedLoopController;

    private final RelativeEncoder elevatorEncoder;
    private final SparkMaxConfig elevatorMotorConfig;
    //private final SparkLimitSwitch elevatorLimitSwitch;
    
        public Elevator() {
        ElevatorMotorOne = new SparkMax(ElevatorConstants.motorOneID, MotorType.kBrushless);
        ElevatorMotorTwo = new SparkMax(ElevatorConstants.motorTwoID, MotorType.kBrushless);
        
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
        pivotMotorConfig.smartCurrentLimit(PivotConstants.PIVOT_CURRENT_LIMIT);
        
        pivotMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(PivotConstants.PIVOT_KP)
            .i(PivotConstants.PIVOT_KI)
            .d(PivotConstants.PIVOT_KD)
            .outputRange(0,0);

        pivotMotorConfig.closedLoop.maxMotion
            .maxVelocity(PivotConstants.MAX_VELOCITY)
            .maxAcceleration(PivotConstants.MAX_ACCELERATION)
            .allowedClosedLoopError(0.0);
        PivotMotorOne.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        //Elevator motor configuration
        SparkMaxConfig follow = new SparkMaxConfig();
        follow.follow(9, true); 
        ElevatorMotorTwo.configure(follow, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        elevatorClosedLoopController = ElevatorMotorOne.getClosedLoopController();

        elevatorEncoder = ElevatorMotorOne.getEncoder();

        elevatorMotorConfig = new SparkMaxConfig();
        elevatorMotorConfig.smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT);
        
        //elevatorLimitSwitch = ElevatorMotorOne.getForwardLimitSwitch();

        //elevatorMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

        //elevatorMotorConfig.limitSwitch.fo
        
        elevatorMotorConfig.encoder
            .positionConversionFactor(ElevatorConstants.ENCODER_TO_METERS); //inches 

        elevatorMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) 
            .p(ElevatorConstants.ELEVATOR_KP) 
            .i(ElevatorConstants.ELEVATOR_KI) 
            .d(ElevatorConstants.ELEVATOR_KD) 
            .outputRange(-1, 1); 

        elevatorMotorConfig.closedLoop.maxMotion
            .maxVelocity(ElevatorConstants.MAX_VELOCITY / ElevatorConstants.ENCODER_TO_METERS) 
            .maxAcceleration(ElevatorConstants.MAX_ACCELERATION / ElevatorConstants.ENCODER_TO_METERS) 
            .allowedClosedLoopError(0.01 / ElevatorConstants.ENCODER_TO_METERS); 

        ElevatorMotorOne.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
}

    public void setElevatorHeight(double targetHeight) {

        elevatorClosedLoopController.setReference(targetHeight/ElevatorConstants.ENCODER_TO_METERS, ControlType.kMAXMotionPositionControl);
    }

    public double getElevatorHeight() {
        return ElevatorMotorOne.getEncoder().getPosition() * ElevatorConstants.ENCODER_TO_METERS;
    }

    public boolean checkElevatorHeight(double targetHeight, double tolerance) {
        double currentHeight = getElevatorHeight();
        return Math.abs(currentHeight - targetHeight) < tolerance;
    }

    
    //public void resetEncoders() {
    //     if(elevatorLimitSwitch.isPressed()) {
    //         elevatorEncoder.setPosition(0);
    //     }
    // }

    public double getAngle() {
        return PivotMotorOne.getEncoder().getPosition();
    }


    public boolean pivotIsAtAngle(double pivotTargetHeight, double pivotTolerance) {
        double pivotCurrentHeight = getAngle();
        return Math.abs(pivotCurrentHeight - pivotTargetHeight) < pivotTolerance;
    }

    public void setAngle(double targetAngle){
        pivotClosedLoopController.setReference(targetAngle, ControlType.kMAXMotionPositionControl);
    }
       
    public Command intake() {
        
        return run(
            () -> setElevatorHeight(0))
            .until(() -> checkElevatorHeight(0, 0.01))
            .andThen(() -> setAngle(0))
            .until(() -> pivotIsAtAngle(0, 0.01));  
    }

    public Command extend() {

        if(getAngle() >= 12){
        
        return run(
            () -> setElevatorHeight(1.3462))
            .until(() -> checkElevatorHeight(1.3462, 0.01));
        }
        else{

        return run(
            () -> setElevatorHeight(1.27))
            .until(() -> checkElevatorHeight(1.27, 0.01));
        }
        }

    public Command moveToL1Command() {
        // tolerance will be tuned or whatever later
        return run(
            () -> setAngle(PivotConstants.L1_ANGLE))
            .until(()-> pivotIsAtAngle(PivotConstants.L1_ANGLE, 0.0))
            .andThen(() -> setElevatorHeight(ElevatorConstants.L1))
            .until(() -> checkElevatorHeight(ElevatorConstants.L1, 0.01)); // tolerance will be tuned or whatever later
           
    }

    public Command moveToL2Command() {
        // tolerance will be tuned or whatever later
        return run(
            () -> setAngle(PivotConstants.L2_ANGLE))
            .until(()-> pivotIsAtAngle(PivotConstants.L2_ANGLE, 0.0))
            .andThen(() -> setElevatorHeight(ElevatorConstants.L2))
            .until(() -> checkElevatorHeight(ElevatorConstants.L2, 0.01)); // tolerance will be tuned or whatever later
    }

    public Command moveToL3Command() {
        // tolerance will be tuned or whatever later
        return run(
            () -> setAngle(PivotConstants.L3_ANGLE))
            .until(()-> pivotIsAtAngle(PivotConstants.L3_ANGLE, 0.0))
            .andThen(() -> setElevatorHeight(ElevatorConstants.L3))
            .until(() -> checkElevatorHeight(ElevatorConstants.L3, 0.01)); // tolerance will be tuned or whatever later 
    }
}


