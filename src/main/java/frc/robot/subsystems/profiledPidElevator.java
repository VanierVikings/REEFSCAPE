package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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


public class profiledPidElevator extends SubsystemBase{
    private final SparkMax ElevatorMotorOne;
    private final SparkMax ElevatorMotorTwo;
    private final SparkMax PivotMotorOne;
    private final SparkMax PivotMotorTwo;
    private final TrapezoidProfile.Constraints m_constraints;


    public profiledPidElevator() {
        ElevatorMotorOne = new SparkMax(ElevatorConstants.motorOneID, MotorType.kBrushless);
        ElevatorMotorTwo = new SparkMax(ElevatorConstants.motorTwoID, MotorType.kBrushless);

        PivotMotorOne = new SparkMax(PivotConstants.motorOneID, MotorType.kBrushless);
        PivotMotorTwo = new SparkMax(PivotConstants.motorTwoID, MotorType.kBrushless);

        m_constraints = new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
        
        ProfiledPIDController controller = new ProfiledPIDController(
            ElevatorConstants.ELEVATOR_KP, ElevatorConstants.ELEVATOR_KI, ElevatorConstants.ELEVATOR_KD, 
            new TrapezoidProfile.Constraints(5, 10));
            
        ProfiledPIDController PivotController = new ProfiledPIDController(
            PivotConstants.PIVOT_KP, PivotConstants.PIVOT_KI, PivotConstants.PIVOT_KD, 
            new TrapezoidProfile.Constraints(5, 10));
        
        SparkMaxConfig follow = new SparkMaxConfig();
        follow.follow(ElevatorConstants.motorOneID, true); 
        ElevatorMotorTwo.configure(follow, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


        SparkMaxConfig pivotFollow = new SparkMaxConfig();
        pivotFollow.follow(PivotConstants.motorTwoID, true);
        PivotMotorTwo.configure(pivotFollow, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


        ElevatorMotorOne.set(controller.calculate(ElevatorMotorOne.getEncoder().getPosition(), ElevatorConstants.L1));
        
        ElevatorMotorOne.set
    }
}