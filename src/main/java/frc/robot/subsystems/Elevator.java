package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;

public class Elevator extends SubsystemBase {
  private final DutyCycleEncoder pivotEncoder;
  private final SparkMax elevatorMotorOne;
  private final SparkMax elevatorMotorTwo;
  private final TalonFX pivotMotorOne;
  private final TalonFX pivotMotorTwo;
  private final MotionMagicExpoVoltage m_request;
  private final MotionMagicVoltage climb_request;
  private final TalonFXConfiguration pivotConfig;
  final MotionMagicVelocityVoltage pivotRequest = new MotionMagicVelocityVoltage(0);
  private final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
  public final ProfiledPIDController elevatorController = new ProfiledPIDController(ElevatorConstants.kP,
      ElevatorConstants.kI, ElevatorConstants.kD, elevatorConstraints, ElevatorConstants.kDt);
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS,
      ElevatorConstants.kG, ElevatorConstants.kV);


  private final RelativeEncoder elevatorEncoder;
  private final SparkMaxConfig elevatorMotorConfig;
  private final SparkLimitSwitch elevatorLimitSwitch;
  public double desiredAngle;

  private final IdleMode elevatorIdleMode = IdleMode.kBrake;

  public enum Setpoint {
    kClimbDown,
    kRest,
    kLevel1,
    kLevel2,
    kLevel3,
    kHang,
    kSource,
    KAlgaeLowStart,
    KAlgaeLowEnd,
    KAlgaeHighStart,
    KAlgaeHighEnd
  }

  public Elevator() {
    desiredAngle = PivotConstants.L0_ANGLE/360;
    pivotEncoder = new DutyCycleEncoder(1,360,0);
    elevatorController.setGoal(ElevatorConstants.L0);

    elevatorMotorOne = new SparkMax(ElevatorConstants.motorOneID, MotorType.kBrushless);
    elevatorMotorTwo = new SparkMax(ElevatorConstants.motorTwoID, MotorType.kBrushless);    
    
    pivotMotorOne = new TalonFX(PivotConstants.motorOneID);
    pivotMotorTwo = new TalonFX(PivotConstants.motorTwoID);
    pivotMotorTwo.setControl(new Follower(PivotConstants.motorOneID, true));

    pivotConfig = new TalonFXConfiguration();
    pivotConfig.ClosedLoopGeneral.ContinuousWrap = true;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfig.Feedback.SensorToMechanismRatio = (4*3*34/18*52/18*46/12);
    var slot0Configs = pivotConfig.Slot0;


    slot0Configs.kS = PivotConstants.kS;
    slot0Configs.kV = PivotConstants.kV;
    slot0Configs.kA = PivotConstants.kA; // if no need, remove
    slot0Configs.kG = PivotConstants.kG;
    slot0Configs.kP = PivotConstants.kP;
    slot0Configs.kI = PivotConstants.kI;
    slot0Configs.kD = PivotConstants.kD;
    
    var motionMagicConfigs = pivotConfig.MotionMagic;

    // pivotCurrentLimitConfigs.StatorCurrentLimit = 160;

    motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.MAX_VELOCITY; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = PivotConstants.MAX_ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 30; // ??? what even is this. "jerk is the rate of change of acceleration", so like ramprate?


    pivotMotorOne.getConfigurator().apply(pivotConfig);
    
    m_request = new MotionMagicExpoVoltage(0);
    climb_request = new MotionMagicVoltage(0);

    pivotMotorOne.setPosition(0);

    // Elevator motor configuration
    SparkMaxConfig elevatorFollow = new SparkMaxConfig();
    elevatorFollow
    .follow(9, true)
    .idleMode(elevatorIdleMode)
    .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT);
    elevatorMotorTwo.configure(elevatorFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorEncoder = elevatorMotorOne.getEncoder();

    elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotorConfig
    .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT)
    .inverted(false)
    .idleMode(elevatorIdleMode);

    elevatorLimitSwitch = elevatorMotorOne.getForwardLimitSwitch();

    elevatorMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

    elevatorMotorConfig.encoder
        .positionConversionFactor(ElevatorConstants.ENCODER_TO_METERS); // inches

    elevatorMotorOne.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorController.setTolerance(0.0001);

    SmartDashboard.putData("Elevator Controller", elevatorController);
  }

  public void moveToSetpoint() {
  elevatorMotorOne.setVoltage(
  elevatorController.calculate(elevatorEncoder.getPosition())
  + elevatorFeedforward.calculate(elevatorController.getSetpoint().velocity));
  }


  public Command setPivot(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) { 
            case kClimbDown:
              pivotMotorOne.setControl(climb_request.withPosition(PivotConstants.CLIMB_ANGLE/360));
              break;
            case kRest:
            desiredAngle = PivotConstants.L0_ANGLE/360;
              break;
            case kLevel1:
            desiredAngle = PivotConstants.L1_ANGLE/360;
              break;
            case kLevel2:
            desiredAngle = PivotConstants.L2_ANGLE/360;
              break;
            case kLevel3:
            desiredAngle = PivotConstants.L3_ANGLE/360;
              break;
            case kHang:
            desiredAngle = PivotConstants.HANG_ANGLE/360;
              break;
            case kSource:
            desiredAngle = PivotConstants.SOURCE_ANGLE/360;
              break;
            case KAlgaeLowStart:
            desiredAngle = PivotConstants.ALGAE_ANGLE_LOW_START/360;
              break;
            case KAlgaeLowEnd:
            desiredAngle = PivotConstants.ALGAE_ANGLE_LOW_END/360;              
              break;
            case KAlgaeHighStart:
            desiredAngle = PivotConstants.ALGAE_ANGLE_HIGH_START/360;   
              break;
            case KAlgaeHighEnd:
            desiredAngle = PivotConstants.ALGAE_ANGLE_HIGH_END/360;   
              break;
          }
        });
  }

  public Command setElevator(Setpoint setpoint) {
    return this.runOnce(
      () -> {
        switch (setpoint) {
          case kRest:
            elevatorController.setGoal(ElevatorConstants.L0);
            break;
          case kLevel1:
            elevatorController.setGoal(ElevatorConstants.L1);
            break;
          case kLevel2:
            elevatorController.setGoal(ElevatorConstants.L2);
            break;
          case kLevel3:
            elevatorController.setGoal(ElevatorConstants.L3);
            break;
          case kSource:
            elevatorController.setGoal(ElevatorConstants.SOURCE);
            break;
          case KAlgaeLowStart:
            elevatorController.setGoal(ElevatorConstants.ALGAE_HEIGHT_LOW_START);
            break;
          case KAlgaeLowEnd:
            elevatorController.setGoal(ElevatorConstants.ALGAE_HEIGHT_LOW_END);
            break;
            case KAlgaeHighStart:
            elevatorController.setGoal(ElevatorConstants.ALGAE_HEIGHT_HIGH_START);
            break;
          case KAlgaeHighEnd:
            elevatorController.setGoal(ElevatorConstants.ALGAE_HEIGHT_HIGH_END);
            break;
        }
      });
  }

  private void rezeroElevator() {
    if (elevatorLimitSwitch.isPressed()) {
      elevatorEncoder.setPosition(0);
    }
  }

  @Override
  public void periodic() {
    pivotMotorOne.setControl(m_request.withPosition(desiredAngle));

    rezeroElevator();
    moveToSetpoint();
    SmartDashboard.putNumber("Elevator Setpoint Velocity", elevatorController.getSetpoint().velocity);
    SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Setpoint Position", elevatorController.getGoal().position);
    SmartDashboard.putNumber("Elevator Error", elevatorController.getPositionError());
    SmartDashboard.putNumber("Abs Encoder", pivotEncoder.get());
    SmartDashboard.putNumber("Pivot Setpoint Velocity", pivotMotorOne.getClosedLoopReferenceSlope().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Position turn", pivotMotorOne.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Position degrees", pivotMotorOne.getPosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Pivot Velocity", pivotMotorOne.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Setpoint Position", pivotMotorOne.getClosedLoopReference().getValueAsDouble()); 
  }
}