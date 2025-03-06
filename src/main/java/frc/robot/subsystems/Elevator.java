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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
  private final TalonFXConfiguration pivotConfig;
  final MotionMagicVelocityVoltage pivotRequest = new MotionMagicVelocityVoltage(0);
  private final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
  private final ProfiledPIDController elevatorController = new ProfiledPIDController(ElevatorConstants.kP,
      ElevatorConstants.kI, ElevatorConstants.kD, elevatorConstraints, ElevatorConstants.kDt);
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS,
      ElevatorConstants.kG, ElevatorConstants.kV);


  private final RelativeEncoder elevatorEncoder;
  private final SparkMaxConfig elevatorMotorConfig;
  private final SparkLimitSwitch elevatorLimitSwitch;

  private final IdleMode elevatorIdleMode = IdleMode.kBrake;

  public enum Setpoint {
    kRest,
    kLevel1,
    kLevel2,
    kLevel3,
    kHang,
    kSource
  }

  public Elevator() {
    pivotEncoder = new DutyCycleEncoder(1,360,0);
    elevatorController.setGoal(ElevatorConstants.L0);


    elevatorMotorOne = new SparkMax(ElevatorConstants.motorOneID, MotorType.kBrushless);
    elevatorMotorTwo = new SparkMax(ElevatorConstants.motorTwoID, MotorType.kBrushless);    
    
    pivotMotorOne = new TalonFX(PivotConstants.motorOneID);
    pivotMotorTwo = new TalonFX(PivotConstants.motorTwoID);
    // pivotMotorOne.setInverted(true); not needed because its here (below)
    pivotMotorTwo.setControl(new Follower(PivotConstants.motorOneID, true));

    pivotConfig = new TalonFXConfiguration();
    pivotConfig.Feedback.SensorToMechanismRatio = PivotConstants.positionConversionFactor;
    var slot0Configs = pivotConfig.Slot0;


    slot0Configs.kS = PivotConstants.kS;
    slot0Configs.kV = PivotConstants.kV;
    slot0Configs.kA = PivotConstants.kA; // if no need, remove
    slot0Configs.kG = PivotConstants.kG;
    slot0Configs.kP = PivotConstants.kP;
    slot0Configs.kI = PivotConstants.kI;
    slot0Configs.kD = PivotConstants.kD;
    
    var motionMagicConfigs = pivotConfig.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.MAX_VELOCITY; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = PivotConstants.MAX_ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // ??? what even is this. "jerk is the rate of change of acceleration", so like ramprate?
    
    pivotMotorOne.getConfigurator().apply(pivotConfig);
    
    m_request = new MotionMagicExpoVoltage(0);

    pivotMotorOne.setPosition(pivotEncoder.get());
    SmartDashboard.putData("Abs Encoder", pivotEncoder);
    // pivotFollow
    // .follow(11, false)
    // .smartCurrentLimit(PivotCons.MAX_ACCELERATIONtants.PIVOT_CURRENT_LIMIT)
    // .idleMode(pivotIdleMode); 
    // pivotMotorTwo.configure(pivotFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    // pivotMotorConfig
    // .smartCurrentLimit(PivotConstants.PIVOT_CURRENT_LIMIT)
    // .idleMode(pivotIdleMode)
    // .inverted(false);

    // pivotMotorConfig.encoder.positionConversionFactor(PivotConstants.positionConversionFactor);
    // pivotMotorOne.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


// might be useful

//     Timer.delay(1);
// this.krakenConfiguration = new TalonFXConfiguration();

// krakenConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
// krakenConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

// krakenConfiguration.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
// krakenConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
// krakenConfiguration.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;
// krakenConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
// krakenConfiguration.CurrentLimits.SupplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;

// Slot0Configs krakenSlot0Configs = krakenConfiguration.Slot0;
// krakenSlot0Configs.kS = swerveModuleConstants.getDriveKS();
// krakenSlot0Configs.kV = swerveModuleConstants.getDriveKV();
// krakenSlot0Configs.kP = swerveModuleConstants.getDriveKP();
// krakenSlot0Configs.kI = swerveModuleConstants.getDriveKI();
// krakenSlot0Configs.kD = swerveModuleConstants.getDriveKD();

// driveMotor.getConfigurator().apply(krakenConfiguration);

// driveMotor.getVelocity().setUpdateFrequency(100);
// driveMotor.getPosition().setUpdateFrequency(100);

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
    .idleMode(elevatorIdleMode);

    elevatorLimitSwitch = elevatorMotorOne.getForwardLimitSwitch();

    elevatorMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

    elevatorMotorConfig.encoder
        .positionConversionFactor(ElevatorConstants.ENCODER_TO_METERS); // inches

    elevatorMotorOne.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorController.setTolerance(0.0001);

    
  }

  public Command moveToSetpoint() {
  return this.run(() -> {
  elevatorMotorOne.setVoltage(
  elevatorController.calculate(elevatorEncoder.getPosition())
  + elevatorFeedforward.calculate(elevatorController.getSetpoint().velocity));
  });

  }


  public Command setPivot(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            // case kRest:
            //   pivotMotorOne.setControl(m_request.withPosition(PivotConstants.L0_ANGLE)); //degrees
            //   break;
            // case kLevel1:
            //   pivotMotorOne.setControl(m_request.withPosition(PivotConstants.L1_ANGLE)); 
            //   break;
            // case kLevel2:
            //   pivotMotorOne.setControl(m_request.withPosition(PivotConstants.L2_ANGLE));
            //   break;
            // case kLevel3:
            //   pivotMotorOne.setControl(m_request.withPosition(PivotConstants.L3_ANGLE));
            //   break;
            // case kHang:
            //   pivotMotorOne.setControl(m_request.withPosition(PivotConstants.HANG_ANGLE));
            //   break;
            // case kSource:
            //   pivotMotorOne.setControl(m_request.withPosition(PivotConstants.SOURCE_ANGLE));
            //   break;
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
    rezeroElevator();
    SmartDashboard.putNumber("Elevator Setpoint Velocity", elevatorController.getSetpoint().velocity);
    SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Setpoint Position", elevatorController.getGoal().position);
    SmartDashboard.putNumber("Elevator Error", elevatorController.getPositionError());

    SmartDashboard.putNumber("Pivot Setpoint Velocity", pivotMotorOne.getClosedLoopReferenceSlope().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Position", pivotMotorOne.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Velocity", pivotMotorOne.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Setpoint Position", pivotMotorOne.getClosedLoopReference().getValueAsDouble()); 
  }
}