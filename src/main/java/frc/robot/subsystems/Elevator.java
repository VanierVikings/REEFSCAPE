package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
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
  private final SparkMax elevatorMotorOne;
  private final SparkMax elevatorMotorTwo;
  private final SparkMax pivotMotorOne;
  private final SparkMax pivotMotorTwo;
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0, 360, 0);
  private final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
  private final ProfiledPIDController elevatorController = new ProfiledPIDController(ElevatorConstants.kP,
      ElevatorConstants.kI, ElevatorConstants.kD, elevatorConstraints, ElevatorConstants.kDt);
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS,
      ElevatorConstants.kG, ElevatorConstants.kV);

  private final TrapezoidProfile.Constraints pivotConstraints = new TrapezoidProfile.Constraints(
      PivotConstants.MAX_VELOCITY, PivotConstants.MAX_ACCELERATION);
  private final ProfiledPIDController pivotController = new ProfiledPIDController(PivotConstants.kP, PivotConstants.kI,
      PivotConstants.kD, pivotConstraints, PivotConstants.kDt);
  private final ArmFeedforward pivotFeedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG,
      PivotConstants.kV);

  private final RelativeEncoder elevatorEncoder;
  private final SparkMaxConfig elevatorMotorConfig;
  private final SparkLimitSwitch elevatorLimitSwitch;

  public double elevatorCurrentTarget = ElevatorConstants.L0;
  private double pivotCurrentTarget = PivotConstants.L0_ANGLE;

  public enum Setpoint {
    kRest,
    kLevel1,
    kLevel2,
    kLevel3,
    kHang,
    kSource
  }

  public Elevator() {
    elevatorMotorOne = new SparkMax(ElevatorConstants.motorOneID, MotorType.kBrushless);
    elevatorMotorTwo = new SparkMax(ElevatorConstants.motorTwoID, MotorType.kBrushless);

    pivotMotorOne = new SparkMax(PivotConstants.motorOneID, MotorType.kBrushless);
    pivotMotorTwo = new SparkMax(PivotConstants.motorTwoID, MotorType.kBrushless);

    SparkMaxConfig pivotFollow = new SparkMaxConfig();
    pivotFollow
    .follow(11, false)
    .smartCurrentLimit(PivotConstants.PIVOT_CURRENT_LIMIT)
    .voltageCompensation(12)
    .idleMode(IdleMode.kBrake);
    pivotMotorTwo.configure(pivotFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig
    .smartCurrentLimit(PivotConstants.PIVOT_CURRENT_LIMIT)
    .idleMode(IdleMode.kBrake)
    .inverted(true)
    .voltageCompensation(12);

    pivotMotorOne.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Elevator motor configuration
    SparkMaxConfig elevatorFollow = new SparkMaxConfig();
    elevatorFollow
    .follow(9, true)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT);
    elevatorMotorTwo.configure(elevatorFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorEncoder = elevatorMotorOne.getEncoder();

    elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotorConfig
    .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT)
    .idleMode(IdleMode.kBrake);

    elevatorLimitSwitch = elevatorMotorOne.getForwardLimitSwitch();

    elevatorMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

    elevatorMotorConfig.encoder
        .positionConversionFactor(ElevatorConstants.ENCODER_TO_METERS); // inches

    elevatorMotorOne.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorController.setTolerance(0.0001);

    elevatorController.setGoal(ElevatorConstants.L0);
    pivotController.setGoal(PivotConstants.L0_ANGLE);
  }

  public Command moveToSetpoint() {
  return this.run(() -> {
  elevatorMotorOne.setVoltage(
  elevatorController.calculate(elevatorEncoder.getPosition())
  + elevatorFeedforward.calculate(elevatorController.getSetpoint().velocity));

  pivotMotorOne.setVoltage(
  pivotController.calculate(pivotEncoder.get())
  + pivotFeedforward.calculate(pivotController.getSetpoint().position,
  pivotController.getSetpoint().velocity));

  });
  }

  public Command setPivot(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kRest:
              pivotController.setGoal(PivotConstants.L0_ANGLE);
              break;
            case kLevel1:
              pivotController.setGoal(PivotConstants.L1_ANGLE);
              break;
            case kLevel2:
              pivotController.setGoal(PivotConstants.L2_ANGLE);
              break;
            case kLevel3:
              pivotController.setGoal(PivotConstants.L3_ANGLE);
              break;
            case kHang:
              pivotController.setGoal(PivotConstants.HANG_ANGLE);
              break;
            case kSource:
              elevatorController.setGoal(PivotConstants.SOURCE_ANGLE);
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

    SmartDashboard.putNumber("Pivot Setpoint Velocity", pivotController.getGoal().velocity);
    SmartDashboard.putNumber("Pivot Velocity", pivotController.getSetpoint().velocity);
    SmartDashboard.putNumber("Pivot Position", pivotEncoder.get());
    SmartDashboard.putNumber("Pivot Setpoint Position", pivotController.getGoal().position);
    SmartDashboard.putNumber("Pivot Voltage", pivotMotorOne.getBusVoltage());
  }
}