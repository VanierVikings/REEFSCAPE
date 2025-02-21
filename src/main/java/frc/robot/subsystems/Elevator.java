package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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
  private final SparkMax elevatorMotorOne;
  private final SparkMax elevatorMotorTwo;
  private final SparkMax pivotMotorOne;
  private final SparkMax pivotMotorTwo;
  private final SparkClosedLoopController pivotClosedLoopController;
  private final SparkAbsoluteEncoder pivotEncoder;
  private final SparkClosedLoopController elevatorClosedLoopController;
  private final TrapezoidProfile.Constraints elevatorConstraints =
      new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
  private final ProfiledPIDController elevatorController =
      new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, elevatorConstraints, ElevatorConstants.kDt);
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

  private final TrapezoidProfile.Constraints pivotConstraints =
      new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
  private final ProfiledPIDController pivotController =
      new ProfiledPIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, pivotConstraints, PivotConstants.kDt);
  private final ArmFeedforward pivotFeedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);

  private final RelativeEncoder elevatorEncoder;
  private final SparkMaxConfig elevatorMotorConfig;
  private final SparkLimitSwitch elevatorLimitSwitch;

  private double elevatorCurrentTarget;
  private double pivotCurrentTarget;


  public enum Setpoint {
    rest,
    kLevel1,
    kLevel2,
    kLevel3,
    kHang;
  }

  public Elevator() {
    elevatorMotorOne = new SparkMax(ElevatorConstants.motorOneID, MotorType.kBrushless);
    elevatorMotorTwo = new SparkMax(ElevatorConstants.motorTwoID, MotorType.kBrushless);

    pivotMotorOne = new SparkMax(PivotConstants.motorOneID, MotorType.kBrushless);
    pivotMotorTwo = new SparkMax(PivotConstants.motorTwoID, MotorType.kBrushless);

    elevatorCurrentTarget = ElevatorConstants.L0;
    // placeholder L1 ? idk
    pivotCurrentTarget = PivotConstants.L0_ANGLE;
    // Same shi same shi

    // Pivot motor configuration
    SparkMaxConfig pivotFollow = new SparkMaxConfig();
    pivotFollow.follow(11, true);
    pivotMotorTwo.configure(pivotFollow, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotClosedLoopController = pivotMotorOne.getClosedLoopController();

    pivotEncoder = pivotMotorOne.getAbsoluteEncoder();
    SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig.smartCurrentLimit(PivotConstants.PIVOT_CURRENT_LIMIT);

    pivotMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(PivotConstants.kP)
        .i(PivotConstants.kI)
        .d(PivotConstants.kD)
        .outputRange(-1, 1);

    pivotMotorConfig.closedLoop.maxMotion
        .maxVelocity(PivotConstants.MAX_VELOCITY)
        .maxAcceleration(PivotConstants.MAX_ACCELERATION)
        .allowedClosedLoopError(1.0);

    pivotMotorOne.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Elevator motor configuration
    SparkMaxConfig follow = new SparkMaxConfig();
    follow.follow(9, true);
    elevatorMotorTwo.configure(follow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorClosedLoopController = elevatorMotorOne.getClosedLoopController();

    elevatorEncoder = elevatorMotorOne.getEncoder();

    elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotorConfig.smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT);

    elevatorLimitSwitch = elevatorMotorOne.getForwardLimitSwitch();

    elevatorMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

    elevatorMotorConfig.encoder
        .positionConversionFactor(ElevatorConstants.ENCODER_TO_METERS); // inches

    elevatorMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ElevatorConstants.kP)
        .i(ElevatorConstants.kI)
        .d(ElevatorConstants.kD)
        .outputRange(-1, 1);

    elevatorMotorConfig.closedLoop.maxMotion
        .maxVelocity(ElevatorConstants.MAX_VELOCITY / ElevatorConstants.ENCODER_TO_METERS)
        .maxAcceleration(ElevatorConstants.MAX_ACCELERATION / ElevatorConstants.ENCODER_TO_METERS)
        .allowedClosedLoopError(0.01 / ElevatorConstants.ENCODER_TO_METERS);

    elevatorMotorOne.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getAngle() {
    return pivotMotorOne.getEncoder().getPosition();
  }

  public Command moveToSetpoint() {
    return this.run(() -> {
      pivotClosedLoopController.setReference(pivotCurrentTarget, ControlType.kMAXMotionPositionControl);
      elevatorClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    });
  }

  // public Command moveToSetpoint() {
  //   return this.run(() -> {
  //     elevatorMotorOne.setVoltage(
  //       elevatorController.calculate(elevatorEncoder.getPosition())
  //           + elevatorFeedforward.calculate(elevatorController.getSetpoint().velocity));

  //       pivotMotorOne.setVoltage(
  //         pivotController.calculate(pivotEncoder.getPosition())
  //           + pivotFeedforward.calculate(pivotController.getSetpoint().position, pivotController.getSetpoint().velocity));

  //   });
  // }

  public Command setPivot(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case rest:
              pivotCurrentTarget = PivotConstants.L0_ANGLE;
              pivotController.setGoal(PivotConstants.L0_ANGLE);
              break;
            case kLevel1:
              pivotCurrentTarget = PivotConstants.L1_ANGLE;
              pivotController.setGoal(PivotConstants.L1_ANGLE);
              break;
            case kLevel2:
              pivotCurrentTarget = PivotConstants.L2_ANGLE;
              pivotController.setGoal(PivotConstants.L2_ANGLE);
              break;
            case kLevel3:
              pivotCurrentTarget = PivotConstants.L3_ANGLE;
              pivotController.setGoal(PivotConstants.L3_ANGLE);
              break;
            case kHang:
            pivotCurrentTarget = PivotConstants.LHANG_ANGLE;
            pivotController.setGoal(PivotConstants.LHANG_ANGLE);
            break;
          }
        });
  }

  public Command setElevator(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case rest:
              elevatorCurrentTarget = ElevatorConstants.L0;
              elevatorController.setGoal(ElevatorConstants.L0);
              break;
            case kLevel1:
              elevatorCurrentTarget = ElevatorConstants.L1;
              elevatorController.setGoal(ElevatorConstants.L1);
              break;
            case kLevel2:
              elevatorCurrentTarget = ElevatorConstants.L2;
              elevatorController.setGoal(ElevatorConstants.L2);
              break;
            case kLevel3:
              elevatorCurrentTarget = ElevatorConstants.L3;
              elevatorController.setGoal(ElevatorConstants.L3);
              break;
          }
        });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getPosition());
  }
}