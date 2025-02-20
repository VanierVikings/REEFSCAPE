package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
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

  private final RelativeEncoder elevatorEncoder;
  private final SparkMaxConfig elevatorMotorConfig;
  // private final SparkLimitSwitch elevatorLimitSwitch;

  private double elevatorCurrentTarget;
  private double pivotCurrentTarget;


  public enum Setpoint {
    rest,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  public Elevator() {
    elevatorMotorOne = new SparkMax(ElevatorConstants.motorOneID, MotorType.kBrushless);
    elevatorMotorTwo = new SparkMax(ElevatorConstants.motorTwoID, MotorType.kBrushless);

    pivotMotorOne = new SparkMax(PivotConstants.motorOneID, MotorType.kBrushless);
    pivotMotorTwo = new SparkMax(PivotConstants.motorTwoID, MotorType.kBrushless);

    elevatorCurrentTarget = ElevatorConstants.L1;
    // placeholder L1 ? idk
    pivotCurrentTarget = PivotConstants.L1_ANGLE;
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
        .p(PivotConstants.PIVOT_KP)
        .i(PivotConstants.PIVOT_KI)
        .d(PivotConstants.PIVOT_KD)
        .outputRange(-1, 1);

    pivotMotorConfig.closedLoop.maxMotion
        .maxVelocity(PivotConstants.MAX_VELOCITY)
        .maxAcceleration(PivotConstants.MAX_ACCELERATION)
        .allowedClosedLoopError(1.0);

    pivotMotorOne.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Elevator motor configuration
    SparkMaxConfig follow = new SparkMaxConfig();
    follow.follow(9, true);
    elevatorMotorTwo.configure(follow, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    elevatorClosedLoopController = elevatorMotorOne.getClosedLoopController();

    elevatorEncoder = elevatorMotorOne.getEncoder();

    elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotorConfig.smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT);

    // elevatorLimitSwitch = elevatorMotorOne.getForwardLimitSwitch();

    // elevatorMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

    // elevatorMotorConfig.limitSwitch.fo

    elevatorMotorConfig.encoder
        .positionConversionFactor(ElevatorConstants.ENCODER_TO_METERS); // inches

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

    elevatorMotorOne.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // public void setElevatorHeight(double targetHeight) {

  // elevatorClosedLoopController.setReference(targetHeight/ElevatorConstants.ENCODER_TO_METERS,
  // ControlType.kMAXMotionPositionControl);

  // if(getAngle() <= 12 && getElevatorHeight() >= 138){

  // elevatorClosedLoopController.setReference(getElevatorHeight(),
  // ControlType.kMAXMotionPositionControl);
  // elevatorClosedLoopController.setReference(getAngle(),
  // ControlType.kMAXMotionPositionControl);
  // } // Kill code, don't know if works
  // }

  // public double getElevatorHeight() {
  // return elevatorMotorOne.getEncoder().getPosition() *
  // ElevatorConstants.ENCODER_TO_METERS;
  // }

  // public boolean checkElevatorHeight(double targetHeight, double tolerance) {
  // double currentHeight = getElevatorHeight();
  // return Math.abs(currentHeight - targetHeight) < tolerance;
  // }

  // //public void resetEncoders() {
  // // if(elevatorLimitSwitch.isPressed()) {
  // // elevatorEncoder.setPosition(0);
  // // }
  // // }

  public double getAngle() {
    return pivotMotorOne.getEncoder().getPosition();
  }

  // public boolean pivotIsAtAngle(double pivotTargetHeight, double
  // pivotTolerance) {
  // double pivotCurrentHeight = getAngle();
  // return Math.abs(pivotCurrentHeight - pivotTargetHeight) < pivotTolerance;
  // }

  // public void setAngle(double targetAngle){
  // pivotClosedLoopController.setReference(targetAngle,
  // ControlType.kMAXMotionPositionControl);

  // if(getAngle() >= 90){

  // pivotClosedLoopController.setReference(getElevatorHeight(),
  // ControlType.kMAXMotionPositionControl);
  // pivotClosedLoopController.setReference(getAngle(),
  // ControlType.kMAXMotionPositionControl);
  // }// Kill code, don't know if works
  // }

  // public void kill(){

  // if(getAngle() <= 12 && getElevatorHeight() >= 138){

  // pivotClosedLoopController.setReference(getElevatorHeight(),
  // ControlType.kMAXMotionPositionControl);
  // pivotClosedLoopController.setReference(getAngle(),
  // ControlType.kMAXMotionPositionControl);
  // }
  // //This is supposed to stop the elevator and angle. Idk if it works
  // }

  public Command moveToSetpoint() {
    return this.run(() -> {
      pivotClosedLoopController.setReference(pivotCurrentTarget, ControlType.kMAXMotionPositionControl);
    });
  }

  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case rest:
              pivotCurrentTarget = PivotConstants.L0_ANGLE;
              elevatorCurrentTarget = ElevatorConstants.L0;
              break;
            case kLevel1:
              pivotCurrentTarget = PivotConstants.L1_ANGLE;
              elevatorCurrentTarget = ElevatorConstants.L1;
              break;
            case kLevel2:
              pivotCurrentTarget = PivotConstants.L2_ANGLE;
              elevatorCurrentTarget = ElevatorConstants.L2;
              break;
            case kLevel3:
              pivotCurrentTarget = PivotConstants.L3_ANGLE;
              elevatorCurrentTarget = ElevatorConstants.L3;
              break;
          }
        });
  }

  // public Command intake() {

  // return run(
  // () -> setElevatorHeight(0))
  // .until(() -> checkElevatorHeight(0, 0.01))
  // .andThen(() -> setAngle(0))
  // .until(() -> pivotIsAtAngle(0, 0.01));
  // }

  // public Command extend() {

  // if(getAngle() >= 12){

  // return run(
  // () -> setElevatorHeight(1.3462))
  // .until(() -> checkElevatorHeight(1.3462, 0.01));
  // }
  // else{

  // return run(
  // () -> setElevatorHeight(1.27))
  // .until(() -> checkElevatorHeight(1.27, 0.01));
  // }
  // }//useless

  // public Command moveToPositionCommand() {
  // // tolerance will be tuned or whatever later
  // return run(
  // () -> setAngle(pivotCurrentTarget))
  // .until(()-> pivotIsAtAngle(pivotCurrentTarget, 0.0))
  // .andThen(() -> setElevatorHeight(elevatorCurrentTarget))
  // .until(() -> checkElevatorHeight(elevatorCurrentTarget, 0.01)); // tolerance
  // will be tuned or whatever later

  // }

  // public Command moveToL2Command() {
  // // tolerance will be tuned or whatever later
  // return run(
  // () -> setAngle(PivotConstants.L2_ANGLE))
  // .until(()-> pivotIsAtAngle(PivotConstants.L2_ANGLE, 0.0))
  // .andThen(() -> setElevatorHeight(elevatorCurrentTarget))
  // .until(() -> checkElevatorHeight(elevatorCurrentTarget, 0.01)); // tolerance
  // will be tuned or whatever later
  // }

  // public Command moveToL3Command() {
  // // tolerance will be tuned or whatever later
  // return run(
  // () -> setAngle(PivotConstants.L3_ANGLE))
  // .until(()-> pivotIsAtAngle(PivotConstants.L3_ANGLE, 0.0))
  // .andThen(() -> setElevatorHeight(elevatorCurrentTarget))
  // .until(() -> checkElevatorHeight(ElevatorConstants.L3, 0.01)); // tolerance
  // will be tuned or whatever later
  // }
}